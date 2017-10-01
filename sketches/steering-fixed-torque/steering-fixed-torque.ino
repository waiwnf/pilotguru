// Proof of concept steering by wire module for KIA Cee'd JD 2015 model year,
// steering column assembly part No 56310A2001. Possibly will work on other
// KIA/Hyundai vehicles with similar electric power steering setups (though not
// tested).
// Tested on Arduino UNO with two PCF8591-based YL-40 AD/DA converters.
//
// !!IMPORTANT!! Remove the R4 resistor (1 KOhm) from both YL-40 modules as the
// current leak via the R4 and connected LED caps the DAC output voltage at
// ~4.2V instead of the nominal 5V. See
// https://forum.arduino.cc/index.php?topic=333871.0 for more details with a
// similar module and
// http://imrad.com.ua/userdata/modules/wproducts/wprod_products/135743/Arduino%20YL-40.pdf
// for YL-40 schematics.
//
// YL-40 does not allow the PCF8591 module I2C address to change without
// re-soldering, so the two modules need to be connected to different I2C buses
// on the Arduino.
//
// Module 1 ("green"): stock Arduino I2C bus - SDA pin A4, SCL pin A5.
// Module 2 ("blue"): I2C bus managed by SoftI2CMaster lib - SDA A0, SCL A1.
//
// Each YL-40 module is spliced into the corresponding color torque signal wire
// (from the 6-wire bunch between the steering angle and torque sensor and the
// electrical power steering ECU). Connect the sensor wire to the AIN3 pin on
// its YL-40 module, and the corresponding ECU input to the AOUT on YL-40. This
// way the YL-40 can forward and modify the steering torque signal to the ECU
// and "trick" the power steering ECU into amplifying the simulated steering
// signal, hence turning the steering wheel.
//
// See https://github.com/PolySync/oscc/wiki/Steering for more explanations with
// a very similar KIA SOUL power steering module.

#define DEBUG_PRINT_RAW_TORQUE_IN_VOLTAGE false
#define DEBUG_PRINT_AVG_TORQUE_IN_VOLTAGE false
#define DEBUG_PRINT_SMOOTH_TORQUE_IN_VOLTAGE false
#define DEBUG_PRINT_TORQUE_OUT_VOLTAGE true
#define DEBUG_STEPS_PRINT_EVERY 100

// Raise the stock Arduino I2C interface speed to 400MHz from the 100MHz default
// to speed thins up a bit.
#define TWI_FREQ 400000L

#include <assert.h>

// Stock Arduino I2C library.
#include <Wire.h>

// Configure the second I2C interface with the SoftI2CMaster library to use pins
// A0 and A1 as SDA and SCL.
#define SCL_PIN 1
#define SCL_PORT PORTC
#define SDA_PIN 0
#define SDA_PORT PORTC
#include <SoftI2CMaster.h>

// I2C address of the PCF8591 chip on the YL-40 module
#define PCF8591_ID 0x48

// PCF8591 control byte components. See the datasheet for more details:
// https://www.nxp.com/docs/en/data-sheet/PCF8591.pdf

// Enable the DAC output. Expects a second byte immediately with the output
// level.
#define DAC_ENABLE 0x40
// Use ADC AIN channel 3 for reading the voltage level. Following reads from
// the PCF8591 will return the voltage on this channel.
#define AIN3_ID 0x3

// Voltage measurement postprocessing settings.
// Because PCF8591 is only 8 bit for both AD and DA conversions, even 1-bit
// jitter is noticeable by the power steering ECU when forwarding the signal
// from the torque sensors. To reduce the jitter, we use two tricks:
// 1. Maintain a running average of the torque sensor voltage measurements
//    instead of using only one most recent measurement.
// 2. Change the output voltage level only if the input running average moves
//    away by X from the current output level ("hystheresis").

// Use 2^VOLTAGE_BUFFER_AVERAGING_SHIFT most recent measurements for the running
// average of the torque sensor voltage. This way the running average is simply
// a bit shift of the sum of the individual measurements.
#define VOLTAGE_BUFFER_AVERAGING_SHIFT 4
// Buffer size to hold the individual measurements for computing the running
// average.
#define VOLTAGE_READINGS_BUFFER_SIZE (1 << VOLTAGE_BUFFER_AVERAGING_SHIFT)
// Only change the output voltage when the input running average is more than
// VOLTAGE_UPDATE_HYSTHERESIS from the current output.
#define VOLTAGE_UPDATE_HYSTHERESIS 1

struct SteeringSpoofSettings {
  // Maximum extra spoof torque voltage signal for a turn command, in LSB units
  // of the DAC module.
  int8_t max_steering_magnitude;

  // The spoof torque voltage is changed gradually in 1 LSB increments to avoid
  // sharp voltage jumps that the ECU may perceive as sensor faults. Spend this
  // many loop() cycles between consecutive voltage changes.
  uint16_t steps_per_adjustment_level;

  // Spend this many loop() cycles at STEERING_MAGNITUDE spoof torque level,
  // once
  // it has been reached with consecutive 1 LSB increments (with
  // STEERING_MAGNITUDE between each increment).
  // After this many loop() cycles pass, the spoof torque extra voltage is
  // gradually reduced back to zero.
  // Overall loop() frequency for this sketch is on the order of 1KHz.
  uint16_t steps_at_target_level;
};

// Read the current voltage level on the green wire
uint8_t read_green_voltage() {
  // The ADC makes a fresh measurement only once the previously measured value
  // is read from the cheap. So the first result in the read session may be
  // arbitrarily old. Ignore the first result and only use the first read to
  // trigger a fresh measurement.
  Wire.requestFrom(PCF8591_ID, 2 /* num requested bytes */);
  Wire.read();
  // Read again to receive the just result of the measurement triggered right
  // above.
  return Wire.read();
}

// Read the current voltage level on the blue wire
uint8_t read_blue_voltage() {
  assert(i2c_start(PCF8591_ID << 1 | I2C_READ));
  // The ADC makes a fresh measurement only once the previously measured value
  // is read from the cheap. So the first result in the read session may be
  // arbitrarily old. Ignore the first result and only use the first read to
  // trigger a fresh measurement.
  i2c_read(false /* last read */);
  // Read again to receive the just result of the measurement triggered right
  // above.
  const uint8_t blue_voltage = i2c_read(true /* last read */);
  i2c_stop();
  return blue_voltage;
}

void update_voltage_running_total(uint8_t buffer[], int buffer_index,
                                  int *running_total, uint8_t new_value) {
  (*running_total) -= buffer[buffer_index];
  buffer[buffer_index] = new_value;
  (*running_total) += buffer[buffer_index];
}

class HistoricVoltageData {
public:
  void take_measurement() {
    voltage_buffer_idx =
        (voltage_buffer_idx + 1) % VOLTAGE_READINGS_BUFFER_SIZE;
    update_voltage_running_total(green_voltage_readings, voltage_buffer_idx,
                                 &total_green_voltage, read_green_voltage());
    update_voltage_running_total(blue_voltage_readings, voltage_buffer_idx,
                                 &total_blue_voltage, read_blue_voltage());
  }

  uint8_t get_avg_green_voltage() const {
    return (total_green_voltage >> VOLTAGE_BUFFER_AVERAGING_SHIFT);
  }

  uint8_t get_avg_blue_voltage() const {
    return (total_blue_voltage >> VOLTAGE_BUFFER_AVERAGING_SHIFT);
  }

  uint8_t get_latest_green_voltage() const {
    return green_voltage_readings[voltage_buffer_idx];
  }

  uint8_t get_latest_blue_voltage() const {
    return blue_voltage_readings[voltage_buffer_idx];
  }

private:
  // Individual torque sensor voltage measurement buffers, necessary for
  // computing
  // the running averages.
  uint8_t green_voltage_readings[VOLTAGE_READINGS_BUFFER_SIZE];
  uint8_t blue_voltage_readings[VOLTAGE_READINGS_BUFFER_SIZE];
  // Wraparound index of which sensor voltage measurement buffer element to use
  // next.
  uint16_t voltage_buffer_idx = 0;
  // Running sums of the individual voltage measurement buffers.
  uint16_t total_green_voltage = 0, total_blue_voltage = 0;
};

uint8_t smooth_voltage(uint8_t old_smooth_voltage, uint8_t new_voltage) {
  return ((max(old_smooth_voltage, new_voltage) -
           min(old_smooth_voltage, new_voltage)) > VOLTAGE_UPDATE_HYSTHERESIS)
             ? new_voltage
             : old_smooth_voltage;
}

uint8_t add_offset(uint8_t base, int8_t offset) {
  if (offset == 0) {
    return base;
  } else if (offset > 0) {
    return (0xFFU - base > offset) ? base + offset : 0xFFU;
  } else {
    return (base > -offset) ? base + offset : 0;
  }
}

class TargetVoltageSmoother {
public:
  TargetVoltageSmoother(const SteeringSpoofSettings &steering_spoof_settings)
      : steering_spoof_settings_(steering_spoof_settings) {}

  void set_target_offset(int8_t new_target_offset) {
    target_offset_ = new_target_offset;
    if (target_offset_ == current_offset_) {
      steps_spent_at_current_offset_ =
          min(steps_spent_at_current_offset_,
              steering_spoof_settings_.steps_per_adjustment_level);
    }
  }

  void step() {
    // Prevent overflow.
    if (steps_spent_at_current_offset_ < 0xFFFFU) {
      ++steps_spent_at_current_offset_;
    }
    if (target_offset_ != current_offset_) {
      // We are still in progress of adjusting towards the target offset.
      if (steps_spent_at_current_offset_ >
          steering_spoof_settings_.steps_per_adjustment_level) {
        // Enough steps have been spent at the current offset, can move one unit
        // towards the target.
        current_offset_ += (target_offset_ > current_offset_) ? 1 : -1;
        steps_spent_at_current_offset_ = 0;
      }
    } else if (target_offset_ != 0 &&
               steps_spent_at_current_offset_ >
                   (steering_spoof_settings_.steps_at_target_level +
                    steering_spoof_settings_.steps_per_adjustment_level)) {
      // Enough time has been spent after reaching nonzero target offset, reset
      // the target to 0.
      target_offset_ = 0;
    }
  }

  void update_measurments(const HistoricVoltageData &voltage_data) {
    smoothed_blue_voltage_ = smooth_voltage(
        smoothed_blue_voltage_, voltage_data.get_avg_blue_voltage());
    smoothed_green_voltage_ = smooth_voltage(
        smoothed_green_voltage_, voltage_data.get_avg_green_voltage());
  }

  uint8_t get_smoothed_blue_voltage() const { return smoothed_blue_voltage_; }

  uint8_t get_smoothed_green_voltage() const { return smoothed_green_voltage_; }

  uint8_t get_target_blue_voltage() const {
    return add_offset(smoothed_blue_voltage_, -current_offset_);
  }

  uint8_t get_target_green_voltage() const {
    return add_offset(smoothed_green_voltage_, current_offset_);
  }

  int8_t get_current_offset() const { return current_offset_; }

private:
  // Hystheresis-smoothed historical averages.
  uint8_t smoothed_blue_voltage_ = 0, smoothed_green_voltage_ = 0;
  // Effective torque voltage offset to be applied to the hystheresis-smoothed
  // historical values at current step.
  int8_t current_offset_ = 0;
  // Target voltage offset to move to over time, spending a few steps on every
  // intermediate voltage point.
  int8_t target_offset_ = 0;
  uint16_t steps_spent_at_current_offset_ = 0;

  const SteeringSpoofSettings &steering_spoof_settings_;
};

// Set the BLUE DAC out to the target value.
void set_blue_voltage(uint8_t target_voltage) {
  assert(i2c_start(PCF8591_ID << 1 | I2C_WRITE));
  i2c_write(AIN3_ID | DAC_ENABLE);
  i2c_write(target_voltage);
  i2c_stop();
}

// Set the GREEN DAC out to the target value.
void set_green_voltage(uint8_t target_voltage) {
  Wire.beginTransmission(PCF8591_ID);
  Wire.write(AIN3_ID | DAC_ENABLE);
  Wire.write(target_voltage);
  Wire.endTransmission();
}

SteeringSpoofSettings steering_spoof_settings;
HistoricVoltageData historic_voltage_data;
TargetVoltageSmoother voltage_smoother(steering_spoof_settings);

void setup() {
  steering_spoof_settings.max_steering_magnitude = 5;
  steering_spoof_settings.steps_per_adjustment_level = 20;
  steering_spoof_settings.steps_at_target_level = 400;

  Serial.begin(115200);

  i2c_init();
  Wire.begin();

  assert(i2c_start(PCF8591_ID << 1 | I2C_WRITE));
  i2c_write(AIN3_ID);
  i2c_stop();

  Wire.beginTransmission(PCF8591_ID);
  Wire.write(AIN3_ID);
  Wire.endTransmission();

  // Warm up the voltage buffers.
  for (uint16_t i = 0; i < VOLTAGE_READINGS_BUFFER_SIZE; ++i) {
    historic_voltage_data.take_measurement();
  }
}

int step_idx = 0;

void loop() {
  historic_voltage_data.take_measurement();
  voltage_smoother.update_measurments(historic_voltage_data);

  if (Serial.available()) {
    char command = 0;
    while (Serial.available()) {
      // Skip the commands accumulated in the buffer. Only use the last availabe
      // command to avoid the delayed playback of the old commands and ensure
      // immediate reaction to the recent user commands.
      command = Serial.read();
    }
    if (command == 'l') {
      // Turn left.
      voltage_smoother.set_target_offset(
          steering_spoof_settings.max_steering_magnitude);
    } else if (command == 'r') {
      // Turn right.
      voltage_smoother.set_target_offset(
          -steering_spoof_settings.max_steering_magnitude);
    }
  }

  voltage_smoother.step();

  step_idx = (step_idx + 1) % DEBUG_STEPS_PRINT_EVERY;

  if (step_idx == 0) {
    if (DEBUG_PRINT_RAW_TORQUE_IN_VOLTAGE) {
      Serial.print(historic_voltage_data.get_latest_blue_voltage());
      Serial.print(",");
      Serial.print(historic_voltage_data.get_latest_green_voltage());
      Serial.print(",");
    }
    if (DEBUG_PRINT_AVG_TORQUE_IN_VOLTAGE) {
      Serial.print(historic_voltage_data.get_avg_blue_voltage());
      Serial.print(",");
      Serial.print(historic_voltage_data.get_avg_green_voltage());
      Serial.print(",");
    }
    if (DEBUG_PRINT_SMOOTH_TORQUE_IN_VOLTAGE) {
      Serial.print(voltage_smoother.get_smoothed_blue_voltage());
      Serial.print(",");
      Serial.print(voltage_smoother.get_smoothed_green_voltage());
      Serial.print(",");
    }
    if (DEBUG_PRINT_TORQUE_OUT_VOLTAGE) {
      Serial.print(voltage_smoother.get_target_blue_voltage());
      Serial.print(",");
      Serial.print(voltage_smoother.get_target_green_voltage());
      Serial.print(",");
    }
    Serial.print(voltage_smoother.get_current_offset());
    Serial.print(",");
    Serial.println("");
  }

  set_blue_voltage(voltage_smoother.get_target_blue_voltage());
  set_green_voltage(voltage_smoother.get_target_green_voltage());
}
