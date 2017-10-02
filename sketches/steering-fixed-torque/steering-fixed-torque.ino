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
#include <kia-spoof-steering.h>

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
HistoricVoltageData<4> historic_voltage_data;
TargetVoltageSmoother voltage_smoother(steering_spoof_settings);

void setup() {
  steering_spoof_settings.max_steering_magnitude = 5;
  steering_spoof_settings.steps_per_adjustment_level = 20;
  steering_spoof_settings.steps_at_target_level = 400;
  steering_spoof_settings.voltage_update_hystheresis = 1;

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
  for (size_t i = 0; i < historic_voltage_data.get_buffer_size(); ++i) {
    historic_voltage_data.take_measurement();
  }
}

int step_idx = 0;

void loop() {
  historic_voltage_data.take_measurement();
  voltage_smoother.update_measurments(historic_voltage_data.get_avg_voltage());

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
