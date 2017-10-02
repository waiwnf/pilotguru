// Proof of concept steering by wire module for KIA Cee'd JD 2015 model year,
// steering column assembly part No 56310A2001. Possibly will work on other
// KIA/Hyundai vehicles with similar electric power steering setups (though not
// tested).
// Tested on Arduino UNO with two PCF8591-based YL-40 AD/DA converters.

// Requires kia-spoof-steering library from sketches/libs.

#define DEBUG_PRINT_RAW_TORQUE_IN_VOLTAGE false
#define DEBUG_PRINT_AVG_TORQUE_IN_VOLTAGE false
#define DEBUG_PRINT_SMOOTH_TORQUE_IN_VOLTAGE false
#define DEBUG_PRINT_TORQUE_OUT_VOLTAGE true
#define DEBUG_STEPS_PRINT_EVERY 100

#include <assert.h>
#include <kia-spoof-steering-comm-pcf8591.h>
#include <kia-spoof-steering.h>

SteeringSpoofSettings steering_spoof_settings;
HistoricVoltageData<4> historic_voltage_data;
TargetVoltageSmoother voltage_smoother(steering_spoof_settings);

InstantVoltageData voltage_measurement;

void setup() {
  steering_spoof_settings.max_steering_magnitude = 5;
  steering_spoof_settings.steps_per_adjustment_level = 20;
  steering_spoof_settings.steps_at_target_level = 400;
  steering_spoof_settings.voltage_update_hystheresis = 1;

  Serial.begin(115200);
  init_spoof_steering_pcf8591_comms();

  // Warm up the voltage buffers.
  for (size_t i = 0; i < historic_voltage_data.get_buffer_size(); ++i) {
    voltage_measurement.green_voltage = read_green_voltage();
    voltage_measurement.blue_voltage = read_blue_voltage();
    historic_voltage_data.take_measurement(voltage_measurement);
  }
}

int step_idx = 0;

void loop() {
  voltage_measurement.green_voltage = read_green_voltage();
  voltage_measurement.blue_voltage = read_blue_voltage();
  historic_voltage_data.take_measurement(voltage_measurement);
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
