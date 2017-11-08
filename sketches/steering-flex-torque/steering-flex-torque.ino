#include <kia-spoof-steering-comm-pcf8591.h>
#include <kia-spoof-steering.h>
#include <spoof-steering-serial-commands.h>

#define STEPS_ECHO_EVERY 100

SteeringSpoofSettings steering_spoof_settings;
HistoricVoltageData<4> historic_voltage_data;
InstantVoltageData voltage_measurement, historic_avg_voltage;
TargetVoltageSmoother voltage_smoother(steering_spoof_settings);
pilotguru::kia::KiaControlCommandProcessor command_processor;
pilotguru::kia::KiaControlCommand control_command;

constexpr size_t int16_max_chars = 6; // 5 digits + sign;
// Adding separating comma for every number and the terminating null character.
constexpr size_t voltage_state_buffer_size = (int16_max_chars + 1) * 4 + 1;
// Adding space for an extra first 'v' tag character to the buffer size.
char voltage_state_string[voltage_state_buffer_size + 1];

bool is_echo_on = false;
int step_idx = 0;

void setup() {
  steering_spoof_settings.max_steering_magnitude = 5;
  steering_spoof_settings.steps_per_adjustment_level = 2;
  steering_spoof_settings.steps_at_target_level = 200;
  steering_spoof_settings.voltage_update_hystheresis = 1;

  bool is_echo_on = false;
  voltage_state_string[0] = pilotguru::kia::VOLTAGE_REPORT_TAG;

  Serial.begin(115200);
  init_spoof_steering_pcf8591_comms();

  // Warm up the voltage buffers.
  for (size_t i = 0; i < historic_voltage_data.get_buffer_size(); ++i) {
    voltage_measurement.green_voltage = read_green_voltage();
    voltage_measurement.blue_voltage = read_blue_voltage();
    historic_voltage_data.take_measurement(voltage_measurement);
  }
}

void loop() {
  voltage_measurement.green_voltage = read_green_voltage();
  voltage_measurement.blue_voltage = read_blue_voltage();
  historic_voltage_data.take_measurement(voltage_measurement);
  historic_avg_voltage = historic_voltage_data.get_avg_voltage();
  voltage_smoother.update_measurments(historic_avg_voltage);

  HandleCommandProcessorState(&command_processor, &control_command,
                              &voltage_smoother, &is_echo_on);

  voltage_smoother.step();

  step_idx = (step_idx + 1) % STEPS_ECHO_EVERY;
  if (step_idx == 0 && is_echo_on) {
    SerialWriteCurrentSpoofVoltages(voltage_smoother, voltage_state_string,
                                    voltage_state_buffer_size);
  }

  set_blue_voltage(voltage_smoother.get_target_blue_voltage());
  set_green_voltage(voltage_smoother.get_target_green_voltage());
}

void serialEvent() { ProcessAvailableSerialBuffer(&command_processor); }
