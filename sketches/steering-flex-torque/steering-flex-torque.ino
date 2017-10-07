#include <kia-spoof-steering-comm-pcf8591.h>
#include <kia-spoof-steering.h>
#include <spoof-steering-serial-commands.h>

// TODO move these out to a library to be accessible by the host code.
#define LW232_CR '\r'
#define LW232_RET_ASCII_OK 0x0D
#define LW232_RET_ASCII_ERROR 0x07

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
  steering_spoof_settings.steps_per_adjustment_level = 20;
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

char ExecuteKiaControlCommand() {
  switch (control_command.type) {
  case pilotguru::kia::KiaControlCommand::STEER: {
    voltage_smoother.set_target_offset(control_command.value);
    Serial.write(LW232_RET_ASCII_OK);
    break;
  }
  case pilotguru::kia::KiaControlCommand::ECHO_COMMAND: {
    is_echo_on = (control_command.value != 0);
    Serial.write(LW232_RET_ASCII_OK);
    break;
  }
  case pilotguru::kia::KiaControlCommand::RESET: {
    voltage_smoother.set_target_offset(0);
    is_echo_on = false;
    Serial.write(LW232_RET_ASCII_OK);
    break;
  }
  default:
    Serial.write(LW232_RET_ASCII_ERROR);
  }
}

void loop() {
  voltage_measurement.green_voltage = read_green_voltage();
  voltage_measurement.blue_voltage = read_blue_voltage();
  // historic_voltage_data.take_measurement(voltage_measurement);
  historic_voltage_data.take_measurement({128, 128});
  historic_avg_voltage = historic_voltage_data.get_avg_voltage();
  voltage_smoother.update_measurments(historic_avg_voltage);

  const pilotguru::kia::KiaControlCommandProcessor::CommandStatus
      command_status = command_processor.GetCommandStatus();
  switch (command_status) {
  case pilotguru::kia::KiaControlCommandProcessor::INCOMPLETE: {
    break;
  }
  case pilotguru::kia::KiaControlCommandProcessor::PARSE_FAIL:
  case pilotguru::kia::KiaControlCommandProcessor::COMMAND_OVERFLOW: {
    Serial.write(LW232_RET_ASCII_ERROR);
    command_processor.startNextCommand();
    break;
  }
  case pilotguru::kia::KiaControlCommandProcessor::READY_OK: {
    command_processor.GetCurrentCommand(&control_command);
    ExecuteKiaControlCommand();
    command_processor.startNextCommand();
    break;
  }
  default:
    Serial.write(LW232_RET_ASCII_ERROR);
  }

  voltage_smoother.step();

  step_idx = (step_idx + 1) % STEPS_ECHO_EVERY;

  if (step_idx == 0 && is_echo_on) {
    const int effective_voltage_string_length =
        voltage_smoother.get_voltage_state().make_string(
            voltage_state_string + 1, voltage_state_buffer_size);
    if (effective_voltage_string_length > 0 &&
        effective_voltage_string_length < voltage_state_buffer_size) {
      voltage_state_string[effective_voltage_string_length + 1] =
          LW232_RET_ASCII_OK;
      // Extra characters for the front info type tag, and for the trailing
      // LW232_RET_ASCII_OK end-of-line.
      Serial.write(voltage_state_string, effective_voltage_string_length + 2);
    }
  }

  // Not using the requested offset for now, simply passing through the measured
  // sensors voltage.
  set_blue_voltage(historic_avg_voltage.blue_voltage);
  set_green_voltage(historic_avg_voltage.green_voltage);
}

void serialEvent() {
  while (Serial.available()) {
    const char next_char = Serial.read();
    const pilotguru::kia::KiaControlCommandProcessor::CommandStatus
        command_status = command_processor.ConsumeChar(next_char);
    if (command_status !=
        pilotguru::kia::KiaControlCommandProcessor::INCOMPLETE) {
      break;
    }
  }
}
