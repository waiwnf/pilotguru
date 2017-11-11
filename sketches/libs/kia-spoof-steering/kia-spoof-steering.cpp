#include <kia-spoof-steering.h>

#include <Arduino.h>

bool ExecuteKiaControlCommand(
    const pilotguru::kia::KiaControlCommand &control_command,
    TargetVoltageSmoother *voltage_smoother, bool *is_echo_on) {
  if (voltage_smoother == nullptr || is_echo_on == nullptr) {
    return false;
  }

  switch (control_command.type) {
  case pilotguru::kia::KiaControlCommand::STEER: {
    voltage_smoother->set_target_offset_dac_units(control_command.value);
    Serial.write(LW232_RET_ASCII_OK);
    return true;
  }
  case pilotguru::kia::KiaControlCommand::ECHO_COMMAND: {
    *is_echo_on = (control_command.value != 0);
    Serial.write(LW232_RET_ASCII_OK);
    return true;
  }
  case pilotguru::kia::KiaControlCommand::RESET: {
    voltage_smoother->set_target_offset_dac_units(0);
    *is_echo_on = false;
    Serial.write(LW232_RET_ASCII_OK);
    return true;
  }
  default:
    Serial.write(LW232_RET_ASCII_ERROR);
    return false;
  }
}

void HandleCommandProcessorState(
    pilotguru::kia::KiaControlCommandProcessor *command_processor,
    pilotguru::kia::KiaControlCommand *control_command,
    TargetVoltageSmoother *voltage_smoother, bool *is_echo_on) {
  const pilotguru::kia::KiaControlCommandProcessor::CommandStatus
      command_status = command_processor->GetCommandStatus();
  switch (command_status) {
  case pilotguru::kia::KiaControlCommandProcessor::INCOMPLETE: {
    break;
  }
  case pilotguru::kia::KiaControlCommandProcessor::PARSE_FAIL:
  case pilotguru::kia::KiaControlCommandProcessor::COMMAND_OVERFLOW: {
    Serial.write(LW232_RET_ASCII_ERROR);
    command_processor->startNextCommand();
    break;
  }
  case pilotguru::kia::KiaControlCommandProcessor::READY_OK: {
    command_processor->GetCurrentCommand(control_command);
    ExecuteKiaControlCommand(*control_command, voltage_smoother, is_echo_on);
    command_processor->startNextCommand();
    break;
  }
  default:
    Serial.write(LW232_RET_ASCII_ERROR);
  }
}

void ProcessAvailableSerialBuffer(
    pilotguru::kia::KiaControlCommandProcessor *command_processor) {
  while (Serial.available()) {
    const char next_char = Serial.read();
    const pilotguru::kia::KiaControlCommandProcessor::CommandStatus
        command_status = command_processor->ConsumeChar(next_char);
    if (command_status !=
        pilotguru::kia::KiaControlCommandProcessor::INCOMPLETE) {
      break;
    }
  }
}

void SerialWriteCurrentSpoofVoltages(
    const TargetVoltageSmoother &voltage_smoother, char *voltage_state_string,
    size_t voltage_state_buffer_size) {
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
