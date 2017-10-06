#include <spoof-steering-serial-commands.h>

#include <errno.h>
#include <stdlib.h>
#include <string.h>

namespace pilotguru {
namespace kia {

KiaControlCommandProcessor::CommandStatus
KiaControlCommandProcessor::GetCommandStatus() const {
  return command_status_;
}

bool KiaControlCommandProcessor::GetCurrentCommand(
    KiaControlCommand *command) const {
  if (command == nullptr) {
    return false;
  }
  if (command_status_ != READY_OK) {
    return false;
  }
  memcpy(command, &current_command_, sizeof(KiaControlCommand));
  return true;
}

KiaControlCommandProcessor::CommandStatus
KiaControlCommandProcessor::ConsumeChar(char next_char) {
  if (consumed_chars < max_command_length) {
    // Space still avaliable in the buffer.
    command_buffer_[consumed_chars] =
        (next_char != COMMAND_END) ? next_char : 0;
    ++consumed_chars;
    if (next_char == COMMAND_END) {
      command_status_ = ParseCommand();
      consumed_chars = 0;
    } else {
      command_status_ = INCOMPLETE;
    }
  } else {
    if (next_char == COMMAND_END) {
      command_status_ = COMMAND_OVERFLOW;
      consumed_chars = 0;
    }
  }
  return command_status_;
}

namespace {
// Parses the str argument assuming that the string contains only a non-empty
// decimal value (possibly preceded by spaces) within the range of 16-bit signed
// integers. If the assumption holds, the parsed values is stored at
// result_value address and READY_OK is returned. In all other cases returns
// PARSE_FAIL.
KiaControlCommandProcessor::CommandStatus ParseIntValue(const char *str,
                                                        int16_t *result_value) {
  if (str == nullptr || result_value == nullptr) {
    // Invalid arguments.
    return KiaControlCommandProcessor::PARSE_FAIL;
  }
  if (str[0] == 0) {
    // Zero-length string when nontrivial integer value is expected.
    return KiaControlCommandProcessor::PARSE_FAIL;
  }
  char *endptr = nullptr;
  const long int long_result = strtol(str, &endptr, 10);
  if (endptr == nullptr || *endptr != 0) {
    // Input string is not a valid integer (no conversion was done) or there is
    // unexpected nonempty suffix after the integer.
    return KiaControlCommandProcessor::PARSE_FAIL;
  }
  if (errno == ERANGE) {
    // strtol() signaled out of range value.
    return KiaControlCommandProcessor::PARSE_FAIL;
  }
  if (long_result > INT16_MAX || long_result < INT16_MIN) {
    // The value does not fit into the expected 16 bit integer.
    return KiaControlCommandProcessor::PARSE_FAIL;
  }
  *result_value = long_result;
  return KiaControlCommandProcessor::READY_OK;
}
}

KiaControlCommandProcessor::CommandStatus
KiaControlCommandProcessor::ParseCommand() {
  if (consumed_chars > max_command_length) {
    return COMMAND_OVERFLOW;
  }
  if (consumed_chars == 0) {
    return PARSE_FAIL;
  }

  bool string_within_bounds = false;
  for (uint16_t i = 0; i < max_command_length; ++i) {
    if (command_buffer_[i] == 0) {
      string_within_bounds = true;
      break;
    }
  }
  if (!string_within_bounds) {
    return PARSE_FAIL;
  }
  current_command_.type = command_buffer_[0];
  switch (current_command_.type) {
  case KiaControlCommand::RESET: {
    // Reset has no associated integer value, so there should be no more
    // characters before the end of the line.
    return (command_buffer_[1] == 0) ? READY_OK : PARSE_FAIL;
  }
  case KiaControlCommand::STEER: {
    // Steering magnitude is not bounded apriori, so as long as there is a valid
    // integer, we are fine.
    return ParseIntValue(command_buffer_ + 1, &(current_command_.value));
  }
  case KiaControlCommand::ECHO_COMMAND: {
    const CommandStatus parse_status =
        ParseIntValue(command_buffer_ + 1, &(current_command_.value));
    // Echo command must have value either 0 (off) or 1 (on). Other values are
    // not valid.
    if (parse_status == READY_OK &&
        (current_command_.value == 1 || current_command_.value == 0)) {
      return READY_OK;
    } else {
      return PARSE_FAIL;
    }
  }
  default: { return PARSE_FAIL; }
  }
}

} // namespace kia
} // namespace pilotguru
