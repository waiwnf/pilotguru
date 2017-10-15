#include <spoof-steering-serial-commands.h>

#include <errno.h>
#include <stdio.h>
#include <stdlib.h>
#include <string.h>

namespace pilotguru {
namespace kia {
namespace {
// Parses the str argument assuming that the string contains only a non-empty
// decimal value (possibly preceded by spaces) within the range of 16-bit signed
// integers. If the assumption holds, stores the parsed value at result_value
// and returns true. In all other cases returns false.
bool ParseInt16Value(const char *str, int16_t *result_value) {
  if (str == nullptr || result_value == nullptr) {
    // Invalid arguments.
    return false;
  }
  if (str[0] == 0) {
    // Zero-length string when nontrivial integer value is expected.
    return false;
  }
  char *endptr = nullptr;
  const long int long_result = strtol(str, &endptr, 10 /* base */);
  if (endptr == nullptr || *endptr != 0) {
    // Input string is not a valid integer (no conversion was done) or there is
    // unexpected nonempty suffix after the integer.
    return false;
  }
  if (errno == ERANGE) {
    // strtol() signaled out of range value.
    return false;
  }
  if (long_result > INT16_MAX || long_result < INT16_MIN) {
    // The value does not fit into the expected 16 bit integer.
    return false;
  }
  *result_value = long_result;
  return true;
}
} // namespace

bool KiaControlCommand::TryParse(const char *str, KiaControlCommand *command) {
  if (str == nullptr || command == nullptr) {
    return false;
  }
  if (strlen(str) == 0) {
    return false;
  }
  command->type = str[0];
  switch (command->type) {
  case RESET: {
    // Reset has no associated integer value, so there should be no more
    // characters before the end of the line.
    return str[1] == 0;
  }
  case STEER: {
    // Steering magnitude is not bounded apriori, so as long as there is a valid
    // integer, we are fine.
    return ParseInt16Value(str + 1, &(command->value));
  }
  case ECHO_COMMAND: {
    const bool int_parse_status = ParseInt16Value(str + 1, &(command->value));
    // Echo command must have value either 0 (off) or 1 (on). Other values are
    // not valid.
    return (int_parse_status && (command->value == 1 || command->value == 0));
  }
  default: { return false; }
  }
}

bool KiaControlCommand::ToString(char *str, int str_size) const {
  constexpr char string_format[] = "%c%d";
  if (str == nullptr || str_size < 2) {
    return false;
  }
  switch (type) {
  case STEER: {
    const int written_bytes =
        snprintf(str, str_size, string_format, type, value);
    // Strict equality since snprintf() does not count the terminating null.
    return written_bytes < str_size;
  }
  case ECHO_COMMAND: {
    if (value != 0 && value != 1) {
      return false;
    } else {
      const int written_bytes =
          snprintf(str, str_size, string_format, type, value);
      // Strict equality since snprintf() does not count the terminating null.
      return written_bytes < str_size;
    }
  }
  case RESET: {
    const int written_bytes = snprintf(str, str_size, "%c", type);
    // Strict equality since snprintf() does not count the terminating null.
    return written_bytes < str_size;
  }
  default: { return false; }
  }
}

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

bool KiaControlCommandProcessor::startNextCommand() {
  switch (command_status_) {
  case INCOMPLETE:
    return false;
  case READY_OK:
  case PARSE_FAIL:
  case COMMAND_OVERFLOW:
  default:
    consumed_chars = 0;
    command_status_ = INCOMPLETE;
    return true;
  }
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

KiaControlCommandProcessor::CommandStatus
KiaControlCommandProcessor::ParseCommand() {
  if (consumed_chars > max_command_length) {
    return COMMAND_OVERFLOW;
  }
  if (consumed_chars == 0) {
    return PARSE_FAIL;
  }
  // Do not use strlen() here to make sure we stay witin the buffer limits.
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
  return KiaControlCommand::TryParse(command_buffer_, &current_command_)
             ? READY_OK
             : PARSE_FAIL;
}

} // namespace kia
} // namespace pilotguru
