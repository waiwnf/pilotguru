#ifndef SPOOF_STEERING_SERIAL_COMMANDS_H_
#define SPOOF_STEERING_SERIAL_COMMANDS_H_

#include <stddef.h>
#include <stdint.h>

namespace pilotguru {
namespace kia {

// Control command to be executed on the Arduino board over the USB serial
// connection. Consists of mandatory type indicating the action to be taken and,
// optionally, the value indicating action magnitude, direction etc.
// Interpretation and acceptable range for the value is different for every
// comand type.
struct KiaControlCommand {
  // Set steering torque offset to value.
  static constexpr char STEER = 's';
  // Reply with the current steering voltage values (measured historical
  // voltages, current spoof offset and target spoof offset).
  static constexpr char ECHO_COMMAND = 'e';
  // Reset the state to no spoofing.
  static constexpr char RESET = 'r';

  // str must point to a null-terminated string.
  // Returns true on success, false on failure.
  // command may be modified (and in an invalid state) if parsing overall fails.
  static bool TryParse(const char *str, KiaControlCommand *command);

  bool ToString(char *str, int str_size) const;

  // Command type.
  char type;
  // Command magnitude, direction etc, depending on the type.
  int16_t value;
};

constexpr char VOLTAGE_REPORT_TAG = 'v';

// Helper class to parse a stream of characters over a serial connection into
// a sequence of KiaControlCommand objects.
class KiaControlCommandProcessor {
public:
  static constexpr char COMMAND_END = '\r';
  enum CommandStatus {
    // Nonzero characters since the end of previous command have been consumed,
    // but COMMAND_END has not been received yet.
    INCOMPLETE,
    // Command has been successfully parsed, and no characters have been
    // consumed yet afterwards. This is the only state in which it is legal to
    // call GetCurrentCommand().
    READY_OK,
    // Consumed a sequence ending in COMMAND_END, but the corresponding string
    // does not represent a valid command.
    PARSE_FAIL,
    // Consumed a sequence ending in COMMAND_END, but the corresponding string
    // could not fit into the buffer, hence could not be parsed.
    COMMAND_OVERFLOW
  };

  CommandStatus GetCommandStatus() const;

  // Copies the most recent parsed command to the argument. Only makes sense to
  // be called in READY_OK state, i.e. right after consuming COMMAND_END of a
  // well-formed command. Returns true if the resulting command is valid to use,
  // false otherwise (i.e. when called in state other than READY_OK).
  bool GetCurrentCommand(KiaControlCommand *command) const;

  // Between a parse attempt of a complete command and consumption of the next
  // character (i.e. in states other than INCOMPLETE), signals to the parser
  // that the result of the previous parse attempt (either the parsed command or
  // the error state) has been processed, and the parser should move on to a
  // state waiting for the next command (i.e. state == INCOMPLETE and zero
  // consumed characters).
  // When called between the parse attempt and arrival of the next character,
  // this resets the state and returns true.
  // When called in the middle of receiving a new command (i.e. in INCOMPLETE
  // state), this is a no-op and returns false.
  bool startNextCommand();

  // Consume the next character from the serial connection; try to parse the
  // resulting string when consuming COMMAND_END. Returns the parser state after
  // consuming the argument and if applicable parsing attempt.
  CommandStatus ConsumeChar(char next_char);

private:
  // Try to parse the contents of command_buffer_.
  CommandStatus ParseCommand();

  static constexpr uint16_t max_command_length = 16;
  uint16_t consumed_chars = 0;
  char command_buffer_[max_command_length];
  KiaControlCommand current_command_;
  CommandStatus command_status_ = INCOMPLETE;
};
}
}

#endif // SPOOF_STEERING_SERIAL_COMMANDS_H_
