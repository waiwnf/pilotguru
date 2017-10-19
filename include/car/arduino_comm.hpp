#ifndef PILOTGURU_CAR_ARDUINO_COMM_HPP_
#define PILOTGURU_CAR_ARDUINO_COMM_HPP_

#include <mutex>
#include <string>

#include <termios.h>

#include <car/timestamped_history.hpp>
#include <spoof-steering-serial-commands.h>

// TODO refactor. The command struct should have a ToString().
constexpr char COMMAND_EOL_OK = '\r';
constexpr char COMMAND_EOL_ERR = 0x07;

namespace pilotguru {

// TTY wrapper hardcoded to 115200 baud.
class OpenedTty {
public:
  OpenedTty(const std::string &tty_name);
  ~OpenedTty();

  int fd() const;
  int wait_read(timeval *timeout) const;

private:
  // Opened TTY file descriptor.
  int fd_;

  // Speed settings before opening the TTY, to be restored on closing.
  speed_t old_ispeed_, old_ospeed_;
};

class ArduinoCommandChannel {
public:
  ArduinoCommandChannel(const std::string &tty_name, size_t history_length = 5);
  char SendCommand(const kia::KiaControlCommand &command);
  const TimestampedHistory<kia::KiaControlCommand> &CommandsHistory() const;

private:
  OpenedTty arduino_tty_;
  std::unique_ptr<TimestampedHistory<kia::KiaControlCommand>> commands_history_;

  // TODO does it make sense to unify buffer length with
  // KiaControlCommandProcessor?
  static constexpr uint16_t max_command_length = 16;
  char command_buffer_[max_command_length];

  // Guards both the tty and command buffer.
  std::mutex mutex_;
};
}

#endif
