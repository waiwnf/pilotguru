#ifndef PILOTGURU_CAR_ARDUINO_COMM_HPP_
#define PILOTGURU_CAR_ARDUINO_COMM_HPP_

#include <mutex>
#include <string>

#include <termios.h>

// TODO refactor. The command struct should have a ToString().
constexpr char COMMAND_EOL_OK = '\r';
constexpr char COMMAND_EOL_ERR = 0x07;

namespace pilotguru {

// TTY wrapper hardcoded to 115200 baud.
class OpenedTty {
public:
  OpenedTty(const std::string& tty_name);
  ~OpenedTty();

  int fd() const;
  int wait_read(timeval *timeout);

private:
  // Opened TTY file descriptor.
  int fd_;

  // Speed settings before opening the TTY, to be restored on closing.
  speed_t old_ispeed_, old_ospeed_;
};

class ArduinoCommandChannel {
public:
  ArduinoCommandChannel(const std::string& tty_name);
  char SendCommand(const std::string& command);

private:
  OpenedTty arduino_tty_;
  std::mutex tty_mutex_;
};

}

#endif
