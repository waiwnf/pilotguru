#include <car/arduino_comm.hpp>

#include <fcntl.h>
#include <termios.h>

#include <glog/logging.h>

namespace pilotguru {

OpenedTty::OpenedTty(const std::string &tty_name) {
  fd_ = open(tty_name.c_str(), O_RDWR | O_NOCTTY);
  CHECK_GE(fd_, 0);

  // Configure baud rate.
  termios tios;
  memset(&tios, 0, sizeof(termios));
  const int tcgetattr_result = tcgetattr(fd_, &tios);
  CHECK_GE(tcgetattr_result, 0);
  old_ispeed_ = cfgetispeed(&tios);
  old_ospeed_ = cfgetospeed(&tios);

  /* Reset UART settings */
  cfmakeraw(&tios);
  tios.c_iflag &= ~IXOFF;
  tios.c_cflag &= ~CRTSCTS;

  /* Baud Rate */
  cfsetispeed(&tios, B115200);
  cfsetospeed(&tios, B115200);

  /* apply changes */
  const int tcsetattr_result = tcsetattr(fd_, TCSADRAIN, &tios);
  CHECK_GE(tcsetattr_result, 0);
}

OpenedTty::~OpenedTty() {
  termios tios;
  memset(&tios, 0, sizeof(termios));
  const int tcgetattr_result = tcgetattr(fd_, &tios);
  CHECK_GE(tcgetattr_result, 0);

  /* Reset old rates */
  cfsetispeed(&tios, old_ispeed_);
  cfsetospeed(&tios, old_ospeed_);

  /* apply changes */
  const int tcsetattr_restore_result = tcsetattr(fd_, TCSADRAIN, &tios);
  CHECK_GE(tcsetattr_restore_result, 0);
}

int OpenedTty::fd() const { return fd_; }

int OpenedTty::wait_read(timeval *timeout) {
  fd_set fd_set_singleton;
  FD_ZERO(&fd_set_singleton);
  FD_SET(fd_, &fd_set_singleton);

  return select(fd_ + 1, &fd_set_singleton, nullptr, nullptr, timeout);
}

ArduinoCommandChannel::ArduinoCommandChannel(const std::string &tty_name)
    : arduino_tty_(tty_name) {
  // TODO refactor.
  constexpr char RESET_COMMAND[] = "r";
  // Send first reset command to stop Arduino from writing new data to the
  // serial conection. Note that the response to this reset may not arrive if
  // the serial buffer is already full.
  const ssize_t reset_write_result = write(arduino_tty_.fd(), RESET_COMMAND, 1);
  CHECK_EQ(reset_write_result, 1);
  const ssize_t eol_write_result = write(arduino_tty_.fd(), &COMMAND_EOL_OK, 1);
  CHECK_EQ(eol_write_result, 1);
  // Read everything that shows up in the serial buffer until it is empty.
  constexpr timeval ONE_SECOND = {1 /* seconds */, 0 /* micros */};
  timeval timeout = ONE_SECOND;
  int wait_result = arduino_tty_.wait_read(&timeout);
  char read_result;
  while (wait_result > 0) {
    const ssize_t read_bytes_num = read(arduino_tty_.fd(), &read_result, 1);
    CHECK_EQ(read_bytes_num, 1);
    timeout = ONE_SECOND;
    wait_result = arduino_tty_.wait_read(&timeout);
  }
  // Once the buffer is empty, the second reset command should return an OK
  // result.
  const char reset_result = SendCommand(RESET_COMMAND);
  CHECK_EQ(reset_result, COMMAND_EOL_OK);
  // Make sure that what we got from the serial connection was actually in
  // response to the second reset command and there is nothing in the buffer
  // afterwards.
  timeout = ONE_SECOND;
  wait_result = arduino_tty_.wait_read(&timeout);
  CHECK_EQ(wait_result, 0);
}

char ArduinoCommandChannel::SendCommand(const std::string &command) {
  std::unique_lock<std::mutex> lock(tty_mutex_, std::try_to_lock);
  if (!lock.owns_lock()) {
    // TODO separate signal for lock error.
    return COMMAND_EOL_ERR;
  }

  const ssize_t command_length = command.length();
  const ssize_t write_result =
      write(arduino_tty_.fd(), command.c_str(), command_length);
  const ssize_t eol_write_result = write(arduino_tty_.fd(), &COMMAND_EOL_OK, 1);
  if (write_result != command_length || eol_write_result != 1) {
    return COMMAND_EOL_ERR;
  }

  // Wait for reply
  const int wait_result = arduino_tty_.wait_read(nullptr);
  if (wait_result <= 0) {
    return COMMAND_EOL_ERR;
  }
  // Read reply.
  char result;
  const ssize_t read_bytes_num = read(arduino_tty_.fd(), &result, 1);
  if (read_bytes_num != 1) {
    return COMMAND_EOL_ERR;
  }
  return result;
}
}
