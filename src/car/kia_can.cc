#include <car/can.hpp>
#include <car/kia_can.hpp>

#include <arpa/inet.h>
#include <glog/logging.h>
#include <sys/ioctl.h>

namespace pilotguru {
namespace kia {

int16_t parse_can_int16(const uint8_t *can_bytes) {
  CHECK_NOTNULL(can_bytes);

  // CAN frames use little endian format. While on x86 we can simply memcpy()
  // the two bytes to an int16, on big-endian architectures that would produce
  // wrong results. So first convert the data to big endian (network format),
  // then use ntohs() to convert from network to host format.
  uint8_t big_endian_bytes[2];
  big_endian_bytes[0] = can_bytes[1];
  big_endian_bytes[1] = can_bytes[0];
  uint16_t value_unsigned_big_endian = 0;
  memcpy(&value_unsigned_big_endian, big_endian_bytes, 2);
  const uint16_t result_unsigned = ntohs(value_unsigned_big_endian);
  return *reinterpret_cast<const int16_t *>(&result_unsigned);
}

int16_t integer_average(const std::vector<int16_t> &values) {
  int16_t result = 0, remainder = 0;
  for (const int16_t v : values) {
    const int16_t v_fraction = v / values.size();
    result += v_fraction;
    remainder += (v - v_fraction);
  }
  result += remainder / values.size();
  return result;
}

SteeringAngle::SteeringAngle() : angle_deci_degrees(0) {}

SteeringAngle::SteeringAngle(int16_t angle_deci_degrees_in)
    : angle_deci_degrees(angle_deci_degrees_in) {}

std::unique_ptr<SteeringAngle> ParseSteeringAngle(const can_frame &frame) {
  if (frame.can_dlc != STEERING_WHEEL_ANGLE_FRAME_PAYLOAD_SIZE) {
    return nullptr;
  }

  return std::unique_ptr<SteeringAngle>(
      new SteeringAngle(parse_can_int16(frame.data)));
}

Velocity::Velocity()
    : front_left_v(0), front_right_v(0), rear_left_v(0), rear_right_v(0) {}

Velocity::Velocity(int16_t front_left_v_in, int16_t front_right_v_in,
                   int16_t rear_left_v_in, int16_t rear_right_v_in)
    : front_left_v(front_left_v_in), front_right_v(front_right_v_in),
      rear_left_v(rear_left_v_in), rear_right_v(rear_right_v_in) {}

int16_t Velocity::average_wheel_speed() const {
  return integer_average(
      {front_left_v, front_right_v, rear_left_v, rear_right_v});
}

std::unique_ptr<Velocity> ParseVelocity(const can_frame &frame) {
  if (frame.can_dlc != VELOCITY_FRAME_PAYLOAD_SIZE) {
    return nullptr;
  }

  return std::unique_ptr<Velocity>(new Velocity(
      parse_can_int16(frame.data), parse_can_int16(frame.data + 2),
      parse_can_int16(frame.data + 4), parse_can_int16(frame.data + 6)));
}

CarMotionData::CarMotionData(size_t history_length)
    : steering_angles_(history_length), velocities_(history_length) {}

void CarMotionData::update(const can_frame &frame, const timeval &timestamp) {
  switch (frame.can_id) {
  case STEERING_WHEEL_ANGLE_CAN_ID: {
    std::unique_ptr<SteeringAngle> angle = ParseSteeringAngle(frame);
    if (angle != nullptr) {
      steering_angles_.update(*angle, timestamp);
    }
    break;
  }
  case VELOCITY_CAN_ID: {
    std::unique_ptr<Velocity> velocity = ParseVelocity(frame);
    if (velocity != nullptr) {
      velocities_.update(*velocity, timestamp);
    }
    break;
  }
  default:
    return;
  }
}

const TimestampedHistory<SteeringAngle> &
CarMotionData::steering_angles() const {
  return steering_angles_;
}
const TimestampedHistory<Velocity> &CarMotionData::velocities() const {
  return velocities_;
}

CarMotionDataUpdater::CarMotionDataUpdater(
    CarMotionData *car_motion_data, const std::string &can_interface_name,
    const std::vector<canid_t> &accepted_ids, const timeval &timeout)
    : car_motion_data_(car_motion_data) {
  CHECK_NOTNULL(car_motion_data_);

  can_socket_ = connect_new_can_soket(can_interface_name, accepted_ids);
  CHECK_GE(can_socket_, 0);

  const int set_timeout_result =
      setsockopt(can_socket_, SOL_SOCKET, SO_RCVTIMEO, (const char *)&timeout,
                 sizeof(timeval));
  CHECK_EQ(set_timeout_result, 0);
}

void CarMotionDataUpdater::start() {
  std::unique_lock<std::mutex> lock;
  if (update_thread_ == nullptr) {
    should_run_ = true;
    update_thread_.reset(
        new std::thread(&CarMotionDataUpdater::updateLoop, this));
  }
}

void CarMotionDataUpdater::stop() {
  std::unique_lock<std::mutex> lock;
  if (update_thread_ != nullptr) {
    should_run_ = false;
    update_thread_->join();
    update_thread_.reset(nullptr);
  }
}

void CarMotionDataUpdater::updateLoop() {
  can_frame frame;
  timeval timestamp;

  while (should_run_) {
    const int nbytes = read(can_socket_, &frame, sizeof(struct can_frame));
    if (nbytes == sizeof(can_frame)) {
      ioctl(can_socket_, SIOCGSTAMP, &timestamp);
      car_motion_data_->update(frame, timestamp);
    }
  }
}
}
}
