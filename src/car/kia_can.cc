#include <car/kia_can.hpp>

#include <arpa/inet.h>
#include <glog/logging.h>

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

std::vector<Timestamped<SteeringAngle>>
CarMotionData::get_steering_angles_history() const {
  return steering_angles_.get_history();
}

std::vector<Timestamped<Velocity>>
CarMotionData::get_velocities_history() const {
  return velocities_.get_history();
}
}
}