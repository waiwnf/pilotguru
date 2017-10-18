#ifndef PILOTGURU_CAR_KIA_CAN_HPP_
#define PILOTGURU_CAR_KIA_CAN_HPP_

// Helpers for communicating over KIA Cee'd C-CAN bus.

#include <cstdint>
#include <memory>
#include <mutex>
#include <thread>
#include <vector>

#include <linux/can.h>

#include <car/timestamped_history.hpp>

namespace pilotguru {
namespace kia {

constexpr uint32_t STEERING_WHEEL_ANGLE_CAN_ID = 0x2B0;
constexpr uint8_t STEERING_WHEEL_ANGLE_FRAME_PAYLOAD_SIZE = 5;

constexpr uint32_t VELOCITY_CAN_ID = 0x4B0;
constexpr uint8_t VELOCITY_FRAME_PAYLOAD_SIZE = 8;

int16_t parse_can_int16(const uint8_t *can_bytes);
// TODO move out to a more generic library.
// TODO templatize?
int16_t integer_average(const std::vector<int16_t> &values);

struct SteeringAngle {
  SteeringAngle();
  SteeringAngle(int16_t angle_deci_degrees_in);

  int16_t angle_deci_degrees;
};

std::unique_ptr<SteeringAngle> ParseSteeringAngle(const can_frame &frame);

struct Velocity {
  Velocity();
  Velocity(int16_t front_left_v_in, int16_t front_right_v_in,
           int16_t rear_left_v_in, int16_t rear_right_v_in);

  int16_t average_wheel_speed() const;

  int16_t front_left_v, front_right_v, rear_left_v, rear_right_v;
};
std::unique_ptr<Velocity> ParseVelocity(const can_frame &frame);

class CarMotionData {
public:
  CarMotionData(size_t history_length);

  void update(const can_frame &frame, const timeval &timestamp);

  const TimestampedHistory<SteeringAngle> &steering_angles() const;
  const TimestampedHistory<Velocity> &velocities() const;

private:
  TimestampedHistory<SteeringAngle> steering_angles_;
  TimestampedHistory<Velocity> velocities_;

  bool update_thread_should_run_ = false;
  std::unique_ptr<std::thread> update_thread_;
  std::mutex update_thread_status_mutex_;
};

class CarMotionDataUpdater {
public:
  CarMotionDataUpdater(CarMotionData *car_motion_data,
                       const std::string &can_interface_name,
                       const std::vector<canid_t> &accepted_ids,
                       const timeval &timeout);
  void start();
  void stop();

private:
  void updateLoop();

  CarMotionData *car_motion_data_;
  int can_socket_;

  bool should_run_ = false;
  std::unique_ptr<std::thread> update_thread_;
  std::mutex thread_status_mutex_;
};
}
}

#endif
