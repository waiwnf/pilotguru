#ifndef PILOTGURU_CAR_KIA_CAN_HPP_
#define PILOTGURU_CAR_KIA_CAN_HPP_

// Helpers for communicating over KIA Cee'd C-CAN bus.

#include <algorithm>
#include <chrono>
#include <condition_variable>
#include <cstdint>
#include <memory>
#include <mutex>
#include <thread>
#include <vector>

#include <linux/can.h>

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

template <typename T> class Timestamped {
public:
  Timestamped() {}
  Timestamped(const T &data, const timeval &timestamp)
      : data_(data), timestamp_(timestamp) {}

  const T &data() const { return data_; }
  const timeval &timestamp() const { return timestamp_; }

private:
  T data_;
  timeval timestamp_;
};

// TODO Proper warm-up handling (when fewer than history_length updates have
// been made).
template <typename T> class TimestampedHistory {
public:
  TimestampedHistory(size_t history_length)
      : history_length_(history_length), values_(history_length),
        latest_idx_(history_length - 1) {}

  void update(const T &t, const timeval &timestamp) {
    // TODO timestamp checks.
    std::unique_lock<std::mutex> lock(data_mutex_);
    latest_idx_ = (latest_idx_ + 1) % history_length_;
    values_.at(latest_idx_) = Timestamped<T>(t, timestamp);
    data_update_condition_.notify_all();
  }

  // Returns a copy of the history properly sorted from the oldest to the most
  // recent element.
  std::vector<Timestamped<T>> get_history() const {
    std::vector<Timestamped<T>> result(history_length_);
    std::unique_lock<std::mutex> lock(data_mutex_);
    // Copy tail.
    const auto values_oldest = values_.begin() + latest_idx_ + 1;
    std::copy(values_oldest, values_.end(), result.begin());
    // Copy head.
    const size_t tail_size = history_length_ - latest_idx_;
    std::copy(values_.begin(), values_oldest, result.begin() + tail_size);
    return result;
  }

  bool wait_get_next(const timeval &prev_timestamp,
                     const std::chrono::microseconds *timeout,
                     Timestamped<T> *result) const {
    if (result == nullptr) {
      return false;
    }
    std::unique_lock<std::mutex> lock(data_mutex_);
    auto predicate = [this, &prev_timestamp]() {
      return !(prev_timestamp.tv_sec ==
                   this->values_.at(this->latest_idx_).timestamp().tv_sec &&
               prev_timestamp.tv_usec ==
                   this->values_.at(this->latest_idx_).timestamp().tv_usec);
    };
    if (timeout != nullptr) {
      const bool wait_result =
          data_update_condition_.wait_for(lock, *timeout, predicate);
      if (!wait_result) {
        return false;
      }
      // else fall out of the condition to assign result and return true.
    } else {
      data_update_condition_.wait(lock, predicate);
    }
    *result = values_.at(latest_idx_);
    return true;
  }

private:
  const size_t history_length_;
  std::vector<Timestamped<T>> values_;
  size_t latest_idx_;
  mutable std::mutex data_mutex_;
  mutable std::condition_variable data_update_condition_;
};

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
