#ifndef PILOTGURU_CAR_TIMESTAMPED_HISTORY_HPP_
#define PILOTGURU_CAR_TIMESTAMPED_HISTORY_HPP_

#include <sys/time.h>

#include <algorithm>
#include <chrono>
#include <condition_variable>
#include <mutex>
#include <vector>

namespace pilotguru {

template <typename T> class Timestamped {
public:
  Timestamped() : timestamp_({0, 0}) {}
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
    num_valid_values_ = std::min(num_valid_values_ + 1, values_.size());
    latest_idx_ = (latest_idx_ + 1) % history_length_;
    values_.at(latest_idx_) = Timestamped<T>(t, timestamp);
    data_update_condition_.notify_all();
  }

  // Returns a copy of the history properly sorted from the oldest to the most
  // recent element.
  // TODO only copy the valid values.
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

  bool get_latest(Timestamped<T> *result) const {
    std::unique_lock<std::mutex> lock(data_mutex_);
    if (num_valid_values_ > 0) {
      *result = values_.at(latest_idx_);
      return true;
    } else {
      return false;
    }
  }

  bool wait_get_next(const timeval &prev_timestamp, const timeval *timeout,
                     Timestamped<T> *result) const {
    if (result == nullptr) {
      return false;
    }
    std::unique_lock<std::mutex> lock(data_mutex_);
    auto predicate = [this, &prev_timestamp]() {
      return !(num_valid_values_ <= 0 /* no data yet */ ||
               (prev_timestamp.tv_sec ==
                    this->values_.at(this->latest_idx_).timestamp().tv_sec &&
                prev_timestamp.tv_usec ==
                    this->values_.at(this->latest_idx_).timestamp().tv_usec));
    };
    if (timeout != nullptr) {
      const std::chrono::microseconds timeout_micros =
          std::chrono::seconds(timeout->tv_sec) +
          std::chrono::microseconds(timeout->tv_usec);
      const bool wait_result =
          data_update_condition_.wait_for(lock, timeout_micros, predicate);
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
  size_t num_valid_values_ = 0;
  mutable std::mutex data_mutex_;
  mutable std::condition_variable data_update_condition_;
};

// Logic for computing wait timeouts to make sure that a loop, including the
// wait, takes at most a fixed amount of time. To compute effective remaining
// timeout duration, automatically subtracts the time taken by the rest of the
// loop from the maximum allowed wait time.
//
// Intended usage waiting for new elements from TimestampedHistory:
//
// LoopWaitEffectiveTimeout loop_timeout(max_wait);
// while(loop condition) {
//   timeval remaining_timeout = loop_timeout.GetRemainingTimeout();
//   bool wait_result = history.wait_get_next(..., &remaining_timeout, ...);
//   loop_timeout.WaitFinished();
//   ... rest of the loop logic ...
// }
class LoopWaitEffectiveTimeout {
public:
  LoopWaitEffectiveTimeout(timeval total_timeout)
      : total_timeout_(total_timeout) {
    gettimeofday(&previous_loop_time_, nullptr);
  }

  // Update the iteration start timestamp to the current time.
  void WaitFinished() { gettimeofday(&previous_loop_time_, nullptr); }

  // Returns the remaining possible wait duration so that the wait since the
  // previous WaitFinished() call does not exceed total_timeout.
  timeval GetRemainingTimeout() const {
    timeval current_time, time_since_prev_loop, effective_timeout;
    gettimeofday(&current_time, nullptr);
    timersub(&current_time, &previous_loop_time_, &time_since_prev_loop);
    if (timercmp(&time_since_prev_loop, &total_timeout_, <)) {
      timersub(&total_timeout_, &time_since_prev_loop, &effective_timeout);
    } else {
      effective_timeout = {0, 0};
    }
    return effective_timeout;
  }

private:
  timeval total_timeout_, previous_loop_time_;
};

} // namespace pilotguru

#endif // PILOTGURU_CAR_TIMESTAMPED_HISTORY_HPP_
