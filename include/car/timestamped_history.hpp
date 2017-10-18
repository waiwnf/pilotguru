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

} // namespace pilotguru

#endif // PILOTGURU_CAR_TIMESTAMPED_HISTORY_HPP_
