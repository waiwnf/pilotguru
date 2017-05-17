#ifndef PILOTGURU_INTERPOLATION_ALIGN_TIME_SERIES_HPP_
#define PILOTGURU_INTERPOLATION_ALIGN_TIME_SERIES_HPP_

#include <limits>
#include <memory>
#include <string>
#include <vector>

#include <io/json_converters.hpp>

#include <glog/logging.h>

#include <json.hpp>

namespace pilotguru {

// A class to represent a time series, supporting
// - reading from a JSON file.
// - lookups of the most recent value no later than a fixed time.
// - time-weighted averaging for a given interval.
template <typename T> class TimeSeries {
public:
  struct ValueLookupResult {
    T value;
    // This is false if a meaningful value cannot be computed, e.g. when a value
    // is requested for a timestamped that is earlier than the first value of
    // the time series.
    bool is_valid;
    // Latest event of this time series no later than the end time of the query.
    // The end time of the query is either the timestamps of the point query
    // (for MostRecentPreviousValue()) or the end of the query interval (for
    // TimeAveragedValue()).
    size_t end_index;
  };

  virtual ~TimeSeries() {}

  const std::vector<long> &TimesUsec() const { return times_usec_; }

  const std::vector<T> &Values() const { return values_; }

  // Most recent value no later than time_usec.
  // Uses linear (not binary!) search over the timestamps, for better efficiency
  // of sequential scanning.
  // start_index_hint is the event index from which to start searching.
  virtual ValueLookupResult
  MostRecentPreviousValue(long time_usec, size_t start_index_hint = 0) const {
    CHECK_LT(start_index_hint, times_usec_.size());

    // Cannot provide a valid answer if the earliest value comes after the
    // target time.
    if (times_usec_.front() > time_usec) {
      return {InvalidValue(), false, 0};
    }

    // Start index hint must be valid (i.e. no later than the target timestamp).
    CHECK_LE(times_usec_.at(start_index_hint), time_usec);

    // Move the index until the event time crosses the requested time.
    size_t result_next_idx = start_index_hint;
    while (result_next_idx < times_usec_.size() &&
           times_usec_.at(result_next_idx) <= time_usec) {
      ++result_next_idx;
    }
    CHECK_GT(result_next_idx, 0);
    const size_t result_idx = result_next_idx - 1;
    return {values_.at(result_idx), true, result_idx};
  }

  // Time-weighted average value over a given interval. The interval must be
  // fully covered by the time series.
  // start_index_hint is the index from which to start linear search.
  // start_index_hint timestamp must be no later than the requested start time.
  virtual ValueLookupResult
  TimeAveragedValue(long start_time_usec, long end_time_usec,
                    size_t start_index_hint = 0) const {
    CHECK_LT(start_index_hint, times_usec_.size());
    CHECK_GT(end_time_usec, start_time_usec);

    // Only return a valid result if the target interval is fully covered by the
    // time series.
    if (start_time_usec < times_usec_.front() ||
        end_time_usec > times_usec_.back()) {
      return {InvalidValue(), false, 0};
    }

    const ValueLookupResult start_event =
        MostRecentPreviousValue(start_time_usec, start_index_hint);
    CHECK(start_event.is_valid);
    const ValueLookupResult end_event =
        MostRecentPreviousValue(end_time_usec, start_event.end_index);
    CHECK(end_event.is_valid);

    T total_weighted_value = Zero();
    // Sum up all the data intervals that are wholly inside the target interval.
    for (size_t inner_idx = start_event.end_index + 1;
         inner_idx < end_event.end_index; ++inner_idx) {
      const double inner_interval_sec =
          static_cast<double>(times_usec_.at(inner_idx + 1) -
                              times_usec_.at(inner_idx)) *
          1e-6;
      total_weighted_value +=
          (inner_interval_sec * 0.5 *
           (values_.at(inner_idx) + values_.at(inner_idx + 1)));
    }

    const T left_value = LinearInterpolate(
        start_event.end_index, start_event.end_index + 1, start_time_usec);
    const T right_value = LinearInterpolate(
        end_event.end_index, end_event.end_index + 1, end_time_usec);

    if (start_event.end_index == end_event.end_index) {
      total_weighted_value += (left_value + right_value) * 0.5 *
                              IntervalSec(start_time_usec, end_time_usec);
    } else {
      total_weighted_value +=
          (left_value + values_.at(start_event.end_index + 1)) * 0.5 *
          IntervalSec(start_time_usec,
                      times_usec_.at(start_event.end_index + 1));
      total_weighted_value +=
          (values_.at(end_event.end_index) + right_value) * 0.5 *
          IntervalSec(times_usec_.at(end_event.end_index), end_time_usec);
    }

    const double target_duration_sec =
        IntervalSec(start_time_usec, end_time_usec);
    return {total_weighted_value / target_duration_sec, true,
            end_event.end_index};
  }

protected:
  // To be used in constructors of derived classes.
  // Requires ReadValue() to be implemented.
  virtual void ReadJson(const std::string &filename,
                        const std::string &root_element_name) {
    std::unique_ptr<nlohmann::json> json_raw = ReadJsonFile(filename);
    for (const nlohmann::json &event : (*json_raw)[root_element_name]) {
      times_usec_.push_back(ReadTimestamp(event));
      values_.push_back(ReadValue(event));
    }
  }

  virtual long ReadTimestamp(const nlohmann::json &event) {
    return event[kTimeUsec];
  }

  // Linear interpolation between two evens with given indices for a given
  // target time.
  virtual T LinearInterpolate(size_t left_idx, size_t right_idx,
                              long target_time_usec) const {
    CHECK_LT(left_idx, right_idx);
    CHECK_LT(right_idx, values_.size());
    CHECK_LE(times_usec_.at(left_idx), target_time_usec);
    CHECK_LE(target_time_usec, times_usec_.at(right_idx));

    const double left_time_sec =
        IntervalSec(times_usec_.at(left_idx), target_time_usec);
    const double right_time_sec =
        IntervalSec(target_time_usec, times_usec_.at(right_idx));
    const double total_time_sec =
        IntervalSec(times_usec_.at(left_idx), times_usec_.at(right_idx));

    return (left_time_sec / total_time_sec) * values_.at(right_idx) +
           (right_time_sec / total_time_sec) * values_.at(left_idx);
  }

  // Interval duration in seconds from microsecond timestamps.
  double IntervalSec(long time_usec_left, long time_usec_right) const {
    CHECK_LE(time_usec_left, time_usec_right);
    return static_cast<double>(time_usec_right - time_usec_left) * 1e-6;
  }

  virtual T ReadValue(const nlohmann::json &event) = 0;

  virtual T InvalidValue() const = 0;

  virtual T Zero() const = 0;

private:
  std::vector<long> times_usec_;
  std::vector<T> values_;
};

class RealTimeSeries : public TimeSeries<double> {
public:
  RealTimeSeries(const std::string &filename,
                 const std::string &root_element_name,
                 const std::string &value_name)
      : value_name_(value_name) {
    ReadJson(filename, root_element_name);
  }

protected:
  double ReadValue(const nlohmann::json &event) override {
    return event[value_name_];
  }

  double InvalidValue() const override {
    return std::numeric_limits<double>::quiet_NaN();
  }

  double Zero() const override { return 0; }

  const std::string value_name_;
};
} // namespace

#endif // PILOTGURU_INTERPOLATION_ALIGN_TIME_SERIES_HPP_