#include <interpolation/align_time_series.hpp>

#include <algorithm>

#include <glog/logging.h>

namespace pilotguru {

namespace {
bool IsAllIndicesNotAtEnd(
    const std::vector<size_t> &indices,
    const std::vector<const std::vector<long> *> &in_timestamps) {
  CHECK_EQ(indices.size(), in_timestamps.size());
  for (size_t i = 0; i < indices.size(); ++i) {
    if (indices.at(i) >= in_timestamps.at(i)->size()) {
      return false;
    }
  }
  return true;
}

void CheckTimestampsIncreasing(const std::vector<long> &times) {
  for (size_t i = 0; i + 1 < times.size(); ++i) {
    CHECK_LT(times.at(i), times.at(i + 1));
  }
}
}

std::vector<std::vector<size_t>>
MergeTimeSeries(const std::vector<const std::vector<long> *> &in_timestamps) {
  std::vector<std::vector<size_t>> result;

  for (const std::vector<long> *component : in_timestamps) {
    CHECK_NOTNULL(component);
    CHECK(!component->empty());
    CheckTimestampsIncreasing(*component);
  }

  // Per component start times.
  std::vector<long> start_times, end_times;
  for (const std::vector<long> *component : in_timestamps) {
    start_times.push_back(component->front());
    end_times.push_back(component->back());
  }

  // Interpolated sequence start time is the latest start time of all the
  // components. We want every component value to be known for every
  // intermediate time.
  const long start_time =
      *std::max_element(start_times.begin(), start_times.end());
  // Last valid interpolated time is the earliest end time of all the sequences.
  // Again, we want all the component values to be 'valid' for all intemediate
  // times.
  const long end_time = *std::min_element(end_times.begin(), end_times.end());

  // If one of the component sequences ends before some other sequence begins,
  // we cannot meaningfully merge anything.
  if (end_time < start_time) {
    return {};
  }

  std::vector<size_t> current_indices;
  for (const std::vector<long> *component : in_timestamps) {
    // First component point no earlier than the start time.
    const size_t component_time_idx =
        std::lower_bound(component->begin(), component->end(), start_time) -
        component->begin();
    CHECK_LT(component_time_idx, component->size());
    if (component->at(component_time_idx) > start_time) {
      CHECK_GT(component_time_idx, 0);
      current_indices.push_back(component_time_idx - 1);
    } else {
      CHECK_EQ(component->at(component_time_idx), start_time);
      current_indices.push_back(component_time_idx);
    }
  }

  while (true) {
    CHECK(IsAllIndicesNotAtEnd(current_indices, in_timestamps));
    result.push_back(current_indices);
    // Try to advance all the indices and see which one will result in the
    // earliest next time.
    std::vector<long> next_times;
    for (size_t i = 0; i < current_indices.size(); ++i) {
      const size_t next_index = current_indices.at(i) + 1;
      if (next_index >= in_timestamps.at(i)->size()) {
        return result;
      } else {
        next_times.push_back(in_timestamps.at(i)->at(next_index));
      }
    }

    // Next timestamp is the earliest of the next times.
    const long next_time =
        *std::min_element(next_times.begin(), next_times.end());
    // Advance all the indices where the next timestamp matches.
    // This is important for exactly aligned time series, e.g. vector time
    // series represented as a bunch of perfectly aligned per-component time
    // series.
    for (size_t i = 0; i < current_indices.size(); ++i) {
      const size_t next_index = current_indices.at(i) + 1;
      CHECK_LT(next_index, in_timestamps.at(i)->size());
      const long component_next_time = in_timestamps.at(i)->at(next_index);
      if (component_next_time == next_time) {
        current_indices.at(i) = next_index;
      } else {
        CHECK_GT(component_next_time, next_time);
      }
    }
  }

  return result;
}

long GetEffectiveTimeStamp(
    const std::vector<const std::vector<long> *> &component_timestamps,
    const std::vector<size_t> &event_indices) {
  CHECK(!component_timestamps.empty());
  CHECK_EQ(component_timestamps.size(), event_indices.size());
  long result = std::numeric_limits<long>::min();
  for (size_t i = 0; i < component_timestamps.size(); ++i) {
    CHECK_NOTNULL(component_timestamps.at(i));
    CHECK_LT(event_indices.at(i), component_timestamps.at(i)->size());
    result =
        std::max(result, component_timestamps.at(i)->at(event_indices.at(i)));
  }
  return result;
}

MergedTimeSeries::MergedTimeSeries(
    const std::vector<const std::vector<long> *> &component_timestamps)
    : component_timestamps_(component_timestamps),
      merged_events_(MergeTimeSeries(component_timestamps)) {}

const std::vector<std::vector<size_t>> &MergedTimeSeries::MergedEvents() const {
  return merged_events_;
}

long MergedTimeSeries::MergedEventTimeUsec(size_t event_index) const {
  CHECK_LT(event_index, merged_events_.size());
  return GetEffectiveTimeStamp(component_timestamps_,
                               merged_events_.at(event_index));
}

double InterpolationInterval::DurationSec() const {
  CHECK_LE(start_usec, end_usec);
  return static_cast<double>(end_usec - start_usec) * 1e-6;
}

long InterpolationInterval::DurationUsec() const {
  CHECK_LE(start_usec, end_usec);
  return end_usec - start_usec;
}

std::vector<std::vector<InterpolationInterval>>
MakeInterpolationIntervals(const std::vector<long> &reference_timestamps,
                           const std::vector<long> &interpolation_timestamps) {
  CheckTimestampsIncreasing(reference_timestamps);
  CheckTimestampsIncreasing(interpolation_timestamps);

  std::vector<std::vector<InterpolationInterval>> result;
  long latest_ts =
      std::min(interpolation_timestamps.front(), reference_timestamps.front());
  for (size_t reference_idx = 0, interpolation_idx = 0;
       reference_idx < reference_timestamps.size(); ++reference_idx) {
    const long reference_ts = reference_timestamps.at(reference_idx);
    // All intervals iontersecting with the current reference interval.
    std::vector<InterpolationInterval> intervals;
    while (interpolation_idx < interpolation_timestamps.size() &&
           interpolation_timestamps.at(interpolation_idx) <= reference_ts) {
      const long interpolation_ts =
          interpolation_timestamps.at(interpolation_idx);
      if (interpolation_ts > latest_ts && interpolation_idx > 0 &&
          reference_idx > 0) {
        intervals.push_back(
            {reference_idx, interpolation_idx, latest_ts, interpolation_ts});
      }
      latest_ts = interpolation_ts;
      ++interpolation_idx;
    }
    // We have reached an interpolation interval that ends after the current
    // reference interval ends. But the beginning of that interpolation interval
    // may still intersect with the current reference interval. Add a
    // corresponding intersection if it is nonempty.
    if (interpolation_idx > 0 && reference_idx > 0 &&
        interpolation_idx < interpolation_timestamps.size() &&
        reference_ts > latest_ts) {
      intervals.push_back(
          {reference_idx, interpolation_idx, latest_ts, reference_ts});
    }
    latest_ts = reference_ts;
    result.push_back(intervals);
  }

  return result;
}
}