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
}