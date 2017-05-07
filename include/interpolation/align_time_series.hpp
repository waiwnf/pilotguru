#ifndef PILOTGURU_INTERPOLATION_ALIGN_TIME_SERIES_HPP_
#define PILOTGURU_INTERPOLATION_ALIGN_TIME_SERIES_HPP_

#include <cstddef>
#include <memory>
#include <vector>

namespace pilotguru {

// A helper for merging multiple time series with non-aligned timestamps.
//
// Takes in one vector of timestamps per component time series.
// Produces a vector, where each element is a vector of indices into the
// corresponding component time series vector. The effective timestamp of every
// result element is the latest timestamp among the component time series.

// Example: two input time series with timestamps
// 1, 3, 4, 6, 7
// 2, 3, 4, 5  6
//
// Merged result:
// (0,0)  (effective time 2)
// (1,1)  (effective time 3)
// (2,2)  (effective time 4)
// (2,3)  (effective time 5)
// (3,4)  (effective time 6)
std::vector<std::vector<size_t>>
MergeTimeSeries(const std::vector<const std::vector<long> *> &in_timestamps);

// Timing datastructures for fitting fine-grained motion to a series of coarse
// 'reference' measurements. The use case is to fit a trajectory defined by
// noisy
// fine-grained information (e.g. accelerometer readings at ~200Hz) while
// matching the averages on coarse intervals (GPS measurements at ~1Hz) and
// thereby calibrate the accelerometer data "online".

// An interval (start_usec, end_usec] corresponding to an intersection of the
// reference measurement interval (i.e. an interval between two neighboring GPS
// measurements) with an interpolation interval (an interval between neighboring
// accelerometer/gyroscope readings).
//
// Reference and interpolation intervals are represented as indices into
// corresponding vectors of measurement timestamps (external to this struct). By
// convention an interval is represented by the index of its ending timestep.
struct InterpolationInterval {
  // Index of the end timestamp of the reference interval.
  size_t reference_end_time_index;
  // Index of the end timestamp of the interpolation interval.
  size_t interpolation_end_time_index;
  long start_usec;
  long end_usec;

  double DurationSec() const;
};

// Make all intervals (see InterpolationInterval above) corresponding to
// intersection of the intervals defined by reference timestamps and
// interpolation timestamps.
std::vector<std::vector<InterpolationInterval>>
MakeInterpolationIntervals(const std::vector<long> &reference_timestamps,
                           const std::vector<long> &interpolation_timestamps);
}

#endif // PILOTGURU_INTERPOLATION_ALIGN_TIME_SERIES_HPP_