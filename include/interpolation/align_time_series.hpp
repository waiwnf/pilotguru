#ifndef PILOTGURU_INTERPOLATION_ALIGN_TIME_SERIES_HPP_
#define PILOTGURU_INTERPOLATION_ALIGN_TIME_SERIES_HPP_

#include <cstddef>
#include <memory>
#include <vector>

namespace pilotguru {

// A helper for merging multiple time series with non-aligned timestamps.
//
// The resulting merged time series has measurements at each of the components'
// timestamps (i.e. the set of merged series timestamps is a union of
// components' timestamps). The corresponding merged time series value is the
// vector of the most recent values of component time series for that given
// timestamp.
//
// Example: inputs (time, value)
// (1, 2.0), (3, 4.0), (4, 5.0), (6, 7.0), (7, 9.0)
// (2, 12.0), (3, 13.0), (4, 14.0), (5, 15.0) (6, 16.0)
//
// Merged result:
// (2, (2.0, 12.0)) - start at 2, don't know the value of sequence 2 at time 1.
// (3, (4.0, 13.0))
// (4, (5.0, 14.0))
// (5, (5.0, 15.0)) - seq. 1 has no value at 5, use the most recent value at 4.
// (6, (7.0, 16.0)) - end at 6, seq. 2 ends at 6, don't know the values after.
//
// Input: timestamps of several independent time series. each as a vector of
// long.
// Outputs: vectors indices into the each individual components for every
// timestamp of the merged sequence.
//
std::vector<std::vector<size_t>>
MergeTimeSeries(const std::vector<const std::vector<long> *> &in_timestamps);
}

#endif // PILOTGURU_INTERPOLATION_ALIGN_TIME_SERIES_HPP_