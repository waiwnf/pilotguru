#ifndef PILOTGURU_CALIBRATION_DATA_HPP_
#define PILOTGURU_CALIBRATION_DATA_HPP_

#include <vector>

// Datastructures and helpers for storing timestamped sensor data.

namespace pilotguru {

// Absolute velocity magnitude from GPS data, in m/sec.
struct TimestampedVelocity {
  double velocity;
  long time_usec;
};

// Rotation velocities around the 3 device axes, in radians/sec.
struct TimestampedRotationVelocity {
  double rate_x_rad_s, rate_y_rad_s, rate_z_rad_s;
  long time_usec;
};

// Accelerations along the device axes, in m/sec.
struct TimestampedAcceleration {
  double acc_x, acc_y, acc_z;
  long time_usec;
};

// From a vector of structs, extracts a vector of timestamps to pass to
// MergeTimeSeries() later.
template <typename T>
std::vector<long> ExtractTimestamps(const std::vector<T> &events) {
  std::vector<long> result;
  for (const T &event : events) {
    result.push_back(event.time_usec);
  }
  return result;
}

} // namespace pilotguru

#endif // PILOTGURU_CALIBRATION_DATA_HPP_