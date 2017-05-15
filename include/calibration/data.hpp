#ifndef PILOTGURU_CALIBRATION_DATA_HPP_
#define PILOTGURU_CALIBRATION_DATA_HPP_

// Datastructures for storing timestamped sensor data.

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

} // namespace pilotguru

#endif // PILOTGURU_CALIBRATION_DATA_HPP_