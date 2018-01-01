#include <geometry/geometry.hpp>

#include <glog/logging.h>

namespace pilotguru {
Eigen::Quaterniond RotationMotionToQuaternion(double rate_x_rad_s,
                                              double rate_y_rad_s,
                                              double rate_z_rad_s,
                                              double duration_sec) {
  const double rate_overall_rad_s =
      sqrt(rate_x_rad_s * rate_x_rad_s + rate_y_rad_s * rate_y_rad_s +
           rate_z_rad_s * rate_z_rad_s);
  const double half_theta = rate_overall_rad_s * duration_sec * 0.5;
  const double sin_half_theta_normalized =
      sin(half_theta) / (rate_overall_rad_s + 1e-30);

  const double dw = cos(half_theta);
  const double dx = rate_x_rad_s * sin_half_theta_normalized;
  const double dy = rate_y_rad_s * sin_half_theta_normalized;
  const double dz = rate_z_rad_s * sin_half_theta_normalized;
  return {dw, dx, dy, dz};
}

MotionIntegrationOutcome
IntegrateMotion(const Eigen::Quaterniond &start_orientation,
                const Eigen::Vector3d &start_velocity,
                const Eigen::Quaterniond &raw_rotation,
                const Eigen::Vector3d &raw_acceleration,
                const Eigen::Vector3d &acceleration_global_bias,
                const Eigen::Vector3d &acceleration_local_bias,
                long duration_usec) {
  CHECK_GE(duration_usec, 0);
  const double duration_sec = static_cast<double>(duration_usec) * 1e-6;

  // Local bias.
  const Eigen::Vector3d acceleration_local_calibrated =
      raw_acceleration + acceleration_local_bias;
  // Rotate to the fixed reference frame.
  const Eigen::Vector3d acceleration_rotated =
      start_orientation._transformVector(acceleration_local_calibrated);
  // Fixed reference frame bias (~gravity).
  const Eigen::Vector3d acceleration_global =
      acceleration_rotated + acceleration_global_bias;
  // First integration yields velocity.
  const Eigen::Vector3d result_velocity =
      start_velocity + acceleration_global * duration_sec;

  // Integrate rotation.
  const Eigen::Quaterniond result_orientation =
      start_orientation * raw_rotation;

  return {result_orientation, result_velocity, duration_usec};
}
}
