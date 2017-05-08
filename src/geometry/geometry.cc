#include <geometry/geometry.hpp>

namespace pilotguru {
Eigen::Quaterniond RotationMotionToQuaternion(double rate_x_rad_s,
                                              double rate_y_rad_s,
                                              double rate_z_rad_s,
                                              double duration_sec) {
  const double rate_overall_rad_s =
      sqrt(rate_x_rad_s * rate_x_rad_s + rate_y_rad_s * rate_y_rad_s +
           rate_z_rad_s * rate_z_rad_s);
  const double half_theta = rate_overall_rad_s * duration_sec * 0.5;
  const double sin_half_theta_normalized = sin(half_theta) / rate_overall_rad_s;

  const double dw = cos(half_theta);
  const double dx = rate_x_rad_s * sin_half_theta_normalized;
  const double dy = rate_y_rad_s * sin_half_theta_normalized;
  const double dz = rate_z_rad_s * sin_half_theta_normalized;
  return {dw, dx, dy, dz};
}
}