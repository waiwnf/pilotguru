#ifndef PILOTGURU_GEOMETRY_GEOMETRY_HPP_
#define PILOTGURU_GEOMETRY_GEOMETRY_HPP_

#include <Eigen/Geometry>

namespace pilotguru {
// Overall rotation quaternion from angular velocities around the three axes and
// rotation duration.
Eigen::Quaterniond RotationMotionToQuaternion(double rate_x_rad_s,
                                              double rate_y_rad_s,
                                              double rate_z_rad_s,
                                              double duration_sec);

struct MotionIntegrationOutcome {
  Eigen::Quaterniond orientation;
  Eigen::Vector3d velocity;
  long duration_usec;
};

MotionIntegrationOutcome
IntegrateMotion(const Eigen::Quaterniond &start_orientation,
                const Eigen::Vector3d &start_velocity,
                const Eigen::Quaterniond &raw_rotation,
                const Eigen::Vector3d &raw_acceleration,
                const Eigen::Vector3d &acceleration_global_bias,
                const Eigen::Vector3d &acceleration_local_bias,
                long duration_usec);
}

#endif // PILOTGURU_GEOMETRY_GEOMETRY_HPP_