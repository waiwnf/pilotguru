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
}

#endif // PILOTGURU_GEOMETRY_GEOMETRY_HPP_