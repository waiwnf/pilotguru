#ifndef PILOTGURU_CALIBRATION_ROTATION_HPP_
#define PILOTGURU_CALIBRATION_ROTATION_HPP_

#include <calibration/data.hpp>

#include <opencv2/core/core.hpp>

namespace pilotguru {

cv::Mat GetPrincipalRotationAxes(
    const std::vector<TimestampedRotationVelocity> &raw_rotations,
    long integration_interval_usec, double min_rotation_magnitude_threshold,
    int num_components);

std::vector<double> GetHorizontalTurnAngles(
    const std::vector<TimestampedRotationVelocity> &raw_rotations,
    const cv::Vec3d &vertical_axis);
}

#endif // PILOTGURU_CALIBRATION_ROTATION_HPP_