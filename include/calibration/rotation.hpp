#ifndef PILOTGURU_CALIBRATION_ROTATION_HPP_
#define PILOTGURU_CALIBRATION_ROTATION_HPP_

#include <calibration/data.hpp>

#include <opencv2/core/core.hpp>

namespace pilotguru {

// Extract rotation axes directions from raw_rotations, cluster with k-means
// into num_components and return cluster centers ordered by the number of
// cluster sizes. Rotation events are grouped into intervals of at least
// integration_interval_usec and integrated to reduce noise and make the logic
// invariant to sampling frequency.
//
// Integrated intervals with overall rotation magnitude (i.e. angle) less than
// threshold are ignored.
//
// For a rotations trajectory recorded from a car ride, with each rotation event
// in the moving device reference frame, the first principal rotation axis is
// very close to the vertical (perpendicular to the road plane) axis, unless the
// car was driving in a straight line.
cv::Mat GetPrincipalRotationAxes(
    const std::vector<TimestampedRotationVelocity> &raw_rotations,
    long integration_interval_usec, double min_rotation_magnitude_threshold,
    int num_components);

// Project rotations onto the given axis and return a vector of projected
// rotation magnitudes.
// Useful for computing rotation magnitudes in horizontal plane (i.e. around the
// vertical axis), which correspond to vehicle steering.
std::vector<double> GetHorizontalTurnAngles(
    const std::vector<TimestampedRotationVelocity> &raw_rotations,
    const cv::Vec3d &vertical_axis);
}

#endif // PILOTGURU_CALIBRATION_ROTATION_HPP_