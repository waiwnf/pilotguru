#ifndef PILOTGURU_CALIBRATION_ROTATION_HPP_
#define PILOTGURU_CALIBRATION_ROTATION_HPP_

#include <calibration/data.hpp>

#include <opencv2/core/core.hpp>

namespace pilotguru {

// Extract rotation axes directions from raw_rotations, and apply PCA to get the
// principal axes, ordered by importance.
//
// Rotation events are grouped into intervals of at least
// integration_interval_usec and integrated to reduce noise and make the logic
// invariant to sampling frequency.
//
// For a rotations trajectory recorded from a car ride, with each rotation event
// in the moving device reference frame, the first principal rotation axis is
// very close to the vertical (perpendicular to the road plane) axis, unless the
// car was driving in a straight line.
cv::Mat GetPrincipalRotationAxes(
    const std::vector<TimestampedRotationVelocity> &raw_rotations,
    long integration_interval_usec);

// Project rotations onto the given axis and return a vector of projected
// rotation magnitudes.
// Useful for computing rotation magnitudes in horizontal plane (i.e. around the
// vertical axis), which correspond to vehicle steering.
std::vector<double> GetHorizontalAngularVelocities(
    const std::vector<TimestampedRotationVelocity> &raw_rotations,
    const cv::Vec3d &vertical_axis);

// Project angular velocities onto the given axis and return a vector of
// projected angular velocities. Unlike GetHorizontalTurnAngles(), does not
// integrate the rotations over time before projection. Instead, the projection
// is done directly in angular velocity space.
std::vector<double> GetAngularVelocitiesAroundAxisDirect(
    const std::vector<TimestampedRotationVelocity> &raw_rotations,
    const cv::Vec3d &axis);

// Returns the input 3D angular velocities with rotational component around the
// given axis subtracted out. This produces the complementary rotations to those
// computed by GetAngularVelocitiesAroundAxisDirect.
std::vector<TimestampedRotationVelocity> GetRotationsComplementaryToAxisDirect(
    const std::vector<TimestampedRotationVelocity> &raw_rotations,
    const cv::Vec3d &axis);
}

#endif // PILOTGURU_CALIBRATION_ROTATION_HPP_
