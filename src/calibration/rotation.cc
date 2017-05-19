#include <calibration/rotation.hpp>

#include <geometry/geometry.hpp>
#include <logging/strings.hpp>

#include <opencv2/imgproc/imgproc.hpp>

#include <Eigen/Geometry>

#include <glog/logging.h>

namespace pilotguru {

cv::Mat GetPrincipalRotationAxes(
    const std::vector<TimestampedRotationVelocity> &raw_rotations,
    long integration_interval_usec) {
  CHECK_GT(integration_interval_usec, 0);

  std::vector<cv::Vec3d> interval_rotations;
  Eigen::Quaterniond current_interval_rotation(1, 0, 0, 0);
  long current_interval_usec = 0;
  for (size_t rotation_idx = 1; rotation_idx < raw_rotations.size();
       ++rotation_idx) {
    const TimestampedRotationVelocity &raw_rotation =
        raw_rotations.at(rotation_idx);
    const long rotation_duration_usec =
        raw_rotation.time_usec - raw_rotations.at(rotation_idx - 1).time_usec;
    current_interval_usec += rotation_duration_usec;

    const Eigen::Quaterniond rotation_quaternion = RotationMotionToQuaternion(
        raw_rotation.rate_x_rad_s, raw_rotation.rate_y_rad_s,
        raw_rotation.rate_z_rad_s,
        static_cast<double>(rotation_duration_usec) * 1e-6);
    current_interval_rotation = current_interval_rotation * rotation_quaternion;

    if (current_interval_usec >= integration_interval_usec) {
      interval_rotations.emplace_back(current_interval_rotation.x(),
                                      current_interval_rotation.y(),
                                      current_interval_rotation.z());
      current_interval_rotation = Eigen::Quaterniond(1, 0, 0, 0);
      current_interval_usec = 0;
    }
  }

  CHECK_GE(interval_rotations.size(), 3);
  cv::Mat rotations_matrix(interval_rotations.size(), 3, CV_64FC1);
  for (size_t row = 0; row < interval_rotations.size(); ++row) {
    for (size_t col = 0; col < 3; ++col) {
      rotations_matrix.at<double>(row, col) = interval_rotations.at(row)[col];
    }
  }

  cv::PCA rotations_pca(rotations_matrix, cv::noArray(), CV_PCA_DATA_AS_ROW);
  return rotations_pca.eigenvectors;
}

std::vector<double> GetHorizontalTurnAngles(
    const std::vector<TimestampedRotationVelocity> &raw_rotations,
    const cv::Vec3d &vertical_axis) {
  const double axis_norm = cv::norm(vertical_axis, cv::NORM_L2);
  // The axis should be normalized.
  CHECK_GT(axis_norm, 0.9);
  CHECK_LT(axis_norm, 1.1);

  std::vector<double> result{0};
  for (size_t rotation_idx = 1; rotation_idx < raw_rotations.size();
       ++rotation_idx) {
    const TimestampedRotationVelocity &raw_rotation =
        raw_rotations.at(rotation_idx);
    const double rotation_duration_sec =
        static_cast<double>(raw_rotation.time_usec -
                            raw_rotations.at(rotation_idx - 1).time_usec) *
        1e-6;
    const Eigen::Quaterniond rotation_quaternion = RotationMotionToQuaternion(
        raw_rotation.rate_x_rad_s, raw_rotation.rate_y_rad_s,
        raw_rotation.rate_z_rad_s, rotation_duration_sec);
    const cv::Vec3d quaternion_axis(rotation_quaternion.x(),
                                    rotation_quaternion.y(),
                                    rotation_quaternion.z());
    const double rotation_axis_dot = vertical_axis.dot(quaternion_axis);
    const cv::Vec3d axis_projection =
        vertical_axis * std::abs(rotation_axis_dot) / (axis_norm * axis_norm);

    // TODO check for singularity when the projection is near 0.

    Eigen::Quaterniond projected_quaternion(
        rotation_quaternion.w(), axis_projection[0], axis_projection[1],
        axis_projection[2]);
    projected_quaternion.normalize();
    const double angular_velocity =
        2.0 * std::asin(std::sqrt(
                  1.0 - projected_quaternion.w() * projected_quaternion.w())) /
        (rotation_duration_sec + 1e-10);

    result.push_back(rotation_axis_dot >= 0 ? angular_velocity
                                            : -angular_velocity);
  }
  return result;
}
} // namespace pilotguru