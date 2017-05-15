#include <calibration/rotation.hpp>
#include <geometry/geometry.hpp>
#include <logging/strings.hpp>

#include <Eigen/Geometry>

#include <glog/logging.h>

namespace pilotguru {

namespace {
cv::Vec3d GetRotationAxis(const Eigen::Quaterniond &q) {
  cv::Vec3d result(q.x(), q.y(), q.z());
  if (result[0] >= 0) {
    return result;
  } else {
    return -result;
  }
}

// Permute the cluster centers by the decreasing number of the corresponding
// labels in the input data.
cv::Mat OrderClusterCentersByLabelCount(const cv::Mat &cluster_centers,
                                        const cv::Mat &example_labels) {
  CHECK_EQ(example_labels.type(), CV_32SC1);

  // Count label occurences.
  std::map<int, size_t> label_counts;
  for (int label = 0; label < cluster_centers.rows; ++label) {
    label_counts[label] = 0;
  }
  for (int i = 0; i < example_labels.rows; ++i) {
    const int label = example_labels.at<int>(i);
    CHECK(label_counts.find(label) != label_counts.end());
    ++label_counts[label];
  }

  // Sort cluster center indices by the number of occurences in the labels.
  std::vector<std::pair<int, int>> label_counts_vec(label_counts.begin(),
                                                    label_counts.end());
  std::sort(
      label_counts_vec.begin(), label_counts_vec.end(),
      [](const std::pair<int, int> &left, const std::pair<int, int> &right) {
        if (left.second != right.second) {
          return left.second < right.second;
        } else {
          return left.first < right.first;
        }
      });
  // Default sorting is in increasing order, we need the largest counts first.
  std::reverse(label_counts_vec.begin(), label_counts_vec.end());

  // Copy the rows to a new matrix in the right order.
  cv::Mat sorted_axes(cluster_centers.rows, cluster_centers.cols, CV_32FC1);
  for (int i = 0; i < cluster_centers.rows; ++i) {
    cluster_centers.row(label_counts_vec.at(i).first)
        .copyTo(sorted_axes.row(i));
  }

  return sorted_axes;
}
} // namespace

cv::Mat GetPrincipalRotationAxes(
    const std::vector<TimestampedRotationVelocity> &raw_rotations,
    long integration_interval_usec, double min_rotation_magnitude_threshold,
    int num_components) {
  CHECK_GT(integration_interval_usec, 0);
  CHECK_GE(min_rotation_magnitude_threshold, 0);

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
      const cv::Vec3d axis = GetRotationAxis(current_interval_rotation);
      const double axis_norm = cv::norm(axis, cv::NORM_L2);
      if (axis_norm > min_rotation_magnitude_threshold) {
        interval_rotations.push_back(axis / axis_norm);
      }
      current_interval_rotation = Eigen::Quaterniond(1, 0, 0, 0);
      current_interval_usec = 0;
    }
  }

  CHECK_GE(interval_rotations.size(), num_components);
  // cv::kmeans() wants float (not double) inputs for whatever reason, so use
  // CV_32FC1 instead of CV_64FC1 here.
  cv::Mat input_axes(interval_rotations.size(), 3, CV_32FC1);
  for (size_t row = 0; row < interval_rotations.size(); ++row) {
    for (size_t col = 0; col < 3; ++col) {
      input_axes.at<float>(row, col) = interval_rotations.at(row)[col];
    }
  }

  cv::Mat cluster_centers, labels;
  cv::kmeans(input_axes, num_components, labels,
             cv::TermCriteria(cv::TermCriteria::EPS, 100, 1e-3), 3,
             cv::KMEANS_PP_CENTERS, cluster_centers);

  return OrderClusterCentersByLabelCount(cluster_centers, labels);
}

std::vector<double> GetHorizontalTurnAngles(
    const std::vector<TimestampedRotationVelocity> &raw_rotations,
    const cv::Vec3d &vertical_axis) {
  const double axis_norm = cv::norm(vertical_axis, cv::NORM_L2);
  // The axis should be normalized.
  CHECK_GT(axis_norm, 0.9);
  CHECK_LT(axis_norm, 1.1);

  std::vector<double> result;
  for (size_t rotation_idx = 1; rotation_idx < raw_rotations.size();
       ++rotation_idx) {
    const TimestampedRotationVelocity &raw_rotation =
        raw_rotations.at(rotation_idx);
    const long rotation_duration_usec =
        raw_rotation.time_usec - raw_rotations.at(rotation_idx - 1).time_usec;
    const Eigen::Quaterniond rotation_quaternion = RotationMotionToQuaternion(
        raw_rotation.rate_x_rad_s, raw_rotation.rate_y_rad_s,
        raw_rotation.rate_z_rad_s,
        static_cast<double>(rotation_duration_usec) * 1e-6);
    const cv::Vec3d quaternion_axis(rotation_quaternion.x(),
                                    rotation_quaternion.y(),
                                    rotation_quaternion.z());
    const double rotation_axis_dot = vertical_axis.dot(quaternion_axis);
    const cv::Vec3d axis_projection =
        vertical_axis * rotation_axis_dot / (axis_norm * axis_norm);

    // TODO check for singularity when the projection is near 0.

    Eigen::Quaterniond projected_quaternion(
        rotation_quaternion.w(), axis_projection[0], axis_projection[1],
        axis_projection[2]);
    projected_quaternion.normalize();
    const double angle =
        2.0 * std::asin(std::sqrt(
                  1.0 - projected_quaternion.w() * projected_quaternion.w()));

    result.push_back(rotation_axis_dot >= 0 ? angle : -angle);
  }
  return result;
}
} // namespace pilotguru