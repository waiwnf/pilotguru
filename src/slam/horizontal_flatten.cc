#include <slam/horizontal_flatten.hpp>

#include <Eigen/Geometry>

namespace pilotguru {

std::unique_ptr<std::vector<cv::Mat>>
ProjectDirections(const std::vector<ORB_SLAM2::PoseWithTimestamp> &trajectory,
                  const cv::Mat &projection_plane) {
  // Z axis in camera reference frame is the optical axis of the camera.
  const Eigen::Vector3d z_axis(0, 0, 1);
  std::unique_ptr<std::vector<cv::Mat>> projected_directions(
      new std::vector<cv::Mat>());
  for (size_t point_idx = 0; point_idx < trajectory.size(); ++point_idx) {
    const Eigen::Vector3d camera_direction =
        trajectory.at(point_idx).pose.rotation._transformVector(z_axis);
    // We are not using pca::project() here because we need to project unit
    // vectors corresponding to axes directions, not the translation vectors
    // that the PCA was built on. So we do not need to worry about the mean
    // of the PCA data, just about the eigenvectors themselves.
    projected_directions->push_back(
        projection_plane *
        cv::Mat(cv::Vec3d(camera_direction(0), camera_direction(1),
                          camera_direction(2))));
    CHECK_EQ(projected_directions->back().rows, 2);
    CHECK_EQ(projected_directions->back().cols, 1);
  }
  return std::move(projected_directions);
}

vector<double>
Projected2DDirectionsToTurnAngles(const std::vector<cv::Mat> &directions) {
  vector<double> turn_angles(directions.size(), 0);
  // Start with 1 because we need the direction for the previous frame to
  // compute the change.
  for (size_t point_idx = 1; point_idx < directions.size(); ++point_idx) {
    const cv::Mat &prev_mat = directions.at(point_idx - 1);
    const cv::Vec3d prev(prev_mat.at<double>(0, 0), prev_mat.at<double>(0, 1),
                         0);
    const cv::Mat &curr_mat = directions.at(point_idx);
    const cv::Vec3d curr(curr_mat.at<double>(0, 0), curr_mat.at<double>(0, 1),
                         0);

    const double rotation_cos = prev.dot(curr) / cv::norm(prev, cv::NORM_L2) /
                                cv::norm(curr, cv::NORM_L2);
    const cv::Vec3d cross_product = prev.cross(curr);
    turn_angles.at(point_idx) =
        acos(rotation_cos) * (cross_product(2) > 0 ? 1.0 : -1.0);
  }
  return turn_angles;
}

} // namespace pilotguru