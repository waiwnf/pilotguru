#include <io/json_constants.hpp>
#include <slam/track_image_sequence.hpp>

#include <opencv2/highgui/highgui.hpp>

#include <Eigen/Geometry>

#include <json.hpp>

namespace pilotguru {
namespace {

nlohmann::json PoseToJson(const ORB_SLAM2::Pose &pose) {
  nlohmann::json result;
  result[kTranslation] = {pose.translation(0), pose.translation(1),
                          pose.translation(2)};
  nlohmann::json rotation;
  rotation[kW] = pose.rotation.w();
  rotation[kX] = pose.rotation.x();
  rotation[kY] = pose.rotation.y();
  rotation[kZ] = pose.rotation.z();
  result[kRotation] = rotation;

  return result;
}

std::unique_ptr<cv::PCA>
TrajectoryToPCA(const std::vector<ORB_SLAM2::PoseWithTimestamp> &trajectory) {
  cv::Mat trajectory_matrix(3, trajectory.size(), CV_64FC1);
  for (size_t point_idx = 0; point_idx < trajectory.size(); ++point_idx) {
    const cv::Vec3d &translation = trajectory.at(point_idx).pose.translation;
    trajectory_matrix.at<double>(0, point_idx) = translation(0);
    trajectory_matrix.at<double>(1, point_idx) = translation(1);
    trajectory_matrix.at<double>(2, point_idx) = translation(2);
  }
  // Retain all 3 eigenvectors to check eigenvalues and make sure the
  // last one is insignificant.
  return std::unique_ptr<cv::PCA>(
      new cv::PCA(trajectory_matrix, cv::noArray(), CV_PCA_DATA_AS_COL));
}

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
ProjectDirectionsToTurnAngles(const std::vector<cv::Mat> &directions) {
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

void SetPlane(nlohmann::json *json_root, const cv::Mat &plane) {
  CHECK_EQ(plane.rows, 2);
  CHECK_EQ(plane.cols, 3);
  (*CHECK_NOTNULL(json_root))[kPlane] = {
      {plane.at<double>(0, 0), plane.at<double>(0, 1), plane.at<double>(0, 2)},
      {plane.at<double>(1, 0), plane.at<double>(1, 1), plane.at<double>(1, 2)}};
}

void SetTrajectory(nlohmann::json *json_root,
                   const std::vector<ORB_SLAM2::PoseWithTimestamp> &trajectory,
                   const std::vector<cv::Mat> &projected_directions,
                   const vector<double> &turn_angles, int frame_id_offset) {
  CHECK_NOTNULL(json_root);
  CHECK_EQ(trajectory.size(), projected_directions.size());
  CHECK_EQ(trajectory.size(), turn_angles.size());
  (*json_root)[kTrajectory] = {};
  for (size_t point_idx = 0; point_idx < trajectory.size(); ++point_idx) {
    nlohmann::json point_json;
    const ORB_SLAM2::PoseWithTimestamp& point = trajectory.at(point_idx);
    point_json[kTimeUsec] = point.time_usec;
    point_json[kIsLost] = point.is_lost;
    point_json[kFrameId] = point.frame_id - frame_id_offset;
    point_json[kPose] = PoseToJson(point.pose);

    const cv::Mat &direction = projected_directions.at(point_idx);
    point_json[kPlanarDirection] = {direction.at<double>(0, 0),
                                    direction.at<double>(0, 1)};
    point_json[kTurnAngle] = turn_angles.at(point_idx);

    (*json_root)[kTrajectory].push_back(point_json);
  }
}
} // namespace

bool TrackImageSequence(ORB_SLAM2::System *SLAM, ImageSequenceSource &image_source,
                        const std::string &trajectory_out_file,
                        ImageSequenceSink *tracked_frames_sink) {
  CHECK_NOTNULL(SLAM);

  ORB_SLAM2::TimestampedImage frameImage;
  int tracked = 0;
  int first_tracked_frame_id = 0;
  while (image_source.hasNext()) {
    frameImage = image_source.next();
    CHECK(!frameImage.image.empty());

    SLAM->TrackMonocular(frameImage);
    const ORB_SLAM2::eTrackingState tracking_state = SLAM->tracker().state();

    if (tracking_state == ORB_SLAM2::LOST) {
      break;
    }
    if (tracked == 0 && tracking_state == ORB_SLAM2::OK) {
      first_tracked_frame_id = frameImage.frame_id;
    }
    tracked += (tracking_state == ORB_SLAM2::OK);

    if (tracking_state == ORB_SLAM2::OK && tracked_frames_sink != nullptr) {
      tracked_frames_sink->consume(frameImage.image);
    }
  }

  const std::vector<ORB_SLAM2::PoseWithTimestamp> trajectory = SLAM->GetTrajectory();
  if (trajectory.empty()) {
    return false;
  }

  // TODO rotation smoothing.

  // PCA to infer the horizontal plane (assuming the horizontal motion is
  // dominant).
  std::unique_ptr<cv::PCA> trajectory_pca = TrajectoryToPCA(trajectory);

  // TODO: make sure the eigenvectors form a right-hand-side coordinate
  // system and the 3rd vector points roughly upwards.
  LOG(INFO) << "PCA eigenvectors: " << trajectory_pca->eigenvectors;
  LOG(INFO) << "PCA eigenvalues: " << trajectory_pca->eigenvalues;

  // Bail out if the last eigenvector has too  large (relative) weight. This
  // means that the vertical motion was not negligible and the horizontal plane
  // may not be sufficiently precise.
  // TODO: figure out a good way for the threshold here.
  if (trajectory_pca->eigenvalues.at<double>(2) >
      trajectory_pca->eigenvalues.at<double>(1) * 1e-2) {
    LOG(WARNING) << "3rd eigenvalue was too large, dropping the trajectory. "
                    "Relative magnitude wrt the 2nd eigenvalue: "
                 << trajectory_pca->eigenvalues.at<double>(2) /
                        trajectory_pca->eigenvalues.at<double>(1);
    return false;
  }

  const cv::Mat pca_plane = trajectory_pca->eigenvectors.rowRange(0, 2);
  std::unique_ptr<std::vector<cv::Mat>> projected_directions =
      ProjectDirections(trajectory, pca_plane);

  const vector<double> turn_angles =
      ProjectDirectionsToTurnAngles(*projected_directions);

  nlohmann::json trajectory_json;
  SetPlane(&trajectory_json, pca_plane);
  const int frame_id_offset =
      tracked_frames_sink == nullptr ? 0 : first_tracked_frame_id;
  SetTrajectory(&trajectory_json, trajectory, *projected_directions,
                turn_angles, frame_id_offset);

  std::ofstream trajectory_ostream(trajectory_out_file);
  trajectory_ostream << trajectory_json.dump(2) << std::endl;

  return true;
}

} // namespace pilotguru
