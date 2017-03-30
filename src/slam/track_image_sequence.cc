#include <io/json_converters.hpp>
#include <slam/horizontal_flatten.hpp>
#include <slam/smoothing.hpp>
#include <slam/track_image_sequence.hpp>

#include <opencv2/highgui/highgui.hpp>
#include <opencv2/imgproc/imgproc.hpp>

#include <Eigen/Geometry>

#include <json.hpp>

namespace pilotguru {
namespace {

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

} // namespace

bool TrackImageSequence(ORB_SLAM2::System *SLAM,
                        ImageSequenceSource &image_source,
                        const std::string &trajectory_out_file,
                        ImageSequenceSink *tracked_frames_sink,
                        int rotation_smooth_sigma) {
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

  std::vector<ORB_SLAM2::PoseWithTimestamp> trajectory = SLAM->GetTrajectory();
  if (trajectory.empty()) {
    return false;
  }

  if (rotation_smooth_sigma > 0) {
    SmoothHeadingDirections(&trajectory, rotation_smooth_sigma);
  }

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
      Projected2DDirectionsToTurnAngles(*projected_directions);

  nlohmann::json trajectory_json;
  SetPlane(&trajectory_json, pca_plane);
  const int frame_id_offset =
      tracked_frames_sink == nullptr ? 0 : first_tracked_frame_id;
  SetTrajectory(&trajectory_json, trajectory, projected_directions.get(),
                &turn_angles, frame_id_offset);

  std::ofstream trajectory_ostream(trajectory_out_file);
  trajectory_ostream << trajectory_json.dump(2) << std::endl;

  return true;
}

} // namespace pilotguru
