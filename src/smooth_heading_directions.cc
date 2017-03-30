#include <cstdlib>
#include <fstream>

#include <gflags/gflags.h>
#include <glog/logging.h>

#include <io/json_converters.hpp>
#include <slam/horizontal_flatten.hpp>
#include <slam/smoothing.hpp>

DEFINE_string(trajectory_in_file, "", "");
DEFINE_int64(sigma, -1, "");
DEFINE_string(trajectory_out_file, "", "");

int main(int argc, char **argv) {
  google::InitGoogleLogging(argv[0]);
  google::ParseCommandLineFlags(&argc, &argv, true);
  google::InstallFailureSignalHandler();

  CHECK(!FLAGS_trajectory_in_file.empty());
  CHECK(!FLAGS_trajectory_out_file.empty());
  CHECK_GT(FLAGS_sigma, 0);

  std::ifstream trajectory_istream(FLAGS_trajectory_in_file);
  nlohmann::json trajectory_json;
  trajectory_istream >> trajectory_json;

  std::vector<ORB_SLAM2::PoseWithTimestamp> trajectory;
  pilotguru::ReadTrajectory(trajectory_json, &trajectory, nullptr, nullptr);
  const cv::Mat horizontal_plane = pilotguru::ReadPlane(trajectory_json);

  pilotguru::SmoothHeadingDirections(&trajectory, FLAGS_sigma);
  std::unique_ptr<std::vector<cv::Mat>> projected_directions =
      pilotguru::ProjectDirections(trajectory, horizontal_plane);
  const vector<double> turn_angles =
      pilotguru::Projected2DDirectionsToTurnAngles(*projected_directions);

  // TODO Reproject and recompute steering angles.
  nlohmann::json smoothed_trajectory_json;
  pilotguru::SetTrajectory(&smoothed_trajectory_json, trajectory,
                           projected_directions.get(), &turn_angles, 0);
  pilotguru::SetPlane(&smoothed_trajectory_json, horizontal_plane);
  std::ofstream trajectory_ostream(FLAGS_trajectory_out_file);
  trajectory_ostream << smoothed_trajectory_json.dump(2) << std::endl;

  return EXIT_SUCCESS;
}