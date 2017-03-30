#include <cstdlib>
#include <fstream>

#include <gflags/gflags.h>
#include <glog/logging.h>

#include <io/json_converters.hpp>
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
  pilotguru::SmoothHeadingDirections(&trajectory, FLAGS_sigma);
  // TODO copy projection plane.
  // TODO Reproject and recompute steering angles.
  nlohmann::json smoothed_trajectory_json;
  pilotguru::SetTrajectory(&smoothed_trajectory_json, trajectory, nullptr, nullptr, 0);
  std::ofstream trajectory_ostream(FLAGS_trajectory_out_file);
  trajectory_ostream << smoothed_trajectory_json.dump(2) << std::endl;

  return EXIT_SUCCESS;
}