#include <cstdlib>

#include <gflags/gflags.h>
#include <glog/logging.h>

#include <io/json_converters.hpp>
#include <slam/horizontal_flatten.hpp>

DEFINE_string(trajectory_in_file, "", "");
DEFINE_string(trajectory_out_file, "", "");

int main(int argc, char **argv) {
  google::InitGoogleLogging(argv[0]);
  google::ParseCommandLineFlags(&argc, &argv, true);
  google::InstallFailureSignalHandler();

  CHECK(!FLAGS_trajectory_in_file.empty());
  CHECK(!FLAGS_trajectory_out_file.empty());

  std::vector<ORB_SLAM2::PoseWithTimestamp> trajectory;
  cv::Mat horizontal_plane;
  std::vector<cv::Mat> projected_directions;
  vector<double> turn_angles;
  pilotguru::ReadTrajectoryFromFile(FLAGS_trajectory_in_file, &trajectory,
                                    &horizontal_plane, &projected_directions, &turn_angles);
  pilotguru::ProjectTranslations(&trajectory,horizontal_plane);
  pilotguru::WriteTrajectoryToFile(FLAGS_trajectory_out_file, trajectory,
                                   &horizontal_plane,
                                   &projected_directions, &turn_angles, 0);

  return EXIT_SUCCESS;
}