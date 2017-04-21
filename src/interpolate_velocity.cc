#include <cstdlib>

#include <gflags/gflags.h>
#include <glog/logging.h>

#include <json.hpp>

#include <io/json_converters.hpp>
#include <slam/smoothing.hpp>

DEFINE_string(locations_json, "", "GPS locations and velocities log.");
DEFINE_string(frames_json, "", "Video frames timestamps log.");
DEFINE_double(sigma, 0.5,
              "Gaussian smoothing kernel standard deviation, in seconds.");
DEFINE_string(out_json, "",
              "Copy of frames data with added interpolated velocities.");

int main(int argc, char **argv) {
  google::InitGoogleLogging(argv[0]);
  google::ParseCommandLineFlags(&argc, &argv, true);
  google::InstallFailureSignalHandler();

  CHECK(!FLAGS_locations_json.empty());
  CHECK(!FLAGS_frames_json.empty());
  CHECK(!FLAGS_out_json.empty());

  // Read raw GPS data.
  std::unique_ptr<nlohmann::json> locations_json =
      pilotguru::ReadJsonFile(FLAGS_locations_json);
  const auto &locations = (*locations_json)[pilotguru::kLocations];
  CHECK(!locations.empty());

  // Read raw frame timestamps data.
  std::unique_ptr<nlohmann::json> frames_json =
      pilotguru::ReadJsonFile(FLAGS_frames_json);
  const auto &frames = (*frames_json)[pilotguru::kFrames];
  CHECK(!frames.empty());

  // To reduce precision loss when converting to double timestamps, subtract the
  // earliest timestamp of the two time series from all the timestamps.
  const long locations_start_usec = locations.at(0)[pilotguru::kTimeUsec];
  const long frames_start_usec = frames.at(0)[pilotguru::kTimeUsec];
  const long start_usec = std::min(locations_start_usec, frames_start_usec);

  // GPS data to vectors.
  std::vector<double> gps_velocities, gps_timestamps;
  for (const nlohmann::json &location : locations) {
    gps_velocities.push_back(location[pilotguru::kSpeedMS]);
    const long time_usec = location[pilotguru::kTimeUsec];
    gps_timestamps.push_back(static_cast<double>(time_usec - start_usec) *
                             1e-6);
  }

  // Frame timestamps to vector.
  std::vector<double> frame_timestamps;
  for (const nlohmann::json &frame : frames) {
    const long time_usec = frame[pilotguru::kTimeUsec];
    frame_timestamps.push_back(static_cast<double>(time_usec - start_usec) *
                               1e-6);
  }

  // Smooth/interpolate.
  const std::vector<double> frame_velocities = pilotguru::SmoothTimeSeries(
      gps_velocities, gps_timestamps, frame_timestamps, FLAGS_sigma);

  nlohmann::json result_json;
  result_json[pilotguru::kFrames] = {};
  for (size_t frame_idx = 0; frame_idx < frames.size(); ++frame_idx) {
    nlohmann::json frame_data = frames.at(frame_idx);
    frame_data[pilotguru::kSpeedMS] = frame_velocities.at(frame_idx);
    result_json[pilotguru::kFrames].push_back(frame_data);
  }
  std::ofstream result_ostream(FLAGS_out_json);
  result_ostream << result_json.dump(2) << std::endl;

  return EXIT_SUCCESS;
}
