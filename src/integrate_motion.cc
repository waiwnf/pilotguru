#include <cmath>
#include <cstdlib>

#include <Eigen/Geometry>

#include <gflags/gflags.h>
#include <glog/logging.h>

#include <json.hpp>

#include <interpolation/align_time_series.hpp>
#include <io/json_converters.hpp>
#include <slam/smoothing.hpp>

DEFINE_string(rotations_json, "", "Rotations.");
DEFINE_string(accelerations_json, "", "Accelerations.");
DEFINE_string(out_json, "", "Resulting integrated headings");

namespace {
vector<long> GetTimestamps(const nlohmann::json &events) {
  vector<long> result;
  for (const auto &event : events) {
    result.push_back(event[pilotguru::kTimeUsec]);
  }
  return result;
}
}

int main(int argc, char **argv) {
  google::InitGoogleLogging(argv[0]);
  google::ParseCommandLineFlags(&argc, &argv, true);
  google::InstallFailureSignalHandler();

  CHECK(!FLAGS_rotations_json.empty());
  CHECK(!FLAGS_accelerations_json.empty());
  CHECK(!FLAGS_out_json.empty());

  // Read raw gyroscope data.
  std::unique_ptr<nlohmann::json> rotations_json =
      pilotguru::ReadJsonFile(FLAGS_rotations_json);
  const auto &rotations = (*rotations_json)[pilotguru::kRotations];
  CHECK(!rotations.empty());

  // Read raw accelerometer data.
  std::unique_ptr<nlohmann::json> accelerations_json =
      pilotguru::ReadJsonFile(FLAGS_accelerations_json);
  const auto &accelerations = (*accelerations_json)["accelerations"];
  CHECK(!accelerations.empty());

  const std::vector<long> rotations_timestamps = GetTimestamps(rotations);
  const std::vector<long> accelerations_timestamps =
      GetTimestamps(accelerations);
  const std::vector<const std::vector<long> *> component_timestamps = {
      &rotations_timestamps, &accelerations_timestamps};

  const std::vector<std::vector<size_t>> sensor_events =
      pilotguru::MergeTimeSeries(component_timestamps);

  std::vector<Eigen::Quaterniond> integrated_rotations = {{1.0, 0.0, 0.0, 0.0}};
  std::vector<Eigen::Vector3d> integrated_velocities = {{0.0, 0.0, 0.0}};
  for (size_t i = 1; i < sensor_events.size(); ++i) {
    const std::vector<size_t> current = sensor_events.at(i);
    const std::vector<size_t> prev = sensor_events.at(i - 1);
    const long current_time_usec =
        pilotguru::GetEffectiveTimeStamp(component_timestamps, current);
    const long prev_time_usec =
        pilotguru::GetEffectiveTimeStamp(component_timestamps, prev);
    const double interval_sec =
        static_cast<double>(current_time_usec - prev_time_usec) * 1e-6;

    // Integrate acceleration using the latest previous overall rotation.
    const auto &acceleration_local = accelerations.at(current.at(1));
    const Eigen::Vector3d acceleration_local_vec(
        acceleration_local[pilotguru::kX], acceleration_local[pilotguru::kY],
        acceleration_local[pilotguru::kZ]);
    const Eigen::Vector3d acceleration_rotated =
        integrated_rotations.back()._transformVector(acceleration_local_vec);
    const Eigen::Vector3d velocity =
        integrated_velocities.back() + interval_sec * acceleration_rotated;
    integrated_velocities.push_back(velocity);

    // Integrate rotation.
    const auto &rotation_rate = rotations.at(current.at(0));
    const double rate_x_rad_s = rotation_rate[pilotguru::kX];
    const double rate_y_rad_s = rotation_rate[pilotguru::kY];
    const double rate_z_rad_s = rotation_rate[pilotguru::kZ];
    const double rate_overall_rad_s =
        sqrt(rate_x_rad_s * rate_x_rad_s + rate_y_rad_s * rate_y_rad_s +
             rate_z_rad_s * rate_z_rad_s);
    if (rate_overall_rad_s > 1e-5) {
      const double half_theta = rate_overall_rad_s * interval_sec * 0.5;
      const double sin_half_theta_normalized =
          sin(half_theta) / rate_overall_rad_s;

      const double dw = cos(half_theta);
      const double dx = rate_x_rad_s * sin_half_theta_normalized;
      const double dy = rate_y_rad_s * sin_half_theta_normalized;
      const double dz = rate_z_rad_s * sin_half_theta_normalized;
      const Eigen::Quaterniond d_rotation(dw, dx, dy, dz);
      const Eigen::Quaterniond &overall_rotation = integrated_rotations.back();
      integrated_rotations.push_back(overall_rotation * d_rotation);
    }
  }

  LOG(INFO) << "Final rotation.  w: " << integrated_rotations.back().w()
            << "  x " << integrated_rotations.back().x() << "  y "
            << integrated_rotations.back().y() << "  z "
            << integrated_rotations.back().z();

  // Assume the velocity is actually zero in the beginning and end and debias.
  const Eigen::Vector3d delta_velocity =
      integrated_velocities.back() - integrated_velocities.front();
  const long start_usec = pilotguru::GetEffectiveTimeStamp(
      component_timestamps, sensor_events.front());
  const long end_usec = pilotguru::GetEffectiveTimeStamp(component_timestamps,
                                                         sensor_events.back());
  const double overall_time_sec =
      static_cast<double>(end_usec - start_usec) * 1e-6;
  const Eigen::Vector3d acceleration_bias = delta_velocity / overall_time_sec;

  LOG(INFO) << "Acceleration bias: " << acceleration_bias.x() << " "
            << acceleration_bias.y() << " " << acceleration_bias.z();

  // Correct the integrated velocities using the acceleration bias.
  for (size_t i = 1; i < sensor_events.size(); ++i) {
    const std::vector<size_t> current = sensor_events.at(i);
    const long current_time_usec =
        pilotguru::GetEffectiveTimeStamp(component_timestamps, current);
    const double current_from_start_sec =
        static_cast<double>(current_time_usec - start_usec) * 1e-6;
    integrated_velocities.at(i) -= acceleration_bias * current_from_start_sec;
  }

  // Write out resulting integrated velocities.
  nlohmann::json out_json;
  out_json[pilotguru::kFrames] = {};
  auto &out_frames = out_json[pilotguru::kFrames];
  for (size_t acceleration_idx = 0; acceleration_idx < accelerations.size();
       ++acceleration_idx) {
    nlohmann::json velocity_json;
    velocity_json[pilotguru::kTimeUsec] =
        accelerations.at(acceleration_idx)[pilotguru::kTimeUsec];
    velocity_json[pilotguru::kSpeedMS] =
        integrated_velocities.at(acceleration_idx).norm();
    out_frames.push_back(velocity_json);
  }

  std::ofstream result_ostream(FLAGS_out_json);
  result_ostream << out_json.dump(2) << std::endl;

  return EXIT_SUCCESS;
}
