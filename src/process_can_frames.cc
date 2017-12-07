#include <cstdlib>

#include <gflags/gflags.h>
#include <glog/logging.h>

#include <json.hpp>

#include <car/can.hpp>
#include <car/kia_can.hpp>
#include <io/json_converters.hpp>

// Inputs.

DEFINE_string(can_frames_json, "", "");

// Outputs.

DEFINE_string(steering_out_json, "", "");
DEFINE_string(velocities_out_json, "", "");

// Config.

// TODO velocities scaling factor.

int main(int argc, char **argv) {
  google::InitGoogleLogging(argv[0]);
  google::ParseCommandLineFlags(&argc, &argv, true);
  google::InstallFailureSignalHandler();

  std::unique_ptr<nlohmann::json> can_json_root =
      pilotguru::ReadJsonFile(FLAGS_can_frames_json);
  const nlohmann::json &can_json = (*can_json_root)[pilotguru::kCanFrames];

  std::unique_ptr<nlohmann::json> steering_out_json_root(new nlohmann::json());
  nlohmann::json &steering_out_json =
      (*steering_out_json_root)[pilotguru::kSteering];

  std::unique_ptr<nlohmann::json> velocities_out_json_root(
      new nlohmann::json());
  nlohmann::json &velocities_out_json =
      (*steering_out_json_root)[pilotguru::kVelocities];

  for (const nlohmann::json &timestamped_can_frame : can_json) {
    nlohmann::json out_element;
    out_element[pilotguru::kTimeUsec] =
        timestamped_can_frame[pilotguru::kTimeUsec];
    const string &can_frame_string =
        timestamped_can_frame[pilotguru::kCanFrame];
    const can_frame frame = pilotguru::parse_can_frame_or_die(can_frame_string);
    switch (frame.can_id) {
    case pilotguru::kia::STEERING_WHEEL_ANGLE_CAN_ID: {
      std::unique_ptr<pilotguru::kia::SteeringAngle> angle =
          pilotguru::kia::ParseSteeringAngle(frame);
      CHECK(angle != nullptr);
      out_element[pilotguru::kSteeringAngleDegrees] = angle->degrees();
      steering_out_json.push_back(out_element);
      break;
    }
    case pilotguru::kia::VELOCITY_CAN_ID: {
      std::unique_ptr<pilotguru::kia::Velocity> velocity =
          pilotguru::kia::ParseVelocity(frame);
      CHECK(velocity != nullptr);
      // TODO scaling.
      out_element[pilotguru::kSpeedMS] = velocity->average_wheel_speed();
      velocities_out_json.push_back(out_element);
      break;
    }
    default:
      continue;
    }
  }

  pilotguru::WriteJsonFile(steering_out_json, FLAGS_steering_out_json);
  pilotguru::WriteJsonFile(velocities_out_json, FLAGS_velocities_out_json);

  return EXIT_SUCCESS;
}
