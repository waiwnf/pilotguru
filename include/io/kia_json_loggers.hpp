#ifndef IO_KIA_JSON_LOGGERS_HPP_
#define IO_KIA_JSON_LOGGERS_HPP_

#include <car/kia_can.hpp>
#include <car/kia_steering_angle_holder.hpp>
#include <io/timestamped_json_logger.hpp>
#include <spoof-steering-serial-commands.h>

namespace pilotguru {

constexpr char STEERING_COMMANDS_LOG_ROOT_ELEMENT[] = "steering_commands";
constexpr char STEERING_ANGLES_LOG_ROOT_ELEMENT[] = "steering_angles";
constexpr char TARGET_STEERING_ANGLES_LOG_ROOT_ELEMENT[] =
    "target_steering_angles";

class SteeringCommandsJsonWriter
    : public JsonSteamWriter<kia::KiaControlCommand> {
public:
  void WriteAsJsonString(const kia::KiaControlCommand &command,
                         std::ostream &file_stream) override;
};

class SteeringAngleJsonWriter : public JsonSteamWriter<kia::SteeringAngle> {
public:
  void WriteAsJsonString(const kia::SteeringAngle &data,
                         std::ostream &file_stream) override;
};

class TargetSteeringAngleStatusJsonWriter
    : public JsonSteamWriter<kia::TargetSteeringAngleStatus> {
public:
  void WriteAsJsonString(const kia::TargetSteeringAngleStatus &data,
                         std::ostream &file_stream) override;
};

} // namespace pilotguru

#endif // IO_KIA_JSON_LOGGERS_HPP_
