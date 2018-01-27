#ifndef IO_KIA_JSON_LOGGERS_HPP_
#define IO_KIA_JSON_LOGGERS_HPP_

#include <car/kia_can.hpp>
#include <io/timestamped_json_logger.hpp>
#include <spoof-steering-serial-commands.h>

namespace pilotguru {

constexpr char STEERING_COMMANDS_LOG_ROOT_ELEMENT[] = "steering_commands";
constexpr char STEERING_ANGLES_LOG_ROOT_ELEMENT[] = "steering_angles";

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

} // namespace pilotguru

#endif // IO_KIA_JSON_LOGGERS_HPP_
