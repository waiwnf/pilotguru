#ifndef PILOTGURU_CAR_KIA_STEERING_ANGLE_HOLDER_HPP_
#define PILOTGURU_CAR_KIA_STEERING_ANGLE_HOLDER_HPP_

#include <memory>
#include <mutex>
#include <thread>

#include <car/arduino_comm.hpp>
#include <car/kia_can.hpp>

namespace pilotguru {
namespace kia {

struct SteeringAngleHolderSettings {
  static constexpr int16_t angle_amplitude_hard_limit = 180;
  static constexpr int16_t torque_hard_limit = 10;

  int16_t max_angle_amplitude = 180;
  int16_t max_torque = 5;
  int16_t target_angle_accuracy_tolerance_degrees = 2;
  int16_t target_angle_magnitude_max_torque = 10;

  bool IsValid() const;
};

int16_t
SteeringAngleHolderEffectiveTorque(int16_t target_angle_degrees,
                                   int16_t measured_angle_degrees,
                                   const SteeringAngleHolderSettings &settings);

class SteeringAngleHolderController {
public:
  SteeringAngleHolderController(
      const TimestampedHistory<SteeringAngle> *steering_angle_sensor,
      ArduinoCommandChannel *arduino_command_channel,
      const SteeringAngleHolderSettings& settings);

  bool SetTargetAngle(int16_t target_angle_degrees);
  void ClearTargetAngle();
  void ControllerLoop();
  void RequestStop();

private:
  // Initialized by the constructor.
  const TimestampedHistory<SteeringAngle> * const steering_angle_sensor_;
  ArduinoCommandChannel * const arduino_command_channel_;
  const SteeringAngleHolderSettings settings_;

  // Adjusted at runtime.
  int16_t target_angle_degrees_ = 0;
  bool is_target_angle_set_ = false;
  bool must_run_ = true;

  std::mutex mutex_;

  std::unique_ptr<std::thread> controller_loop_thread_;
};
}
}

#endif // PILOTGURU_CAR_KIA_STEERING_ANGLE_HOLDER_HPP_
