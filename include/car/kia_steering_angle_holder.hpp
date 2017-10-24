#ifndef PILOTGURU_CAR_KIA_STEERING_ANGLE_HOLDER_HPP_
#define PILOTGURU_CAR_KIA_STEERING_ANGLE_HOLDER_HPP_

#include <memory>
#include <mutex>
#include <thread>

#include <car/arduino_comm.hpp>
#include <car/kalman_filter.hpp>
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
  double kalman_filter_observation_variance = 2.0;
  double kalman_filter_perturbation_variance_per_second = 1e6;

  bool IsValid() const;
};

int16_t
SteeringAngleHolderEffectiveTorque(double target_angle_degrees,
                                   double measured_angle_degrees,
                                   const SteeringAngleHolderSettings &settings);

class SteeringAngleHolderController {
public:
  SteeringAngleHolderController(
      const TimestampedHistory<SteeringAngle> *steering_angle_sensor,
      ArduinoCommandChannel *arduino_command_channel,
      const SteeringAngleHolderSettings &settings);

  const SteeringAngleHolderSettings &settings() const;
  bool SetTargetAngle(double target_angle_degrees);
  void ClearTargetAngle();
  void ControllerLoop();
  void RequestStop(); // non-blocking
  void Join();
  void Stop(); // blocking

private:
  // Initialized by the constructor.
  const TimestampedHistory<SteeringAngle> *const steering_angle_sensor_;
  ArduinoCommandChannel *const arduino_command_channel_;
  const SteeringAngleHolderSettings settings_;
  std::unique_ptr<KalmanFilter1D> angle_sensor_filter_;

  // Adjusted at runtime.
  double target_angle_degrees_ = 0;
  bool is_target_angle_set_ = false;
  bool must_run_ = true;

  std::mutex mutex_;

  std::unique_ptr<std::thread> controller_loop_thread_;
};
}
}

#endif // PILOTGURU_CAR_KIA_STEERING_ANGLE_HOLDER_HPP_
