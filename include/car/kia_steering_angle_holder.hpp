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
  static constexpr double angle_amplitude_hard_limit = 180.0;
  static constexpr int16_t torque_hard_limit = 80;

  // Maximum absolute angle amplitude for which the controller will attempt to
  // compute nonzero spoof steering torque. For absolute steering angles
  // exceeding this magnitude, regardless of the requested target steering
  // angle, zero spoof torque voltage will be used to avoid accidentall damaging
  // the hardware.
  double max_angle_amplitude = 180.0;
  // Maximum absolute spoof torque voltage, in controller-internal units.
  int16_t max_torque = 80;

  // Maximum absolute difference between the actual steering angle and the
  // requested angle. Within this tolerance, it is acceptable to have zero
  // angular velocity.
  double target_angle_accuracy_tolerance_degrees = 1.0;
  // Minimum magnitude of actual vs requested angle difference for which the
  // lower bound on the steering angular velocity reaches the full
  // min_steering_rotation_degrees_per_second value (see below).
  // The lower bound on the angular velocity is
  // piecewise-linear:
  //        _______
  // ______/
  //
  // with values
  // * min_steering_rotation_degrees_per_second for angle residual larger than
  //    target_angle_diff_full_angular_velocity_lower_bound
  // * max( -max_steering_rotation_degrees_per_second,
  //         the line between points
  //             (target_angle_accuracy_tolerance_degrees, 0) and
  //             (target_angle_diff_full_angular_velocity_lower_bound,
  //                   min_steering_rotation_degrees_per_second)).
  double target_angle_diff_full_angular_velocity_lower_bound = 10.0;

  // Hard cap on the minimum absolute linear velocity more than
  // target_angle_diff_full_angular_velocity_lower_bound away from the target
  // angle.
  double min_steering_rotation_degrees_per_second = 90.0;
  // Hard cap on the maximum absolute angular velocity.
  double max_steering_rotation_degrees_per_second = 270.0;

  // Kalman filter settings.
  double kalman_filter_observation_variance = 2.0;
  double kalman_filter_perturbation_variance_per_second = 1e8;

  // How far ahead in time to try to predict the angle and angular velocity to
  // compute the spoof steering torque on the next step.
  double lookahead_estimate_time_sec = 0.2;
  // Factor by which to scale angular acceleration in the lookahead prediction,
  // to be able to reduce the influence of relatively poor acceleration
  // estimates on the controller decisions.
  double lookahead_acceleration_scale = 0.8;

  // Step size by which to change the internal real-valued spoof torque voltage
  // level on a single iteration (if necessary). The actual spoof voltage level
  // sent to a controller will rounded to a whole integer, but having a
  // fractional change step for internal representation makes it possible to
  // change the spoof torque slower *on average* than unit whole unit per
  // control loop iteration.
  double torque_change_step = 0.2;

  bool IsValid() const;
};

double BoundedRotationVelocityEffectiveTorque(
    double torque_voltage_finegrained, double target_angle_degrees,
    double measured_angle_degrees, double angular_velocity_degrees_per_second,
    const SteeringAngleHolderSettings &settings);

class SteeringAngleHolderController {
public:
  SteeringAngleHolderController(
      const TimestampedHistory<SteeringAngle> *steering_angle_sensor,
      ArduinoCommandChannel *arduino_command_channel,
      const SteeringAngleHolderSettings &settings);

  const SteeringAngleHolderSettings &settings() const;
  bool SetTargetAngle(double target_angle_degrees);
  bool IsTargetAngleSet() const;
  double GetTargetAngle();
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
  std::unique_ptr<KalmanFilter1D2Order> angle_sensor_filter_;

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
