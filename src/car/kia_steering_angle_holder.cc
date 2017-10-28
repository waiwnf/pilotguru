#include <car/kia_steering_angle_holder.hpp>

#include <sys/time.h>

#include <glog/logging.h>

namespace pilotguru {
namespace kia {

bool SteeringAngleHolderSettings::IsValid() const {
  if (max_angle_amplitude < 0 ||
      max_angle_amplitude > angle_amplitude_hard_limit) {
    return false;
  }
  if (max_torque < 0 || max_torque > torque_hard_limit) {
    return false;
  }
  if (target_angle_accuracy_tolerance_degrees <= 0) {
    return false;
  }
  if (target_angle_diff_full_angular_velocity_lower_bound < 0) {
    return false;
  }
  if (target_angle_accuracy_tolerance_degrees >
      target_angle_diff_full_angular_velocity_lower_bound) {
    return false;
  }
  if (min_steering_rotation_degrees_per_second < 0 ||
      max_steering_rotation_degrees_per_second < 0) {
    return false;
  }
  if (min_steering_rotation_degrees_per_second >=
      max_steering_rotation_degrees_per_second) {
    return false;
  }
  if (kalman_filter_observation_variance <= 0 ||
      kalman_filter_perturbation_variance_per_second <= 0) {
    return false;
  }
  if (lookahead_estimate_time_sec < 0) {
    return false;
  }
  if (lookahead_acceleration_scale < 0) {
    return false;
  }
  if (torque_change_step <= 0) {
    return false;
  }

  // All checks passed.
  return true;
}

namespace {
// Checks whether the current angular velocity of the steering angle is above
// the upper limit, which is piecewise linear depending on the residual from the
// target steering angle to the actual currently measured angle.
//
// Upper bound is stepwise:
// For residual in (-infty,
// -target_angle_diff_full_angular_velocity_lower_bound) -->
//    (-min_steering_rotation_degrees_per_second).
// From (-target_angle_diff_full_angular_velocity_lower_bound) and up --> linear
//    function intercepting (angular_velocity = 0) at angle
//    (-target_angle_accuracy_tolerance_degrees), so that within
//    target_angle_accuracy_tolerance_degrees of zero residual it is acceptable
//    to have zero angular velocity.
// Additionally, there is a hard cap at
// max_steering_rotation_degrees_per_second.
bool IsAngularVelocityAboveUpperBound(
    double angular_velocity, double target_angle_residual,
    const SteeringAngleHolderSettings &settings) {
  if (target_angle_residual <=
      -settings.target_angle_diff_full_angular_velocity_lower_bound) {
    return angular_velocity >
           -settings.min_steering_rotation_degrees_per_second;
  } else if (angular_velocity >
             settings.max_steering_rotation_degrees_per_second) {
    return true;
  } else {
    // The linear function goes from (-min_steering_rotation_degrees_per_second)
    // to 0 in the interval
    // (-target_angle_diff_full_angular_velocity_lower_bound -->
    //    -target_angle_accuracy_tolerance_degrees).
    // Compute the slope of this line.
    const double linear_tangent =
        settings.min_steering_rotation_degrees_per_second /
        (settings.target_angle_diff_full_angular_velocity_lower_bound -
         settings.target_angle_accuracy_tolerance_degrees);
    return angular_velocity >
           linear_tangent *
                   (target_angle_residual +
                    settings
                        .target_angle_diff_full_angular_velocity_lower_bound) -
               settings.min_steering_rotation_degrees_per_second;
  }
}
}

double BoundedRotationVelocityEffectiveTorque(
    double torque_voltage_finegrained, double target_angle_degrees,
    double measured_angle_degrees, double angular_velocity_degrees_per_second,
    const SteeringAngleHolderSettings &settings) {
  if (std::abs(measured_angle_degrees) > settings.max_angle_amplitude) {
    // Steering angle is out of bounds. Remove the steering torque completely to
    // avoid accidentally damaging the power steering drive.
    return 0;
  } else if (std::abs(torque_voltage_finegrained) > settings.max_torque + 1) {
    // Current input torque voltage exceeds the limits set in the settings. This
    // should not happen. Reset the spoof voltage to zero to avoid damage to the
    // hardware.
    return 0;
  } else {
    const double target_angle_residual =
        target_angle_degrees - measured_angle_degrees;
    double result = torque_voltage_finegrained;
    if (IsAngularVelocityAboveUpperBound(angular_velocity_degrees_per_second,
                                         target_angle_residual, settings)) {
      // Angular velocity is too large. Reduce the steering spoof torque.
      result -= settings.torque_change_step;
    } else if (IsAngularVelocityAboveUpperBound(
                   -angular_velocity_degrees_per_second, -target_angle_residual,
                   settings)) {
      // Symmetric velocty constraint in the opposite direction.
      // Angular velocity is too low. Increase the steering spoof torque.
      result += settings.torque_change_step;
    }
    // Cap the result to the limits from settings.
    return std::min(std::max(result, static_cast<double>(-settings.max_torque)),
                    static_cast<double>(settings.max_torque));
  }
}

SteeringAngleHolderController::SteeringAngleHolderController(
    const TimestampedHistory<SteeringAngle> *steering_angle_sensor,
    ArduinoCommandChannel *arduino_command_channel,
    const SteeringAngleHolderSettings &settings)
    : steering_angle_sensor_(steering_angle_sensor),
      arduino_command_channel_(arduino_command_channel), settings_(settings) {
  CHECK_NOTNULL(steering_angle_sensor_);
  CHECK_NOTNULL(arduino_command_channel_);
  CHECK(settings_.IsValid());

  angle_sensor_filter_.reset(new KalmanFilter1D2Order(
      settings_.kalman_filter_observation_variance,
      settings_.kalman_filter_perturbation_variance_per_second));

  controller_loop_thread_.reset(
      new std::thread(&SteeringAngleHolderController::ControllerLoop, this));
}

const SteeringAngleHolderSettings &
SteeringAngleHolderController::settings() const {
  return settings_;
}

bool SteeringAngleHolderController::SetTargetAngle(
    double target_angle_degrees) {
  if (std::abs(target_angle_degrees) > settings_.max_angle_amplitude) {
    return false;
  }

  std::unique_lock<std::mutex> lock(mutex_);
  is_target_angle_set_ = true;
  target_angle_degrees_ = target_angle_degrees;
  return true;
}

void SteeringAngleHolderController::ClearTargetAngle() {
  std::unique_lock<std::mutex> lock(mutex_);
  is_target_angle_set_ = false;
}

void SteeringAngleHolderController::ControllerLoop() {
  Timestamped<SteeringAngle> steering_instance = {{}, {0, 0}};
  KiaControlCommand steering_command = {KiaControlCommand::STEER, 0};
  bool is_first_measurement = true;
  // Potentially fractional spoof torque voltage to preserve the finer grained
  // change information. On each step the steering command uses this value
  // rounded to an integer.
  double torque_voltage_finegrained = 0;

  LoopWaitEffectiveTimeout loop_timeout({0 /* seconds */, 200000 /* usec */});
  while (must_run_) {
    // Read the raw angle sensor value or get a timeout indication.
    timeval wait_timeout = loop_timeout.GetRemainingTimeout();
    const bool steering_wait_result = steering_angle_sensor_->wait_get_next(
        steering_instance.timestamp(), &wait_timeout, &steering_instance);
    loop_timeout.WaitFinished();

    if (steering_wait_result &&
        steering_instance.data().angle_deci_degrees !=
            kia::STEERING_WHEEL_ANGLE_INVALID_VALUE) {
      // Steering angle read successful. Pass the result through the Kalman
      // filter and read off the updated angle and angular velocity estimates.
      const double raw_steering_angle_degrees =
          static_cast<double>(steering_instance.data().angle_deci_degrees) /
          10.0;
      angle_sensor_filter_->Update(
          {raw_steering_angle_degrees, steering_instance.timestamp()});
      const KalmanFilter1D2Order::EstimateValue &kalman_estimate =
          angle_sensor_filter_->LatestEstimate().data();

      // If this is the first angle measurement, skip the rest of the
      // computations as the angular velocity estimate is not reliable (very
      // dependent on the Kalman filter initialization), and we cannot
      // meaningfully estimate the acceleration either.
      if (is_first_measurement) {
        is_first_measurement = false;
        continue;
      }

      const double effective_steering_angle_degrees = kalman_estimate.mean(0);
      const double effective_angular_velocity_degrees_per_second =
          kalman_estimate.mean(1);
      const double effective_angular_acceleration = kalman_estimate.mean(2);

      // Compute the lookahead angle and angular velocity based on current
      // estimates.
      const double scaled_acceleration =
          settings_.lookahead_acceleration_scale *
          effective_angular_acceleration;
      const double lookahead_velocity =
          effective_angular_velocity_degrees_per_second +
          scaled_acceleration * settings_.lookahead_estimate_time_sec;
      const double lookahead_angle =
          effective_steering_angle_degrees +
          effective_angular_velocity_degrees_per_second *
              settings_.lookahead_estimate_time_sec +
          scaled_acceleration * settings_.lookahead_estimate_time_sec *
              settings_.lookahead_estimate_time_sec * 0.5;

      std::unique_lock<std::mutex> lock(mutex_);
      if (is_target_angle_set_) {
        // Absolute proposed torque voltage for this iteration.
        torque_voltage_finegrained = BoundedRotationVelocityEffectiveTorque(
            torque_voltage_finegrained, target_angle_degrees_, lookahead_angle,
            lookahead_velocity, settings_);
      }
    } else {
      // Steering angle sensor timed out. The car is not powered up or CAN bus
      // conection is not working. Clear the target steering angle and restore
      // the spoof stering torque to 0.
      ClearTargetAngle();
      torque_voltage_finegrained = 0;

      // Reset all the estimators.
      angle_sensor_filter_.reset(new KalmanFilter1D2Order(
          settings_.kalman_filter_observation_variance,
          settings_.kalman_filter_perturbation_variance_per_second));
      is_first_measurement = true;
    }
    steering_command.value = torque_voltage_finegrained; // Rounds to int.
    arduino_command_channel_->SendCommand(steering_command);
  }
}

void SteeringAngleHolderController::RequestStop() { must_run_ = false; }

void SteeringAngleHolderController::Join() { controller_loop_thread_->join(); }

void SteeringAngleHolderController::Stop() {
  RequestStop();
  Join();
}
}
}
