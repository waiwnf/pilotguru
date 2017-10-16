#include <car/kia_steering_angle_holder.hpp>

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
  if (target_angle_accuracy_tolerance_degrees < 0) {
    return false;
  }
  if (target_angle_magnitude_max_torque < 0) {
    return false;
  }
  if (target_angle_accuracy_tolerance_degrees >
      target_angle_magnitude_max_torque) {
    return false;
  }
  // All checks passed.
  return true;
}

int16_t SteeringAngleHolderEffectiveTorque(
    int16_t target_angle_degrees, int16_t measured_angle_degrees,
    const SteeringAngleHolderSettings &settings) {
  if (std::abs(measured_angle_degrees) > settings.max_angle_amplitude) {
    // Steering angle is out of bounds. Remove the steering torque completely to
    // avoid accidentally damaging the power steering drive.
    return 0;
  } else {
    // Upcast to full ints to avoid over/underflow in linear interpolation
    // below.
    const int target_angle_diff = target_angle_degrees - measured_angle_degrees;
    const int abs_target_angle_diff = std::abs(target_angle_diff);
    const int sign_target_angle_diff = target_angle_diff >= 0 ? 1 : -1;
    // Steering torque is stepwise:
    // * 0 within target_angle_accuracy_tolerance_degrees of the target angle.
    // * linear between target_angle_accuracy_tolerance_degrees and
    //   target_angle_magnitude_max_torque away from the target.
    // * maximum available value more than target_angle_magnitude_max_torque
    //   away from the target.
    if (abs_target_angle_diff <=
        settings.target_angle_accuracy_tolerance_degrees) {
      return 0;
    } else if (abs_target_angle_diff >=
               settings.target_angle_magnitude_max_torque) {
      return sign_target_angle_diff * settings.max_torque;
    } else {
      const int linear_normalization =
          settings.target_angle_magnitude_max_torque -
          settings.target_angle_accuracy_tolerance_degrees;
      return ((abs_target_angle_diff -
               settings.target_angle_accuracy_tolerance_degrees) *
              sign_target_angle_diff * settings.max_torque) /
             linear_normalization;
    }
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

  controller_loop_thread_.reset(
      new std::thread(&SteeringAngleHolderController::ControllerLoop, this));
}

bool SteeringAngleHolderController::SetTargetAngle(
    int16_t target_angle_degrees) {
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
  while (must_run_) {
    // TODO wait timeout;
    // TODO subtract time spent in the rest of the iteration from the timeout.
    if (steering_angle_sensor_->wait_get_next(steering_instance.timestamp(),
                                              nullptr, &steering_instance)) {
      std::unique_lock<std::mutex> lock(mutex_);
      if (is_target_angle_set_) {
        steering_command.value = SteeringAngleHolderEffectiveTorque(
            target_angle_degrees_,
            steering_instance.data().angle_deci_degrees / 10, settings_);
        // TODO: also post commands to out queue to be picked up by the UI.
        arduino_command_channel_->SendCommand(steering_command);
      }
    } else {
      // Steering angle sensor timed out. This should not happen. Clear the
      // target steering angle and restore the spoof stering torque to 0.
      ClearTargetAngle();
      steering_command.value = 0;
      arduino_command_channel_->SendCommand(steering_command);
    }
  }
}

void SteeringAngleHolderController::RequestStop() { must_run_ = false; }
}
}
