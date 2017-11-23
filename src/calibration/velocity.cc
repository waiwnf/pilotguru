#include <calibration/velocity.hpp>

#include <geometry/geometry.hpp>
#include <logging/strings.hpp>

#include <Eigen/Geometry>

#include <glog/logging.h>

namespace pilotguru {
namespace {
// Init the interpolation intervals for the merged accelerations+rotations time
// series wrt the reference time series of GPS measurements.
std::vector<std::vector<InterpolationInterval>> InitInterpolationIntervals(
    const std::vector<TimestampedVelocity> &reference_velocities,
    const MergedTimeSeries &imu_events) {
  std::vector<long> imu_times;
  for (size_t imu_index = 0; imu_index < imu_events.MergedEvents().size();
       ++imu_index) {
    imu_times.push_back(imu_events.MergedEventTimeUsec(imu_index));
  }

  const std::vector<long> reference_timestamps =
      ExtractTimestamps(reference_velocities);
  return MakeInterpolationIntervals(reference_timestamps, imu_times);
}
}

AccelerometerCalibrator::AccelerometerCalibrator(
    const std::vector<TimestampedVelocity> &reference_velocities,
    const std::vector<TimestampedRotationVelocity> &rotation_velocities,
    const std::vector<TimestampedAcceleration> &accelerations)
    : reference_velocities_(reference_velocities),
      rotation_velocities_(rotation_velocities), accelerations_(accelerations),
      rotation_times_(ExtractTimestamps(rotation_velocities)),
      accelerations_times_(ExtractTimestamps(accelerations)),
      imu_times_({&rotation_times_, &accelerations_times_}),
      reference_intervals_(
          InitInterpolationIntervals(reference_velocities, imu_times_)) {}

double AccelerometerCalibrator::eval(const std::vector<double> &in,
                                     std::vector<double> *gradient) {
  CHECK_NOTNULL(gradient);
  // Global acceleration bias (gravity) +
  // local acceleration bias (accelerometer calibration) +
  // initial velocity.
  CHECK_EQ(in.size(), 9);
  CHECK_EQ(in.size(), gradient->size());

  for (size_t i = 0; i < gradient->size(); ++i) {
    gradient->at(i) = 0.0;
  }

  // Named aliases for biases.
  const Eigen::Vector3d acceleration_global_bias(in.at(0), in.at(1), in.at(2));
  const Eigen::Vector3d acceleration_local_bias(in.at(3), in.at(4), in.at(5));
  const Eigen::Vector3d initial_velocity(in.at(6), in.at(7), in.at(8));

  double result = 0; // Sum of squared travel distance differences.
  // Without loss of generality on the first step take the device to be aligned
  // with the fixed reference frame.
  Eigen::Quaterniond integrated_rotation(1.0, 0.0, 0.0, 0.0);
  Eigen::Vector3d integrated_velocity = initial_velocity;
  Eigen::Matrix3d total_time_weighted_rotation = Eigen::Matrix3d::Zero();
  long total_time_usec = 0;

  for (const std::vector<InterpolationInterval> &intervals :
       reference_intervals_) {
    // Total effective shift in 3D for this interval between two reference
    // points, integrated from accelerometer/gyro.
    Eigen::Vector3d integrated_travel = Eigen::Vector3d::Zero();

    // Travel distance from GPS, assuming fixed speed and straight line travel.
    double reference_distance = 0;

    // Iterate over all inertial measurements on the given GPS reference
    // interval.
    std::vector<MotionIntegrationOutcome> integrated_intervals;
    for (const InterpolationInterval &interval : intervals) {
      const std::vector<size_t> &sensor_indices =
          imu_times_.MergedEvents().at(interval.interpolation_end_time_index);
      const TimestampedRotationVelocity &timestamped_raw_rotation =
          rotation_velocities_.at(sensor_indices.at(0));
      const TimestampedAcceleration &raw_acceleration =
          accelerations_.at(sensor_indices.at(1));

      const Eigen::Quaterniond raw_rotation = RotationMotionToQuaternion(
          timestamped_raw_rotation.rate_x_rad_s,
          timestamped_raw_rotation.rate_y_rad_s,
          timestamped_raw_rotation.rate_z_rad_s, interval.DurationSec());
      const Eigen::Vector3d acceleration_local_raw =
          Eigen::Vector3d(raw_acceleration.acc_x, raw_acceleration.acc_y,
                          raw_acceleration.acc_z);

      const MotionIntegrationOutcome integration_outcome = IntegrateMotion(
          integrated_rotation, integrated_velocity, raw_rotation,
          acceleration_local_raw, acceleration_global_bias,
          acceleration_local_bias, interval.DurationUsec());
      integrated_intervals.push_back(integration_outcome);

      integrated_rotation = integration_outcome.orientation;
      integrated_velocity = integration_outcome.velocity;

      // Notice that we are summing up the shifts in 3D to get the effective
      // straight line shift in the end to match the GPS assumption of straight
      // line travel. This does not seem to matter much in practice though,
      // apparently the curves are not curved too much.
      //
      // TODO: This is not quite right - we should be taking the average of
      // before and after velocities here.
      integrated_travel +=
          interval.DurationSec() * integration_outcome.velocity;

      // Add up GPS derived travel distance assuming fixed velocity and straight
      // line travel.
      reference_distance +=
          interval.DurationSec() *
          reference_velocities_.at(interval.reference_end_time_index).velocity;
    }

    // Integrated shift computed, now we can get the loss function value.
    const double distance_diff = integrated_travel.norm() - reference_distance;
    result += distance_diff * distance_diff;

    // Loss derivative wrt the overall integrated travel vector.
    const Eigen::Vector3d d_loss_d_travel = 2.0 * distance_diff *
                                            integrated_travel /
                                            (integrated_travel.norm() + 1e-5);

    for (const MotionIntegrationOutcome &integration_outcome :
         integrated_intervals) {
      const double interval_sec =
          static_cast<double>(integration_outcome.duration_usec) * 1e-6;
      total_time_usec += integration_outcome.duration_usec;
      const double total_time_sec = static_cast<double>(total_time_usec) * 1e-6;

      // Derivative wrt the acceleration bias in the fixed reference frame.
      // Both integrated travel and bias are in the same reference frame
      // ==> no rotation
      // ==> derivative can be computed component-wise.
      gradient->at(0) += total_time_sec * interval_sec * d_loss_d_travel.x();
      gradient->at(1) += total_time_sec * interval_sec * d_loss_d_travel.y();
      gradient->at(2) += total_time_sec * interval_sec * d_loss_d_travel.z();

      // Derivative wrt the acceleration bias in the local reference frame.
      // The local bias is rotated by the cumulative rotation up to the current
      // time, so we need to use the rotation matrix to go from gradient in
      // fixed frame to gradient in local frame.
      const Eigen::Matrix3d d_rotation_matrix =
          integration_outcome.orientation.toRotationMatrix();
      // First time integration to get velocity.
      total_time_weighted_rotation += (d_rotation_matrix * interval_sec);
      // Second time integration to get distance.
      const Eigen::Vector3d dloss_dbias_local =
          interval_sec * total_time_weighted_rotation.transpose() *
          d_loss_d_travel;
      gradient->at(3) += dloss_dbias_local.x();
      gradient->at(4) += dloss_dbias_local.y();
      gradient->at(5) += dloss_dbias_local.z();

      // Gradient wrt initial speed.
      gradient->at(6) += interval_sec * d_loss_d_travel.x();
      gradient->at(7) += interval_sec * d_loss_d_travel.y();
      gradient->at(8) += interval_sec * d_loss_d_travel.z();
    }
  }

  // Normalize everything by the total time duration in seconds.
  const double total_time_sec = static_cast<double>(total_time_usec) * 1e-6;
  result /= total_time_sec;
  for (size_t i = 0; i < in.size(); ++i) {
    gradient->at(i) /= total_time_sec;
  }

  LOG(INFO) << "AccelerometerCalibrator input: " << in;
  LOG(INFO) << "AccelerometerCalibrator gradient: " << *gradient;
  LOG(INFO) << "AccelerometerCalibrator loss: " << result;

  return result;
}

double AccelerometerCalibrator::operator()(const Eigen::VectorXd &x,
                                           Eigen::VectorXd &grad) {
  std::vector<double> x_vec(x.size()), grad_vec(x.size(), 0.0);
  for (long i = 0; i < x.size(); ++i) {
    x_vec.at(i) = x[i];
  }
  const double result = eval(x_vec, &grad_vec);
  for (long i = 0; i < x.size(); ++i) {
    grad[i] = grad_vec.at(i);
  }
  return result;
}

const MergedTimeSeries &AccelerometerCalibrator::ImuTimes() const {
  return imu_times_;
}

const std::map<size_t, MotionIntegrationOutcome>
AccelerometerCalibrator::IntegrateTrajectory(
    const Eigen::Vector3d &acceleration_global_bias,
    const Eigen::Vector3d &acceleration_local_bias,
    const Eigen::Vector3d &initial_velocity) {
  std::map<size_t, MotionIntegrationOutcome> result;

  Eigen::Quaterniond integrated_rotation(1.0, 0.0, 0.0, 0.0);
  Eigen::Vector3d integrated_velocity = initial_velocity;

  for (const std::vector<InterpolationInterval> &intervals :
       reference_intervals_) {
    for (const InterpolationInterval &interval : intervals) {
      size_t interpolation_index = interval.interpolation_end_time_index;
      const std::vector<size_t> &sensor_indices =
          imu_times_.MergedEvents().at(interpolation_index);
      const TimestampedRotationVelocity &timestamped_raw_rotation =
          rotation_velocities_.at(sensor_indices.at(0));
      const TimestampedAcceleration &raw_acceleration =
          accelerations_.at(sensor_indices.at(1));

      const Eigen::Quaterniond raw_rotation = RotationMotionToQuaternion(
          timestamped_raw_rotation.rate_x_rad_s,
          timestamped_raw_rotation.rate_y_rad_s,
          timestamped_raw_rotation.rate_z_rad_s, interval.DurationSec());
      const Eigen::Vector3d acceleration_local_raw =
          Eigen::Vector3d(raw_acceleration.acc_x, raw_acceleration.acc_y,
                          raw_acceleration.acc_z);

      const MotionIntegrationOutcome integration_outcome = IntegrateMotion(
          integrated_rotation, integrated_velocity, raw_rotation,
          acceleration_local_raw, acceleration_global_bias,
          acceleration_local_bias, interval.DurationUsec());

      integrated_rotation = integration_outcome.orientation;
      integrated_velocity = integration_outcome.velocity;

      // See whether there is already a result for this interpolation index.
      // If yes, that value is only a partial result because the interpolation
      // interval has been broken up by the reference intervals boundary.
      auto existing_outcome = result.find(interpolation_index);
      if (existing_outcome == result.end()) {
        // No result for this interpolation index, just put in the newly
        // computed values.
        result.insert({interpolation_index, integration_outcome});
      } else {
        // Result for this interpolation index already exists. Update the
        // trajectory data, and sum up the interval durations.
        existing_outcome->second.orientation = integration_outcome.orientation;
        existing_outcome->second.velocity = integration_outcome.velocity;
        existing_outcome->second.duration_usec +=
            integration_outcome.duration_usec;
      }
    }
  }

  return result;
}

FixedForwardAxisCalibrator::FixedForwardAxisCalibrator(
    const std::vector<TimestampedVelocity> &reference_velocities,
    const std::vector<TimestampedRotationVelocity> &rotation_velocities,
    const std::vector<TimestampedAcceleration> &accelerations)
    : reference_velocities_(reference_velocities),
      rotation_velocities_(rotation_velocities), accelerations_(accelerations),
      rotation_times_(ExtractTimestamps(rotation_velocities)),
      accelerations_times_(ExtractTimestamps(accelerations)),
      imu_times_({&rotation_times_, &accelerations_times_}),
      reference_intervals_(
          InitInterpolationIntervals(reference_velocities, imu_times_)) {}

double FixedForwardAxisCalibrator::operator()(const Eigen::VectorXd &x,
                                              Eigen::VectorXd &grad) {
  // Global acceleration bias (gravity) +
  // local acceleration bias (accelerometer calibration) +
  // vehicle forward axis in moving reference frame +
  // scalar velocity for every IMU timestamp.
  CHECK_EQ(x.size(), VELOCITY_SCALES_START + imu_times_.MergedEvents().size());
  CHECK_EQ(x.size(), grad.size());

  // Zero out the gradient.
  grad *= 0;

  // Named aliases for biases.
  const Eigen::Vector3d acceleration_global_bias =
      x.segment<3>(ACCELERATION_GLOBAL_BIAS_START);
  const Eigen::Vector3d acceleration_local_bias =
      x.segment<3>(ACCELERATION_LOCAL_BIAS_START);
  const Eigen::Vector3d local_forward_axis = x.segment<3>(FORWARD_AXIS_START);
  const Eigen::VectorBlock<const Eigen::VectorXd> scalar_velocities =
      x.segment(VELOCITY_SCALES_START, imu_times_.MergedEvents().size());

  // Overall loss consists of 3 components:
  // - Sum of squared travel distance differences between GPS and IMU.
  // - Sum of squared diffs of IMU measured accelerations vs the forward-only
  //   motion constrained accelerations.
  // - Penalty on local forward axis magnitude different from 1.
  double travel_distance_loss = 0;
  double acceleration_match_loss = 0;

  const double forward_axis_length = local_forward_axis.norm();
  const double forward_axis_magnitude_weight = 5e-3;
  double forward_axis_magnitude_loss = forward_axis_magnitude_weight *
                                       (forward_axis_length - 1.0) *
                                       (forward_axis_length - 1.0);
  const Eigen::Vector3d forward_axis_magnitude_loss_gradient =
      forward_axis_magnitude_weight * local_forward_axis * 2.0 *
      (1.0 - 1.0 / (forward_axis_length + 1e-5));

  grad.segment<3>(FORWARD_AXIS_START) += forward_axis_magnitude_loss_gradient;

  // Without loss of generality on the first step take the device to be aligned
  // with the fixed reference frame.
  Eigen::Quaterniond integrated_rotation(1.0, 0.0, 0.0, 0.0);
  for (const std::vector<InterpolationInterval> &reference_interval :
       reference_intervals_) {
    // Total effective shift in 3D for this interval between two reference
    // points, integrated from accelerometer/gyro.
    Eigen::Vector3d integrated_travel = Eigen::Vector3d::Zero();
    // Travel distance from GPS, assuming fixed speed and straight line travel.
    double reference_distance = 0;

    std::vector<Eigen::Vector3d> d_integrated_travel_d_velocity_scales;
    Eigen::Matrix3d d_integrated_travel_d_forward_axis =
        Eigen::Matrix3d::Zero();
    for (const InterpolationInterval &imu_interval : reference_interval) {
      const size_t imu_interval_global_idx =
          imu_interval.interpolation_end_time_index;
      const std::vector<size_t> &sensor_indices =
          imu_times_.MergedEvents().at(imu_interval_global_idx);
      const TimestampedRotationVelocity &timestamped_raw_rotation =
          rotation_velocities_.at(sensor_indices.at(0));

      const double imu_duration_sec = imu_interval.DurationSec();
      const Eigen::Quaterniond raw_rotation = RotationMotionToQuaternion(
          timestamped_raw_rotation.rate_x_rad_s,
          timestamped_raw_rotation.rate_y_rad_s,
          timestamped_raw_rotation.rate_z_rad_s, imu_duration_sec);

      // Rotate the forward velocity to fixed reference frame and weigh by
      // interval duration.
      const Eigen::Matrix3d integrated_rotation_matrix =
          integrated_rotation.toRotationMatrix();
      const Eigen::Vector3d time_weighted_velocity_axis =
          imu_duration_sec * integrated_rotation_matrix * local_forward_axis;
      const double scalar_velocity = scalar_velocities[imu_interval_global_idx];
      integrated_travel += scalar_velocity * time_weighted_velocity_axis;
      reference_distance +=
          imu_duration_sec *
          reference_velocities_.at(imu_interval.reference_end_time_index)
              .velocity;

      const TimestampedAcceleration &timestamped_raw_acceleration =
          accelerations_.at(sensor_indices.at(1));
      const Eigen::Vector3d raw_acceleration =
          Eigen::Vector3d(timestamped_raw_acceleration.acc_x,
                          timestamped_raw_acceleration.acc_y,
                          timestamped_raw_acceleration.acc_z);
      const Eigen::Vector3d imu_delta_velocity =
          imu_duration_sec * (acceleration_global_bias +
                              integrated_rotation_matrix *
                                  (acceleration_local_bias + raw_acceleration));

      // Accumulating derivatives wrt per-segment scalar velocity
      d_integrated_travel_d_velocity_scales.push_back(
          time_weighted_velocity_axis);
      // Accumulating derivative wrt forward axis direction.
      d_integrated_travel_d_forward_axis +=
          scalar_velocity * imu_duration_sec * integrated_rotation_matrix;

      integrated_rotation = integrated_rotation * raw_rotation;
      const Eigen::Matrix3d end_integrated_rotation_matrix =
          integrated_rotation.toRotationMatrix();
      // Make sure incremented index is within bounds.
      CHECK_LT(imu_interval_global_idx + 1, scalar_velocities.size());
      const Eigen::Matrix3d delta_forward_transform =
          end_integrated_rotation_matrix *
              scalar_velocities[imu_interval_global_idx + 1] -
          integrated_rotation_matrix *
              scalar_velocities[imu_interval_global_idx];
      const Eigen::Vector3d forward_axis_delta_velocity =
          delta_forward_transform * local_forward_axis;

      const Eigen::Vector3d delta_velocity_diff =
          forward_axis_delta_velocity - imu_delta_velocity;
      acceleration_match_loss += delta_velocity_diff.squaredNorm();

      // wrt forward axis
      grad.segment<3>(FORWARD_AXIS_START) +=
          2.0 * delta_forward_transform.transpose() * delta_forward_transform *
          local_forward_axis;
      grad.segment<3>(FORWARD_AXIS_START) -=
          2.0 * delta_forward_transform.transpose() * imu_delta_velocity;

      // wrt velocity magnitudes before and after
      grad[VELOCITY_SCALES_START + imu_interval_global_idx] +=
          2.0 * scalar_velocities[imu_interval_global_idx] *
          local_forward_axis.transpose() *
          integrated_rotation_matrix.transpose() * integrated_rotation_matrix *
          local_forward_axis;
      grad[VELOCITY_SCALES_START + imu_interval_global_idx] -=
          2.0 * local_forward_axis.transpose() *
          integrated_rotation_matrix.transpose() *
          (end_integrated_rotation_matrix *
               scalar_velocities[imu_interval_global_idx + 1] *
               local_forward_axis -
           imu_delta_velocity);

      grad[VELOCITY_SCALES_START + imu_interval_global_idx + 1] +=
          2.0 * scalar_velocities[imu_interval_global_idx + 1] *
          local_forward_axis.transpose() *
          end_integrated_rotation_matrix.transpose() *
          end_integrated_rotation_matrix * local_forward_axis;
      grad[VELOCITY_SCALES_START + imu_interval_global_idx + 1] -=
          2.0 * local_forward_axis.transpose() *
          end_integrated_rotation_matrix.transpose() *
          (integrated_rotation_matrix *
               scalar_velocities[imu_interval_global_idx] * local_forward_axis +
           imu_delta_velocity);

      // wrt global acceleration bias
      grad.segment<3>(ACCELERATION_GLOBAL_BIAS_START) +=
          2.0 * imu_duration_sec * imu_duration_sec * acceleration_global_bias;
      grad.segment<3>(ACCELERATION_GLOBAL_BIAS_START) +=
          2.0 * imu_duration_sec *
          (imu_duration_sec * integrated_rotation_matrix *
           (acceleration_local_bias + raw_acceleration));
      grad.segment<3>(ACCELERATION_GLOBAL_BIAS_START) -=
          2.0 * imu_duration_sec * forward_axis_delta_velocity;

      // wrt local acceleration bias
      grad.segment<3>(ACCELERATION_LOCAL_BIAS_START) +=
          2.0 * imu_duration_sec * imu_duration_sec *
          integrated_rotation_matrix.transpose() * integrated_rotation_matrix *
          acceleration_local_bias;
      grad.segment<3>(ACCELERATION_LOCAL_BIAS_START) -=
          2.0 * imu_duration_sec * integrated_rotation_matrix.transpose() *
          (forward_axis_delta_velocity -
           imu_duration_sec * acceleration_global_bias -
           imu_duration_sec * integrated_rotation_matrix * raw_acceleration);
    }

    const double distance_diff = integrated_travel.norm() - reference_distance;
    travel_distance_loss += distance_diff * distance_diff;
    const Eigen::Vector3d d_loss_d_travel = 2.0 * distance_diff *
                                            integrated_travel /
                                            (integrated_travel.norm() + 1e-5);

    grad.segment<3>(FORWARD_AXIS_START) +=
        d_integrated_travel_d_forward_axis.transpose() * d_loss_d_travel;

    for (size_t reference_imu_idx = 0;
         reference_imu_idx < reference_interval.size(); ++reference_imu_idx) {
      const InterpolationInterval &imu_interval =
          reference_interval.at(reference_imu_idx);
      const Eigen::Vector3d &d_travel_d_velocity_scale =
          d_integrated_travel_d_velocity_scales.at(reference_imu_idx);
      grad[VELOCITY_SCALES_START + imu_interval.interpolation_end_time_index] +=
          d_travel_d_velocity_scale.dot(d_loss_d_travel);
    }
  }

  const double result = travel_distance_loss + acceleration_match_loss +
                        forward_axis_magnitude_loss;

  LOG(INFO) << "FixedForwardAxisCalibrator overall: " << result
            << "; travel: " << travel_distance_loss << "; acceleration "
            << acceleration_match_loss
            << "; forward axis: " << forward_axis_magnitude_loss;

  return result;
}

void FixedForwardAxisCalibrator::NormalizeVelocities(
    Eigen::VectorXd &calibrated_motion) {
  CHECK_GT(calibrated_motion.size(), VELOCITY_SCALES_START);
  const double forward_axis_scale =
      calibrated_motion.segment<3>(FORWARD_AXIS_START).norm();
  // Make sure the forward axis absolute magnitude is not absurdly small.
  CHECK_GT(forward_axis_scale, 1e-5);
  calibrated_motion.segment<3>(FORWARD_AXIS_START) /= forward_axis_scale;
  calibrated_motion.segment(VELOCITY_SCALES_START,
                            calibrated_motion.size() - VELOCITY_SCALES_START) *=
      forward_axis_scale;
}

FixedForwardAxisCalibrator::CalibrationResult
FixedForwardAxisCalibrator::StateVectorToCalibrationResult(
    const Eigen::VectorXd &state_vector) {
  CHECK_GT(state_vector.size(), VELOCITY_SCALES_START);
  return {state_vector.segment<3>(ACCELERATION_GLOBAL_BIAS_START),
          state_vector.segment<3>(ACCELERATION_LOCAL_BIAS_START),
          state_vector.segment<3>(FORWARD_AXIS_START),
          state_vector.segment(VELOCITY_SCALES_START,
                               state_vector.size() - VELOCITY_SCALES_START)};
}

const MergedTimeSeries &FixedForwardAxisCalibrator::ImuTimes() const {
  return imu_times_;
}
}
