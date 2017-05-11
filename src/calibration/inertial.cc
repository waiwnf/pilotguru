#include <calibration/inertial.hpp>

#include <geometry/geometry.hpp>
#include <logging/strings.hpp>

#include <Eigen/Geometry>

#include <glog/logging.h>

namespace pilotguru {
namespace {
// From a vector of structs, extracts a vector of timestamps to pass to
// MergeTimeSeries() later.
template <typename T>
std::vector<long> ExtractTimestamps(const std::vector<T> &events) {
  std::vector<long> result;
  for (const T &event : events) {
    result.push_back(event.time_usec);
  }
  return result;
}

// Merge the accelerations and rotations time series.
std::vector<std::vector<size_t>> InitSensorEvents(
    const std::vector<TimestampedRotationVelocity> &rotation_velocities,
    const std::vector<TimestampedAcceleration> &accelerations) {
  const std::vector<long> rotation_times =
      ExtractTimestamps(rotation_velocities);
  const std::vector<long> acceleration_times = ExtractTimestamps(accelerations);
  return MergeTimeSeries({&rotation_times, &acceleration_times});
}

// Init the interpolation intervals for the merged accelerations+rotations time
// series wrt the reference time series of GPS measurements.
std::vector<std::vector<InterpolationInterval>> InitInterpolationIntervals(
    const std::vector<TimestampedVelocity> &reference_velocities,
    const std::vector<TimestampedRotationVelocity> &rotation_velocities,
    const std::vector<TimestampedAcceleration> &accelerations,
    const std::vector<std::vector<size_t>> &merged_events) {
  const std::vector<long> rotation_times =
      ExtractTimestamps(rotation_velocities);
  const std::vector<long> acceleration_times = ExtractTimestamps(accelerations);

  std::vector<long> merged_times;
  for (const std::vector<size_t> &merged_event : merged_events) {
    merged_times.push_back(GetEffectiveTimeStamp(
        {&rotation_times, &acceleration_times}, merged_event));
  }

  const std::vector<long> reference_timestamps =
      ExtractTimestamps(reference_velocities);
  return MakeInterpolationIntervals(reference_timestamps, merged_times);
}
}

AccelerometerCalibrator::AccelerometerCalibrator(
    const std::vector<TimestampedVelocity> &reference_velocities,
    const std::vector<TimestampedRotationVelocity> &rotation_velocities,
    const std::vector<TimestampedAcceleration> &accelerations)
    : reference_velocities_(reference_velocities),
      rotation_velocities_(rotation_velocities), accelerations_(accelerations),
      sensor_events_(InitSensorEvents(rotation_velocities_, accelerations_)),
      reference_intervals_(
          InitInterpolationIntervals(reference_velocities, rotation_velocities,
                                     accelerations, sensor_events_)) {}

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

    // Cumulative rotation matrices from the beginning for each
    // accelerometer/gyro measurement within this reference interval. Necessary
    // for computing the derivative wrt local accelerometer bias.
    std::vector<Eigen::Quaterniond> interval_integrated_rotations;

    // Iterate over all inertial measurements on the given GPS reference
    // interval.

    std::vector<MotionIntegrationOutcome> integrated_intervals;
    for (const InterpolationInterval &interval : intervals) {
      const std::vector<size_t> &sensor_indices =
          sensor_events_.at(interval.interpolation_end_time_index);
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

const std::vector<std::vector<size_t>> &
AccelerometerCalibrator::MergedSensorEvents() const {
  return sensor_events_;
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
          sensor_events_.at(interpolation_index);
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
}