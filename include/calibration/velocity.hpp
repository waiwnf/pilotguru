#ifndef PILOTGURU_CALIBRATION_VELOCITY_HPP_
#define PILOTGURU_CALIBRATION_VELOCITY_HPP_

// Logic for auto-calibrating GPS (gravity subtraction and device debiasing) and
// gyroscope data using GPS reference points. Infers calibration parameters:
// - 'gravity', i.e. acceleration bias in the resting reference frame
// - accelerometer bias in the device-local reference frame
// - initial device speed
// by matching the integrated travel distances with GPS data.
//
// The resulting debiased accelerometer/data lets us interpolate the
// coarse-grained GPS data to achieve much higher temporal resolution.
// GPS data typically comes at ~1Hz, while accelerometer can get up to ~500Hz.

#include <map>
#include <vector>

#include <Eigen/Core>

#include <calibration/data.hpp>
#include <geometry/geometry.hpp>
#include <interpolation/align_time_series.hpp>
#include <optimization/loss_function.hpp>

namespace pilotguru {

// Loss function for an L2 difference between the travel distances derived from
// GPS and integrated from accelerometer and gyroscope data.
//
// Given the parameters ([acceleration bias in fixed reference frame],
// [accelerometer bias in device reference frame], [initial velocity]),
// computes
//
//   sum_i (gps_distance[i] - motion_integrated_distance[i])^2
//
// where i is the index of an interval between two neighboring GPS measurements.
// The gradient wrt parameters is also computed.
class AccelerometerCalibrator : public LossFunction {
public:
  // Does not take ownership of arguments. All vectors must outlive this object.
  AccelerometerCalibrator(
      const std::vector<TimestampedVelocity>
          &reference_velocities /* from GPS */,
      const std::vector<TimestampedRotationVelocity> &rotation_velocities,
      const std::vector<TimestampedAcceleration> &accelerations);

  // For simple gradient descent optimmization framework.
  double eval(const std::vector<double> &in,
              std::vector<double> *gradient) override;

  // For the LBFGS implementation.
  double operator()(const Eigen::VectorXd &x, Eigen::VectorXd &grad);

  const std::vector<std::vector<size_t>> &MergedSensorEvents() const;

  // Index is into MergedSensorEvents();
  long GetSensorEventTimestamp(const size_t sensor_event_index) const;

  // Keys are indices into MergedSensorEvents().
  const std::map<size_t, MotionIntegrationOutcome>
  IntegrateTrajectory(const Eigen::Vector3d &acceleration_global_bias,
                      const Eigen::Vector3d &acceleration_local_bias,
                      const Eigen::Vector3d &initial_velocity);

private:
  const std::vector<TimestampedVelocity> &reference_velocities_;
  const std::vector<TimestampedRotationVelocity> &rotation_velocities_;
  const std::vector<TimestampedAcceleration> &accelerations_;

  // Events timestamps.
  const std::vector<long> rotation_times_, accelerations_times_;

  // Merged rotations and accelerations time series.
  const std::vector<std::vector<size_t>> sensor_events_;

  // Intervals of the merged rotations/accelerations wrt reference velocities.
  const std::vector<std::vector<pilotguru::InterpolationInterval>>
      reference_intervals_;
};
}

#endif // PILOTGURU_CALIBRATION_VELOCITY_HPP_