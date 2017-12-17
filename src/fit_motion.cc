// Auto-calibration and integration of IMU measurements (acceleration +
// gyroscope) using GPS data as coarse-grained reference points.
//
// Supported outputs:
// * Inferred vehicle forward axis direction in camera reference frame.
// * Vehicle angular velocity around inferred vertical (in vehicle reference
//   frame) axis. This corresponds to the rotation component from steering the
//   vehicle. The vertical axis is inferred with principal components analysis
//   on the 3D rotations data and assuming that rotations due to steering are
//   dominant in magnitude (i.e., the main principal axis is the vertical axis
//   of the vehicle). This computation can be done either with raw 3D rotations
//   from the IMU data, or with first subtracting the rotations component around
//   the forward vehicle axis (i.e. forcing the inferred vertical axis to be
//   orthogonal to the forward axis).
// * Vehicle velocity magnitude. For velocity inference, fits IMU calibration
//   parameters by matching IMU integrated travel distances to the GPS data.
//   Because some of the drift is not eliminated (not sure whether it white
//   noise or something systematic, e.g. gyroscope drift), instead of
//   calibrating globally on the whole recorded track, we repeatedly calibrate
//   independently using a relatively small sliding window (of e.g. 40 seconds)
//   with overlaps and averaging the results for every IMU timestamp.
//
// Writes out every output type to a separate JSON file.

#include <cmath>
#include <cstdlib>
#include <numeric>

#include <Eigen/Geometry>

#include <LBFGS.h>

#include <gflags/gflags.h>
#include <glog/logging.h>

#include <json.hpp>

#include <calibration/rotation.hpp>
#include <calibration/velocity.hpp>
#include <geometry/geometry.hpp>
#include <io/json_converters.hpp>
#include <math/math.hpp>
#include <slam/smoothing.hpp>

// Inputs

DEFINE_string(rotations_json, "",
              "JSON file with raw timestamped 3D rotations from the "
              "smartphone gyroscope. Comes from PilotGuru Recorder raw data.");
DEFINE_string(accelerations_json, "",
              "JSON file with raw timestamped 3D accelerations from the "
              "smartphone gyroscope. Comes from PilotGuru Recorder raw data. "
              "Preliminary accelerometer calibration is not necessary, nor "
              "detecting and subtracting the gravity component. This binary "
              "auto-calibrates the data by matching velocity magnitudes with "
              "GPS data.");
DEFINE_string(locations_json, "", "JSON file with GPS locations and derived "
                                  "absolute velocities. Comes from PilotGuru "
                                  "Recorder raw data.");

// Flags to select which stages to run. Each can be done independently of
// others.
DEFINE_bool(compute_velocities_from_imu, false,
            "Whether to compute vehicle forward velocity by autocalibrating "
            "IMU parameters against the GPS velocity data.");
DEFINE_bool(compute_steering_raw_rotations, false,
            "Whether to infer steering angular velocities directly from raw 3D "
            "rotations data.");
DEFINE_bool(compute_steering_using_forward_axis, false,
            "Whether to infer steering angular velocities directly from raw 3D "
            "rotations data.");

// Outputs

DEFINE_string(velocities_out_json, "",
              "JSON file to write timestamped absolute "
              "velocities derived from accelerometer "
              "data calibrated using GPS coarse-grained "
              "velocities.");
DEFINE_string(steering_out_json_raw, "",
              "JSON file to write rotations in the inferred horizontal plane, "
              "derived from raw 3D rotations data from the IMU. Results "
              "intended to closely match the 3D rotation component due to "
              "vehicle steering. Horizontal plane is detected via main "
              "principal axis of the 3D rotations.");
DEFINE_string(forward_axis_out_json, "", "JSON file to write the direction of "
                                         "the vehicle forward axis to. Forward "
                                         "axis direction is in the phone-local "
                                         "reference frame.");
DEFINE_string(steering_out_json_forward_axis_complementary, "",
              "JSON file to write rotations in the inferred horizontal plane, "
              "derived from 3D rotations data from the IMU with rotations "
              "component around the inferred forward axis subtracted out. "
              "Results intended to closely match the 3D rotation component due "
              "to vehicle steering. Horizontal plane is detected via main "
              "principal axis of the 3D rotations.");

// Inference parameters.

DEFINE_int64(locations_batch_size, 40,
             "Size of sliding window (in terms of the number of GPS "
             "measurements) to use for calibration. This should not be too "
             "large, as the results become less accurate for long windows "
             "because of accumulating IMU drift.");
DEFINE_int64(locations_shift_step, 5,
             "Step size (in terms of number of GPS measurements) by which to "
             "shift the sliding window for the subsequent calibration runs.");
DEFINE_int64(
    optimization_iters, 500,
    "Max number of L-BFGS iterations to use for every calibration run.");
DEFINE_double(post_smoothing_sigma_sec, 0.003,
              "Smoothing Gaussian kernel width (in seconds) for the final "
              "smoothing of the integrated velocities.");
DEFINE_int64(principal_rotation_axis_integration_interval_usec, 500000, "");
DEFINE_double(forward_axis_inference_min_velocity_m_s, 5.0,
              "Minimum velocity threshold for taking a measurement into "
              "account when computing the forward axis direction. Higher "
              "velocities are more likely to correspond to purely forward "
              "motion (no turning) and have relatively little noise.");

namespace {
template <typename T>
std::vector<T> ReadTimestamp3DData(const std::string &filename,
                                   const std::string &field_name) {
  std::unique_ptr<nlohmann::json> json_root = pilotguru::ReadJsonFile(filename);
  const auto &entries_list = (*json_root)[field_name];
  CHECK(!entries_list.empty());

  std::vector<T> result;
  for (const auto &entry : entries_list) {
    result.push_back({entry[pilotguru::kX], entry[pilotguru::kY],
                      entry[pilotguru::kZ], entry[pilotguru::kTimeUsec]});
  }
  return result;
}

std::vector<pilotguru::TimestampedVelocity>
ReadGpsVelocities(const string &filename) {
  std::unique_ptr<nlohmann::json> locations_json_root =
      pilotguru::ReadJsonFile(filename);
  const auto &locations_json = (*locations_json_root)[pilotguru::kLocations];
  CHECK(!locations_json.empty());
  std::vector<pilotguru::TimestampedVelocity> result;
  for (const auto &location : locations_json) {
    result.push_back(
        {location[pilotguru::kSpeedMS], location[pilotguru::kTimeUsec]});
  }
  return result;
}

// Infers the main principal rotation axis (assumed to be the vertical axis of
// the vehicle) and project all rotations onto that axis to get approximate
// rotations in the horizontal plane (corresponding to steering).
std::vector<double> ComputeAndSaveSteeringAngles(
    const std::vector<pilotguru::TimestampedRotationVelocity> &rotations_3d,
    int64_t rotation_integration_interval_usec,
    const std::string &out_json_name) {
  const cv::Mat pca_axes = pilotguru::GetPrincipalRotationAxes(
      rotations_3d, rotation_integration_interval_usec);
  const cv::Vec3d vertical_axis(pca_axes.at<double>(0, 0),
                                pca_axes.at<double>(0, 1),
                                pca_axes.at<double>(0, 2));
  const std::vector<double> steering_angles =
      GetAngularVelocitiesAroundAxisDirect(rotations_3d, vertical_axis);
  CHECK_EQ(steering_angles.size(), rotations_3d.size());

  // Save the projected horizontal rotations.
  const std::vector<long> rotation_timestamps =
      pilotguru::ExtractTimestamps(rotations_3d);
  pilotguru::JsonWriteTimestampedRealData(rotation_timestamps, steering_angles,
                                          out_json_name, pilotguru::kSteering,
                                          pilotguru::kAngularVelocity);

  return steering_angles;
}

void ComputeAndSaveForwardVelocitiesFromImu(
    const std::vector<pilotguru::TimestampedVelocity> &gps_velocities,
    const std::vector<pilotguru::TimestampedRotationVelocity> &rotations,
    const std::vector<pilotguru::TimestampedAcceleration> &accelerations,
    int64_t locations_batch_size, int64_t locations_shift_step,
    int64_t max_iters, double post_smoothing_sigma_sec,
    const std::string &velocities_out_json,
    Eigen::Vector3d *forward_axis_result,
    const std::string &forward_axis_out_json) {
  // Optimizer parameters are the same for all iterations.
  LBFGSpp::LBFGSParam<double> velocity_calibration_params;
  velocity_calibration_params.epsilon = 1e-5;
  velocity_calibration_params.max_iterations = max_iters;

  Eigen::Vector3d _a = Eigen::Vector3d::Zero(), _b = Eigen::Vector3d::Zero();
  pilotguru::KahanSum<Eigen::Vector3d> total_velocity_local(_a, _b);

  // Slide an inteval of locations_batch_size with a step of
  // locations_shift_step over the reference GPS velocity measurements,
  // and fit the calibration parameters for the inertial measurements falling
  // within that reference interval.
  std::map<size_t, std::vector<double>> integrated_velocities;
  for (size_t reference_start_idx = 0;
       reference_start_idx < gps_velocities.size();
       reference_start_idx += locations_shift_step) {
    const size_t reference_end_idx = std::min(
        reference_start_idx + locations_batch_size, gps_velocities.size());
    const std::vector<pilotguru::TimestampedVelocity> reference_interval(
        gps_velocities.begin() + reference_start_idx,
        gps_velocities.begin() + reference_end_idx);

    // Calibrator restricted to GPS measurements within the sliding window.
    pilotguru::AccelerometerCalibrator velocity_calibrator(
        reference_interval, rotations, accelerations);
    // Fit the calibration parameters.
    LBFGSpp::LBFGSSolver<double> velocity_calibration_solver(
        velocity_calibration_params);
    Eigen::VectorXd x = Eigen::VectorXd::Zero(9);
    double velocity_calibration_residual;
    int niter = velocity_calibration_solver.minimize(
        velocity_calibrator, x, velocity_calibration_residual);

    LOG(INFO) << "Sliding window optimization: " << niter
              << " iterations, result value: " << velocity_calibration_residual;

    // TODO: group this into a struct and factor out vector-struct conversion
    // to calibrator.
    const Eigen::Vector3d acceleration_global_bias(x[0], x[1], x[2]);
    const Eigen::Vector3d acceleration_local_bias(x[3], x[4], x[5]);
    const Eigen::Vector3d initial_velocity(x[6], x[7], x[8]);

    // Integrate the inertial measuremens within the slifding window using the
    // optimal fitted calibration parameters.
    const std::map<size_t, pilotguru::MotionIntegrationOutcome>
        integrated_trajectory =
            velocity_calibrator.IntegrateTrajectory(acceleration_global_bias,
                                                    acceleration_local_bias,
                                                    initial_velocity);

    // Store velocity magnitudes within the sliding window for averaging
    // later.
    for (const auto &point : integrated_trajectory) {
      integrated_velocities[point.first].push_back(
          point.second.velocity.norm());
    }

    // Sum up all the velocity measurements, rotated to device-local reference
    // frame, to get an estimate of forward travel direction.
    for (const auto &point : integrated_trajectory) {
      if (point.second.velocity.norm() >=
          FLAGS_forward_axis_inference_min_velocity_m_s) {
        // Conjugate corresponds to inverse rotation for a normalized
        // quaternion.
        const Eigen::Quaterniond orientation_inverse =
            point.second.orientation.conjugate();
        const Eigen::Vector3d velocity_local_frame =
            orientation_inverse._transformVector(point.second.velocity);
        total_velocity_local.add(velocity_local_frame);
      }
    }
  }

  // This is only created to merge rotations and accelerations again and get
  // indices into the merged time series.
  // TODO: factor out merged time series into a class?
  pilotguru::AccelerometerCalibrator calibrator(gps_velocities, rotations,
                                                accelerations);

  std::vector<double> averaged_integrated_velocities;
  std::vector<double> timestamps_sec; // For smoothing.
  std::vector<long> timestamps_usec;  // For writing to JSON.
  // Average the velocities among the sliding windows falling on every IMU
  // measurement.
  for (auto &point : integrated_velocities) {
    timestamps_usec.push_back(
        calibrator.ImuTimes().MergedEventTimeUsec(point.first));
    timestamps_sec.push_back(
        static_cast<double>(timestamps_usec.back() - timestamps_usec.front()) *
        1e-6);
    const double velocity_sum =
        std::accumulate(point.second.begin(), point.second.end(), 0.0);
    averaged_integrated_velocities.push_back(velocity_sum /
                                             point.second.size());
  }
  // Temporal post-smoothing to remove the very high frequency noise.
  const std::vector<double> smoothed_velocities = pilotguru::SmoothTimeSeries(
      averaged_integrated_velocities, timestamps_sec, timestamps_sec,
      post_smoothing_sigma_sec);

  pilotguru::JsonWriteTimestampedRealData(
      timestamps_usec, smoothed_velocities, velocities_out_json,
      pilotguru::kVelocities, pilotguru::kSpeedMS);

  CHECK_NOTNULL(forward_axis_result);
  *forward_axis_result = total_velocity_local.sum();
  (*forward_axis_result) /= (forward_axis_result->norm() + 1.0);

  nlohmann::json forward_axis_json_root;
  auto &forward_axis_json = forward_axis_json_root[pilotguru::kForwardAxis];
  forward_axis_json[pilotguru::kX] = (*forward_axis_result)(0);
  forward_axis_json[pilotguru::kY] = (*forward_axis_result)(1);
  forward_axis_json[pilotguru::kZ] = (*forward_axis_result)(2);
  pilotguru::WriteJsonFile(forward_axis_json_root, forward_axis_out_json);
}
}

int main(int argc, char **argv) {
  using pilotguru::FixedForwardAxisCalibrator;

  google::InitGoogleLogging(argv[0]);
  google::ParseCommandLineFlags(&argc, &argv, true);
  google::InstallFailureSignalHandler();

  // Sanity checks.
  CHECK(!FLAGS_rotations_json.empty());
  CHECK(!FLAGS_accelerations_json.empty());
  CHECK(!FLAGS_locations_json.empty());
  CHECK_GT(FLAGS_optimization_iters, 0);
  CHECK_GT(FLAGS_locations_batch_size, 0);
  CHECK_GT(FLAGS_locations_shift_step, 0);
  CHECK_GE(FLAGS_locations_batch_size, FLAGS_locations_shift_step);
  CHECK_GT(FLAGS_post_smoothing_sigma_sec, 0);
  CHECK_GT(FLAGS_principal_rotation_axis_integration_interval_usec, 0);

  // Read input JSONs.
  const std::vector<pilotguru::TimestampedVelocity> gps_velocities =
      ReadGpsVelocities(FLAGS_locations_json);
  const std::vector<pilotguru::TimestampedRotationVelocity> rotations =
      ReadTimestamp3DData<pilotguru::TimestampedRotationVelocity>(
          FLAGS_rotations_json, pilotguru::kRotations);
  const std::vector<pilotguru::TimestampedAcceleration> accelerations =
      ReadTimestamp3DData<pilotguru::TimestampedAcceleration>(
          FLAGS_accelerations_json, pilotguru::kAccelerations);

  if (FLAGS_compute_steering_raw_rotations) {
    CHECK(!FLAGS_steering_out_json_raw.empty());
    ComputeAndSaveSteeringAngles(
        rotations, FLAGS_principal_rotation_axis_integration_interval_usec,
        FLAGS_steering_out_json_raw);
  }

  Eigen::Vector3d forward_axis = Eigen::Vector3d::Zero();
  if (FLAGS_compute_velocities_from_imu) {
    CHECK(!FLAGS_velocities_out_json.empty());
    ComputeAndSaveForwardVelocitiesFromImu(
        gps_velocities, rotations, accelerations, FLAGS_locations_batch_size,
        FLAGS_locations_shift_step, FLAGS_optimization_iters,
        FLAGS_post_smoothing_sigma_sec, FLAGS_velocities_out_json,
        &forward_axis, FLAGS_forward_axis_out_json);
  }

  if (FLAGS_compute_steering_using_forward_axis) {
    CHECK(FLAGS_compute_velocities_from_imu);

    const cv::Vec3d forward_axis_cv(forward_axis(0), forward_axis(1),
                                    forward_axis(2));
    // Remove the rotations around the vehicle forward axis from the raw
    // rotations data. This will force the main principal rotation axis to be
    // orthogonal to the vehicle forward axis.
    const std::vector<pilotguru::TimestampedRotationVelocity>
        rotations_no_forward_axis =
            pilotguru::GetRotationsComplementaryToAxisDirect(rotations,
                                                             forward_axis_cv);
    ComputeAndSaveSteeringAngles(
        rotations_no_forward_axis,
        FLAGS_principal_rotation_axis_integration_interval_usec,
        FLAGS_steering_out_json_forward_axis_complementary);
  }

  return EXIT_SUCCESS;
}
