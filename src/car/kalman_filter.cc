#include <car/kalman_filter.hpp>

#include <glog/logging.h>

namespace pilotguru {
namespace {
template <int StateDims>
void KalmanUpdate(
    const Eigen::Matrix<double, StateDims, 1> &prev_mean,
    const Eigen::Matrix<double, StateDims, StateDims> &prev_covariance,
    const Eigen::Matrix<double, StateDims, StateDims>
        &F /* Dynamics based on previous estimate. */,
    const Eigen::Matrix<double, StateDims, 1> &G /* Random noise. */,
    const Eigen::Matrix<double, 1, StateDims> &observation_matrix,
    double observation_value, double observation_variance,
    Eigen::Matrix<double, StateDims, 1, Eigen::DontAlign> &next_mean,
    Eigen::Matrix<double, StateDims, StateDims, Eigen::DontAlign>
        &next_covariance) {
  // Predicted current value based only on previous value and velocity.
  // (DxD) * (Dx1) = (Dx1)
  const Eigen::Matrix<double, StateDims, 1> x_k_k_minus_1 = F * prev_mean;
  // Covariance estimate based only on previous value and velocity estimate,
  // without taking current observation into account.
  // First component: (DxD) * (DxD) * (DxD) = (DxD)
  // Second component: scalar * (Dx1) * (1xD) = (DxD)
  const Eigen::Matrix<double, StateDims, StateDims> P_k_k_minus_1 =
      F * prev_covariance * F.transpose() + G * G.transpose();
  // Observation residual.
  // First component: scalar
  // Second component: (1xD) * (Dx1) = (1x1) = scalar
  const double y_k = observation_value - observation_matrix * x_k_k_minus_1;
  // Observation residual covariance.
  // First component: (1xD) * (DxD) * (Dx1) = (1x1) = scalar
  // Second component: scalar
  const double S =
      observation_matrix * P_k_k_minus_1 * observation_matrix.transpose() +
      observation_variance;
  // Optimal Kalman gain.
  // (DxD) * (Dx1) / scalar = (Dx1)
  const Eigen::Matrix<double, StateDims, 1> K =
      P_k_k_minus_1 * observation_matrix.transpose() / S;
  // Final estimate is estimate from previous step, adjusted by the
  // observation residual.
  // First component: (Dx1)
  // Second component: scalar * (Dx1) = (Dx1)
  next_mean = x_k_k_minus_1 + y_k * K;
  // Final estimate covariance.
  // ((DxD) - (Dx1) * (1xD)) * (DxD) = (DxD)
  next_covariance = (Eigen::Matrix<double, StateDims, StateDims>::Identity() -
                     K * observation_matrix) *
                    P_k_k_minus_1;
}

double TimevalToSeconds(const timeval &t) {
  return static_cast<double>(t.tv_sec) + static_cast<double>(t.tv_usec) * 1e-6;
}

double TimevalDiffToSeconds(const timeval &a, const timeval &b) {
  timeval delta_t;
  timersub(&a, &b, &delta_t);
  return TimevalToSeconds(delta_t);
}
}

KalmanFilter1D::KalmanFilter1D(double observation_variance,
                               double perturbation_variance_per_second)
    : observation_variance_(observation_variance),
      sqrt_perturbation_variance_per_second_(
          sqrt(perturbation_variance_per_second)),
      observation_matrix_((Eigen::RowVector2d() << 1.0, 0.0).finished()) {
  CHECK_GT(observation_variance_, 0);
  CHECK_GT(sqrt_perturbation_variance_per_second_, 0);
}

void KalmanFilter1D::Update(const Timestamped<double> &observation) {
  EstimateValue next_estimate;
  // This is the first observation, simply set the estimate to the observed
  // value.
  if (!has_estimate_) {
    next_estimate.mean << observation.data(), 0.0;
    next_estimate.covariance = Eigen::Matrix2d::Identity();
    has_estimate_ = true;
  } else {
    const double delta_t_sec = TimevalDiffToSeconds(
        observation.timestamp(), latest_estimate_.timestamp());
    // Notation mostly from https://en.wikipedia.org/wiki/Kalman_filter#Details
    // Prediction matrix based on value and velocity estimates from the previous
    // time step.
    // (2x2)
    const Eigen::Matrix2d F(
        (Eigen::Matrix2d() << 1.0, delta_t_sec, 0.0, 1.0).finished());
    // Estimate adjustment coefficient due to random constant acceleation over
    // the past time step.
    // (2x1)
    const Eigen::Vector2d G(
        (Eigen::Vector2d() << (delta_t_sec * delta_t_sec * 0.5), delta_t_sec)
            .finished() *
        sqrt_perturbation_variance_per_second_);

    KalmanUpdate(latest_estimate_.data().mean,
                 latest_estimate_.data().covariance, F, G, observation_matrix_,
                 observation.data(), observation_variance_, next_estimate.mean,
                 next_estimate.covariance);
  }
  latest_estimate_ = {next_estimate, observation.timestamp()};
}

const Timestamped<KalmanFilter1D::EstimateValue> &
KalmanFilter1D::LatestEstimate() const {
  CHECK(has_estimate_);
  return latest_estimate_;
}

KalmanFilter1D2Order::KalmanFilter1D2Order(
    double observation_variance, double perturbation_variance_per_second)
    : observation_variance_(observation_variance),
      sqrt_perturbation_variance_per_second_(
          sqrt(perturbation_variance_per_second)),
      observation_matrix_((Eigen::RowVector3d() << 1.0, 0.0, 0.0).finished()) {
  CHECK_GT(observation_variance_, 0);
  CHECK_GT(sqrt_perturbation_variance_per_second_, 0);
}

void KalmanFilter1D2Order::Update(const Timestamped<double> &observation) {
  EstimateValue next_estimate;
  // This is the first observation, simply set the estimate to the observed
  // value.
  if (!has_estimate_) {
    next_estimate.mean << observation.data(), 0.0, 0.0;
    next_estimate.covariance = Eigen::Matrix3d::Identity();
    has_estimate_ = true;
  } else {
    const double delta_t_sec = TimevalDiffToSeconds(
        observation.timestamp(), latest_estimate_.timestamp());

    // Acceleration diagonal element is 0.8 instead of 1.0 to reduce
    // acceleration inertia.
    const Eigen::Matrix3d F((Eigen::Matrix3d() << 1.0, delta_t_sec,
                             (delta_t_sec * delta_t_sec * 0.5), 0.0, 1.0,
                             delta_t_sec, 0.0, 0.0, 0.8)
                                .finished());
    const Eigen::Vector3d G(
        (Eigen::Vector3d() << (delta_t_sec * delta_t_sec * delta_t_sec / 6.0),
         (delta_t_sec * delta_t_sec * 0.5), delta_t_sec)
            .finished() *
        sqrt_perturbation_variance_per_second_);

    KalmanUpdate(latest_estimate_.data().mean,
                 latest_estimate_.data().covariance, F, G, observation_matrix_,
                 observation.data(), observation_variance_, next_estimate.mean,
                 next_estimate.covariance);
  }
  latest_estimate_ = {next_estimate, observation.timestamp()};
}

const Timestamped<KalmanFilter1D2Order::EstimateValue> &
KalmanFilter1D2Order::LatestEstimate() const {
  CHECK(has_estimate_);
  return latest_estimate_;
}
}
