#include <car/kalman_filter.hpp>

#include <glog/logging.h>

namespace pilotguru {
KalmanFilter1D::KalmanFilter1D(double observation_variance,
                               double perturbation_variance_per_second)
    : observation_variance_(observation_variance),
      perturbation_variance_per_second_(perturbation_variance_per_second),
      observation_matrix_((Eigen::RowVector2d() << 1.0, 0.0).finished()) {
  CHECK_GT(observation_variance_, 0);
  CHECK_GT(perturbation_variance_per_second_, 0);
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
    timeval delta_t;
    timersub(&(observation.timestamp()), &(latest_estimate_.timestamp()),
             &delta_t);
    double delta_t_sec = static_cast<double>(delta_t.tv_sec) +
                         static_cast<double>(delta_t.tv_usec) * 1e-6;
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
            .finished());
    // Predicted current value based only on previous value and velocity.
    // (2x2) * (2x1) = (2x1)
    const Eigen::Vector2d x_k_k_minus_1 = F * latest_estimate_.data().mean;
    // Covariance estimate based only on previous value and velocity estimate,
    // without taking current observation into account.
    // First component: (2x2) * (2x2) * (2x2) = (2x2)
    // Second component: scalar * (2x1) * (1x2) = (2x2)
    const Eigen::Matrix2d P_k_k_minus_1 =
        F * latest_estimate_.data().covariance * F.transpose() +
        perturbation_variance_per_second_ * G * G.transpose();
    // Observation residual.
    // First component: scalar
    // Second component: (1x2) * (2x1) = (1x1) = scalar
    const double y_k = observation.data() - observation_matrix_ * x_k_k_minus_1;
    // Observation residual covariance.
    // First component: (1x2) * (2x2) * (2x1) = (1x1) = scalar
    // Second component: scalar
    const double S =
        observation_matrix_ * P_k_k_minus_1 * observation_matrix_.transpose() +
        observation_variance_;
    // Optimal Kalman gain.
    // (2x2) * (2x1) / scalar = (2x1)
    const Eigen::Vector2d K =
        P_k_k_minus_1 * observation_matrix_.transpose() / S;
    // Final estimate is estimate from previous step, adjusted by the
    // observation residual.
    // First component: (2x1)
    // Second component: scalar * (2x1) = (2x1)
    next_estimate.mean = x_k_k_minus_1 + y_k * K;
    // Final estimate covariance.
    // ((2x2) - (2x1) * (1x2)) * (2x2) = (2x2)
    next_estimate.covariance =
        (Eigen::Matrix2d::Identity() - K * observation_matrix_) * P_k_k_minus_1;
  }
  latest_estimate_ = {next_estimate, observation.timestamp()};
}

const Timestamped<KalmanFilter1D::EstimateValue> &
KalmanFilter1D::LatestEstimate() const {
  CHECK(has_estimate_);
  return latest_estimate_;
}
}
