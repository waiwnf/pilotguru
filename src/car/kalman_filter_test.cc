#include "gtest/gtest.h"

#include <car/kalman_filter.hpp>

namespace pilotguru {
namespace kia {
namespace {

class KalmanFilter1DTest : public ::testing::Test {};

TEST_F(KalmanFilter1DTest, NoMeasurements) {
  KalmanFilter1D filter(2.0, 1e4);
  EXPECT_DEATH(filter.LatestEstimate(), "");
}

TEST_F(KalmanFilter1DTest, FirstMeasurement) {
  KalmanFilter1D filter(2.0, 1e4);
  const Timestamped<double> observation = {1.0, {0, 0}};
  filter.Update(observation);
  const Timestamped<KalmanFilter1D::EstimateValue> &result =
      filter.LatestEstimate();
  EXPECT_EQ(result.data().mean(0), 1.0);
  EXPECT_EQ(result.data().mean(1), 0.0);
}

TEST_F(KalmanFilter1DTest, TwoMeasurements) {
  KalmanFilter1D filter(2.0, 1e4);
  filter.Update({1.0, {0, 0}});
  filter.Update({4.0, {0, 500000}});
  // F = (1, 0.5 ; 0, 1)
  // G = (0.5^2 / 2; 0.5) = (1/8 ; 1/2)
  // x_k_k_minus_1 = F x_prev = (1, 0.5 ; 0 1) * (1; 0) = (1; 0)
  // P_k_k_minus_1 = F P_prev F^T + G G^T perturbation_variance_per_second =
  //       (1, 0.5 ; 0, 1) (1, 0; 0, 1) (1, 0 ; 0.5, 1) +
  //       (1/64, 1/16; 1/16, 1/4) * 1e4
  //    =  (1.25, 0.5; 0.5 1) + (156.25, 625; 625, 2500)
  //    =  (157.5, 625.5; 625.5, 2501)
  // y_k = z_k - H x_prev = 4.0 - (1, 0) * (1; 0) = 3.0
  // S = H P_k_k_minus_1 H^T + observation_variance
  //    = (1, 0) (157.5, 625.5; 625.5, 2501) * (1; 0) + 2.0
  //    = 157.5 + 2.0 = 159.5
  // K = P_k_k_minus_1 * H^T / S
  //    = (157.5, 625.5; 625.5, 2501) * (1;0) / 159.5
  //    = (157.5 / 159.5; 625.5 / 159.5)
  // x_k = x_k_k_minus_1 + y_k * K
  //     = (1; 0) + 3.0 * (157.5 / 159.5; 625.5 / 159.5)
  //     = (1 + 3 * 157.5 / 159.5;  3 * 625.5 / 159.5)
  // P_k = (I - K H) * P_k_k_minus_1
  //     = ((1, 0; 0, 1) - (157.5 / 159.5; 625.5 / 159.5) * (1, 0)) *
  //           (157.5, 625.5; 625.5, 2501)
  //     = (1 - 157.5 / 159.5, 0; -625.5 / 159.5, 1) *
  //           (157.5, 625.5; 625.5, 2501)

  const Timestamped<KalmanFilter1D::EstimateValue> &result =
      filter.LatestEstimate();
  EXPECT_DOUBLE_EQ(result.data().mean(0), 1.0 + 157.5 * 3 / 159.5);
  EXPECT_DOUBLE_EQ(result.data().mean(1), 3 * 625.5 / 159.5);

  const Eigen::Matrix2d expected_covariance =
      (Eigen::Matrix2d() << (1.0 - 157.5 / 159.5), 0, (-625.5 / 159.5),
       1).finished() *
      (Eigen::Matrix2d() << 157.5, 625.5, 625.5, 2501).finished();
  EXPECT_DOUBLE_EQ(result.data().covariance(0, 0), expected_covariance(0, 0));
  EXPECT_DOUBLE_EQ(result.data().covariance(0, 1), expected_covariance(0, 1));
  EXPECT_DOUBLE_EQ(result.data().covariance(1, 0), expected_covariance(1, 0));
  EXPECT_DOUBLE_EQ(result.data().covariance(1, 1), expected_covariance(1, 1));
}
}
}
}
