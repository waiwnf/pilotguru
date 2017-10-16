#include "gtest/gtest.h"

#include <car/kia_steering_angle_holder.hpp>

namespace pilotguru {
namespace kia {
namespace {

class SteeringAngleHolderTest : public ::testing::Test {
protected:
  SteeringAngleHolderSettings settings;
};

TEST_F(SteeringAngleHolderTest, MeasuredOutOfBounds) {
  EXPECT_EQ(0, SteeringAngleHolderEffectiveTorque(0, 200, settings));
  EXPECT_EQ(0, SteeringAngleHolderEffectiveTorque(0, -200, settings));
  EXPECT_EQ(0, SteeringAngleHolderEffectiveTorque(200, -200, settings));
}

TEST_F(SteeringAngleHolderTest, ActualWithinToleranceOfTarget) {
  // Strict equality.
  EXPECT_EQ(0, SteeringAngleHolderEffectiveTorque(0, 0, settings));
  EXPECT_EQ(0, SteeringAngleHolderEffectiveTorque(50, 50, settings));
  EXPECT_EQ(0, SteeringAngleHolderEffectiveTorque(-100, -100, settings));

  // Small difference.
  EXPECT_EQ(0, SteeringAngleHolderEffectiveTorque(1, -1, settings));
  EXPECT_EQ(0, SteeringAngleHolderEffectiveTorque(50, 52, settings));
  EXPECT_EQ(0, SteeringAngleHolderEffectiveTorque(-100, -99, settings));
}

TEST_F(SteeringAngleHolderTest, LinearInterpolation) {
  // 5 * 8 / 8 = 5
  EXPECT_EQ(5, SteeringAngleHolderEffectiveTorque(56, 46, settings));
  EXPECT_EQ(-5, SteeringAngleHolderEffectiveTorque(-56, -46, settings));
  // 5 * 7 / 8 = 4.375
  EXPECT_EQ(4, SteeringAngleHolderEffectiveTorque(56, 47, settings));
  EXPECT_EQ(-4, SteeringAngleHolderEffectiveTorque(-56, -47, settings));
  // 5 * 6 / 8 = 3.75
  EXPECT_EQ(3, SteeringAngleHolderEffectiveTorque(56, 48, settings));
  EXPECT_EQ(-3, SteeringAngleHolderEffectiveTorque(-56, -48, settings));
  // 5 * 5 / 8 = 3.125
  EXPECT_EQ(3, SteeringAngleHolderEffectiveTorque(56, 49, settings));
  EXPECT_EQ(-3, SteeringAngleHolderEffectiveTorque(-56, -49, settings));
  // 5 * 4 / 8 = 2.5
  EXPECT_EQ(2, SteeringAngleHolderEffectiveTorque(56, 50, settings));
  EXPECT_EQ(-2, SteeringAngleHolderEffectiveTorque(-56, -50, settings));
  // 5 * 3 / 8 = 1.875
  EXPECT_EQ(1, SteeringAngleHolderEffectiveTorque(56, 51, settings));
  EXPECT_EQ(-1, SteeringAngleHolderEffectiveTorque(-56, -51, settings));
  // 5 * 2 / 8 = 1.25
  EXPECT_EQ(1, SteeringAngleHolderEffectiveTorque(56, 52, settings));
  EXPECT_EQ(-1, SteeringAngleHolderEffectiveTorque(-56, -52, settings));
  // 5 * 1 / 8 = 0.625
  EXPECT_EQ(0, SteeringAngleHolderEffectiveTorque(56, 53, settings));
  EXPECT_EQ(0, SteeringAngleHolderEffectiveTorque(-56, -53, settings));
  // 5 * 0 / 8 = 0
  EXPECT_EQ(0, SteeringAngleHolderEffectiveTorque(56, 54, settings));
  EXPECT_EQ(0, SteeringAngleHolderEffectiveTorque(-56, -54, settings));
}
}
}
}
