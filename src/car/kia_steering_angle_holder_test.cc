#include "gtest/gtest.h"

#include <car/kia_steering_angle_holder.hpp>

namespace pilotguru {
namespace kia {
namespace {

class BoundedRotationVelocityEffectiveTorqueTest : public ::testing::Test {
protected:
  SteeringAngleHolderSettings settings;
};

TEST_F(BoundedRotationVelocityEffectiveTorqueTest, ValuesOutOfBounds) {
  // Current torque out of bounds.
  EXPECT_EQ(0,
            BoundedRotationVelocityEffectiveTorque(40, 20, -20, 0, settings));
  EXPECT_EQ(0,
            BoundedRotationVelocityEffectiveTorque(-40, 20, -20, 0, settings));

  // Measured angle out of bounds.
  EXPECT_EQ(0, BoundedRotationVelocityEffectiveTorque(0, 0, 200, 0, settings));
  EXPECT_EQ(0, BoundedRotationVelocityEffectiveTorque(0, 0, -200, 0, settings));
}

TEST_F(BoundedRotationVelocityEffectiveTorqueTest,
       RotationVelocityWithinBounds) {
  // Positive direction saturated.
  EXPECT_EQ(
      3, BoundedRotationVelocityEffectiveTorque(3, 20, -20, 100.0, settings));
  EXPECT_EQ(
      -3, BoundedRotationVelocityEffectiveTorque(-3, 20, -20, 100.0, settings));

  // Negative direction saturated.
  EXPECT_EQ(
      4, BoundedRotationVelocityEffectiveTorque(4, -20, 20, -100.0, settings));
  EXPECT_EQ(-4, BoundedRotationVelocityEffectiveTorque(-4, -20, 20, -100.0,
                                                       settings));

  // Linear interpolation between
  // -min_steering_rotation_degrees_per_second = -90 and 0
  // within
  // [-target_angle_diff_full_angular_velocity_lower_bound,
  // -target_angle_accuracy_tolerance_degrees] = 9 degrees
  // results in a slope of 10, so 1 degree diff shifts the acceptable rotation
  // velocity interval by 10.
  EXPECT_EQ(3, BoundedRotationVelocityEffectiveTorque(3, 4, 3, 20, settings));
  EXPECT_EQ(-3,
            BoundedRotationVelocityEffectiveTorque(-3, 3, 4, -20, settings));

  EXPECT_EQ(3,
            BoundedRotationVelocityEffectiveTorque(3, -20, -18, -10, settings));
  EXPECT_EQ(
      -3, BoundedRotationVelocityEffectiveTorque(-3, -20, -18, -30, settings));
}

TEST_F(BoundedRotationVelocityEffectiveTorqueTest,
       RotationVelocityOutOfBoundsTorqueNotSaturated) {
  // Constant hard caps region.
  EXPECT_EQ(3.2,
            BoundedRotationVelocityEffectiveTorque(3, 20, -20, 89, settings));
  EXPECT_EQ(-3.2, BoundedRotationVelocityEffectiveTorque(-3, 20, -20, 271.0,
                                                         settings));

  EXPECT_EQ(3.8,
            BoundedRotationVelocityEffectiveTorque(4, -20, 20, -5.0, settings));
  EXPECT_EQ(-3.8, BoundedRotationVelocityEffectiveTorque(-4, -20, 20, -335.0,
                                                         settings));

  // Linear interpolation region.
  EXPECT_EQ(-3.2,
            BoundedRotationVelocityEffectiveTorque(-3, 20, 19, 21.0, settings));
  EXPECT_EQ(3.2,
            BoundedRotationVelocityEffectiveTorque(3, 20, 19, -1.0, settings));

  EXPECT_EQ(
      2.8, BoundedRotationVelocityEffectiveTorque(3, -20, -18, -9.0, settings));
  EXPECT_EQ(-2.8, BoundedRotationVelocityEffectiveTorque(-3, -20, -18, -31.0,
                                                         settings));
}

TEST_F(BoundedRotationVelocityEffectiveTorqueTest,
       RotationVelocityOutOfBoundsTorqueSaturated) {
  // Constant hard caps region.
  EXPECT_EQ(5,
            BoundedRotationVelocityEffectiveTorque(5, 20, -20, 89, settings));
  EXPECT_EQ(
      -5, BoundedRotationVelocityEffectiveTorque(-5, 20, -20, 271.0, settings));

  EXPECT_EQ(
      -5, BoundedRotationVelocityEffectiveTorque(-5, -20, 20, -5.0, settings));
  EXPECT_EQ(
      5, BoundedRotationVelocityEffectiveTorque(5, -20, 20, -335.0, settings));

  // Linear interpolation region.
  EXPECT_EQ(-5,
            BoundedRotationVelocityEffectiveTorque(-5, 20, 19, 21.0, settings));
  EXPECT_EQ(5,
            BoundedRotationVelocityEffectiveTorque(5, 20, 19, -1.0, settings));

  EXPECT_EQ(
      -5, BoundedRotationVelocityEffectiveTorque(-5, -20, -18, -9.0, settings));
  EXPECT_EQ(
      5, BoundedRotationVelocityEffectiveTorque(5, -20, -18, -31.0, settings));
}
}
}
}
