#include "gtest/gtest.h"

#include <spoof-voltage-smoothing.h>

namespace {

constexpr int16_t max_steering_magnitude_for_test = 6;

class TargetVoltageSmootherTest : public ::testing::Test {
public:
  TargetVoltageSmootherTest() {
    settings_.max_steering_magnitude = max_steering_magnitude_for_test;
    settings_.steps_at_target_level = 1000;
    settings_.voltage_update_hystheresis = 1;
  }

protected:
  SteeringSpoofSettings settings_;
};

TEST_F(TargetVoltageSmootherTest, NoData) {
  TargetVoltageSmoother smoother(settings_, 8 /* adc_bits */, 8 /* dac_bits */);
  EXPECT_EQ(smoother.get_current_offset_dac_units(), 0);
  EXPECT_EQ(smoother.get_target_blue_voltage_dac_units(), 0);
  EXPECT_EQ(smoother.get_target_green_voltage_dac_units(), 0);
}

TEST_F(TargetVoltageSmootherTest, NoTargetOffset) {
  TargetVoltageSmoother smoother(settings_, 8 /* adc_bits */, 8 /* dac_bits */);
  smoother.update_measurments({100, 200});
  EXPECT_EQ(smoother.get_smoothed_green_voltage_adc_units(), 100);
  EXPECT_EQ(smoother.get_smoothed_blue_voltage_adc_units(), 200);
  EXPECT_EQ(smoother.get_current_offset_dac_units(), 0);
  EXPECT_EQ(smoother.get_target_green_voltage_dac_units(), 100);
  EXPECT_EQ(smoother.get_target_blue_voltage_dac_units(), 200);
}

TEST_F(TargetVoltageSmootherTest, TargetOffsetNoStep) {
  TargetVoltageSmoother smoother(settings_, 8 /* adc_bits */, 8 /* dac_bits */);
  smoother.update_measurments({100, 200});
  smoother.set_target_offset_dac_units(-3);
  EXPECT_EQ(smoother.get_current_offset_dac_units(), 0);
  EXPECT_EQ(smoother.get_target_green_voltage_dac_units(), 100);
  EXPECT_EQ(smoother.get_target_blue_voltage_dac_units(), 200);
}

TEST_F(TargetVoltageSmootherTest, TargetOffsetTooFewSteps) {
  TargetVoltageSmoother smoother(settings_, 8 /* adc_bits */, 8 /* dac_bits */);
  smoother.update_measurments({100, 200});
  smoother.set_target_offset_dac_units(-3);
  smoother.step();
  smoother.step();
  EXPECT_EQ(smoother.get_current_offset_dac_units(), -2);
  EXPECT_EQ(smoother.get_target_green_voltage_dac_units(), 98);
  EXPECT_EQ(smoother.get_target_blue_voltage_dac_units(), 202);
}

TEST_F(TargetVoltageSmootherTest, TargetOffsetReached) {
  TargetVoltageSmoother smoother(settings_, 8 /* adc_bits */, 8 /* dac_bits */);
  smoother.update_measurments({100, 200});
  smoother.set_target_offset_dac_units(-3);
  for (int i = 0; i < 10; ++i) {
    smoother.step();
  }
  EXPECT_EQ(smoother.get_current_offset_dac_units(), -3);
  EXPECT_EQ(smoother.get_target_green_voltage_dac_units(), 97);
  EXPECT_EQ(smoother.get_target_blue_voltage_dac_units(), 203);
}

TEST_F(TargetVoltageSmootherTest, TargetOffsetTooLargeCapped) {
  TargetVoltageSmoother smoother(settings_, 8 /* adc_bits */, 8 /* dac_bits */);
  smoother.update_measurments({100, 200});
  smoother.set_target_offset_dac_units(8);
  for (int i = 0; i < 10; ++i) {
    smoother.step();
  }
  EXPECT_EQ(smoother.get_current_offset_dac_units(),
            max_steering_magnitude_for_test);
  EXPECT_EQ(smoother.get_target_green_voltage_dac_units(),
            100 + max_steering_magnitude_for_test);
  EXPECT_EQ(smoother.get_target_blue_voltage_dac_units(),
            200 - max_steering_magnitude_for_test);
}

TEST_F(TargetVoltageSmootherTest, TargetOffsetSeveralChanges) {
  TargetVoltageSmoother smoother(settings_, 8 /* adc_bits */, 8 /* dac_bits */);
  smoother.update_measurments({100, 200});
  smoother.set_target_offset_dac_units(2);
  smoother.step();
  smoother.set_target_offset_dac_units(-3);
  smoother.step();
  smoother.step();
  EXPECT_EQ(smoother.get_current_offset_dac_units(), -1);
  EXPECT_EQ(smoother.get_target_green_voltage_dac_units(), 99);
  EXPECT_EQ(smoother.get_target_blue_voltage_dac_units(), 201);
}

TEST_F(TargetVoltageSmootherTest, DacResolutionHigher) {
  TargetVoltageSmoother smoother(settings_, 10 /* adc_bits */,
                                 12 /* dac_bits */);
  smoother.update_measurments({100, 200});
  smoother.set_target_offset_dac_units(3);
  smoother.step();
  smoother.step();
  EXPECT_EQ(smoother.get_current_offset_dac_units(), 2);
  EXPECT_EQ(smoother.get_target_green_voltage_dac_units(), 402);
  EXPECT_EQ(smoother.get_target_blue_voltage_dac_units(), 798);
}

TEST_F(TargetVoltageSmootherTest, DacResolutionLower) {
  TargetVoltageSmoother smoother(settings_, 8 /* adc_bits */, 6 /* dac_bits */);
  smoother.update_measurments({100, 200});
  smoother.set_target_offset_dac_units(3);
  smoother.step();
  smoother.step();
  EXPECT_EQ(smoother.get_current_offset_dac_units(), 2);
  EXPECT_EQ(smoother.get_target_green_voltage_dac_units(), 27);
  EXPECT_EQ(smoother.get_target_blue_voltage_dac_units(), 48);
}
}
