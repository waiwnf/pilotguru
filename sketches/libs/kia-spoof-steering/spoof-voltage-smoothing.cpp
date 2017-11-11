#include <spoof-voltage-smoothing.h>

#include <stdio.h>

template <typename T> T min(T a, T b) { return a < b ? a : b; }

template <typename T> T max(T a, T b) { return a > b ? a : b; }

template <typename T> T constrain(T a, T low, T high) {
  return min(max(a, low), high);
}

int TargetVoltageSmoother::SmoothedVoltageState::make_string(
    char *buf, size_t buf_size) const {
  if (buf == nullptr || buf_size < 1) {
    return -1;
  }
  return snprintf(buf, buf_size - 1, "%d,%d,%d,%d",
                  smoothed_blue_voltage_adc_units,
                  smoothed_green_voltage_adc_units, current_offset_dac_units,
                  target_offset_dac_units);
}

namespace {
uint16_t smooth_voltage(uint16_t old_smooth_voltage, uint16_t new_voltage,
                        uint16_t voltage_update_hystheresis) {
  return ((max(old_smooth_voltage, new_voltage) -
           min(old_smooth_voltage, new_voltage)) > voltage_update_hystheresis)
             ? new_voltage
             : old_smooth_voltage;
}

uint16_t add_offset(uint16_t base_adc_units, int16_t offset_dac_units,
                    uint8_t adc_bits, uint8_t dac_bits) {
  // Convert base voltage from ADC to DAC units.
  uint16_t base_dac_units = base_adc_units;
  if (adc_bits > dac_bits) {
    base_dac_units >>= (adc_bits - dac_bits);
  } else if (adc_bits < dac_bits) {
    base_dac_units <<= (dac_bits - adc_bits);
  }

  if (offset_dac_units == 0) {
    return base_dac_units;
  } else if (offset_dac_units > 0) {
    return (UINT16_MAX - base_dac_units > offset_dac_units)
               ? base_dac_units + offset_dac_units
               : UINT16_MAX;
  } else { // offset_dac_units < 0
    return (base_dac_units > -offset_dac_units)
               ? base_dac_units + offset_dac_units
               : 0;
  }
}
}

TargetVoltageSmoother::TargetVoltageSmoother(
    const SteeringSpoofSettings &steering_spoof_settings, uint8_t adc_bits,
    uint8_t dac_bits)
    : steering_spoof_settings_(steering_spoof_settings), adc_bits_(adc_bits),
      dac_bits_(dac_bits) {}

void TargetVoltageSmoother::set_target_offset_dac_units(
    int16_t new_target_offset) {
  voltage_state_.target_offset_dac_units = constrain(
      new_target_offset,
      static_cast<int16_t>(-steering_spoof_settings_.max_steering_magnitude),
      steering_spoof_settings_.max_steering_magnitude);
  if (voltage_state_.target_offset_dac_units ==
      voltage_state_.current_offset_dac_units) {
    steps_spent_at_current_offset_ =
        min(steps_spent_at_current_offset_,
            steering_spoof_settings_.steps_per_adjustment_level);
  }
}

void TargetVoltageSmoother::step() {
  // Prevent overflow.
  if (steps_spent_at_current_offset_ < UINT16_MAX) {
    ++steps_spent_at_current_offset_;
  }
  if (voltage_state_.target_offset_dac_units !=
      voltage_state_.current_offset_dac_units) {
    // We are still in progress of adjusting towards the target offset.
    if (steps_spent_at_current_offset_ >=
        steering_spoof_settings_.steps_per_adjustment_level) {
      // Enough steps have been spent at the current offset, can move one unit
      // towards the target.
      voltage_state_.current_offset_dac_units +=
          (voltage_state_.target_offset_dac_units >
           voltage_state_.current_offset_dac_units)
              ? 1
              : -1;
      steps_spent_at_current_offset_ = 0;
    }
  } else if (voltage_state_.target_offset_dac_units != 0 &&
             steps_spent_at_current_offset_ >=
                 (steering_spoof_settings_.steps_at_target_level +
                  steering_spoof_settings_.steps_per_adjustment_level)) {
    // Enough time has been spent after reaching nonzero target offset, reset
    // the target to 0.
    voltage_state_.target_offset_dac_units = 0;
  }
}

void TargetVoltageSmoother::update_measurments(
    const InstantVoltageData &voltage_data) {
  voltage_state_.smoothed_blue_voltage_adc_units = smooth_voltage(
      voltage_state_.smoothed_blue_voltage_adc_units, voltage_data.blue_voltage,
      steering_spoof_settings_.voltage_update_hystheresis);
  voltage_state_.smoothed_green_voltage_adc_units =
      smooth_voltage(voltage_state_.smoothed_green_voltage_adc_units,
                     voltage_data.green_voltage,
                     steering_spoof_settings_.voltage_update_hystheresis);
}

uint16_t TargetVoltageSmoother::get_smoothed_blue_voltage_adc_units() const {
  return voltage_state_.smoothed_blue_voltage_adc_units;
}

uint16_t TargetVoltageSmoother::get_smoothed_green_voltage_adc_units() const {
  return voltage_state_.smoothed_green_voltage_adc_units;
}

uint16_t TargetVoltageSmoother::get_target_blue_voltage_dac_units() const {
  return add_offset(voltage_state_.smoothed_blue_voltage_adc_units,
                    -voltage_state_.current_offset_dac_units, adc_bits_,
                    dac_bits_);
}

uint16_t TargetVoltageSmoother::get_target_green_voltage_dac_units() const {
  return add_offset(voltage_state_.smoothed_green_voltage_adc_units,
                    voltage_state_.current_offset_dac_units, adc_bits_,
                    dac_bits_);
}

int16_t TargetVoltageSmoother::get_current_offset_dac_units() const {
  return voltage_state_.current_offset_dac_units;
}

int16_t TargetVoltageSmoother::get_target_offset_dac_units() const {
  return voltage_state_.current_offset_dac_units;
}

const TargetVoltageSmoother::SmoothedVoltageState &
TargetVoltageSmoother::get_voltage_state() const {
  return voltage_state_;
}
