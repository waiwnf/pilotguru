#ifndef SKETCHES_LIBS_KIA_SPOOF_STEERING_LIB_SPOOF_VOLTAGE_SMOOTHING_H_
#define SKETCHES_LIBS_KIA_SPOOF_STEERING_LIB_SPOOF_VOLTAGE_SMOOTHING_H_

#include <stddef.h>
#include <stdint.h>

struct InstantVoltageData {
  uint16_t green_voltage, blue_voltage;
};

// Voltage measurement postprocessing logic.
// Because PCF8591 is only 8 bit for both AD and DA conversions, even 1-bit
// jitter is noticeable by the power steering ECU when forwarding the signal
// from the torque sensors. To reduce the jitter, we use two tricks:
// 1. Maintain a running average of the torque sensor voltage measurements
//    instead of using only one most recent measurement.
// 2. Change the output voltage level only if the input running average moves
//    away by X from the current output level ("hystheresis").

// Use 2^averaging_shift most recent measurements for the running average of the
// torque sensor voltage. This way the running average is simply a bit shift of
// the sum of the individual measurements.
template <uint8_t averaging_shift> class HistoricVoltageData {
public:
  void take_measurement(const InstantVoltageData &voltage_data) {
    voltage_buffer_idx = (voltage_buffer_idx + 1) % buffer_size;
    update_voltage_running_total(green_voltage_readings + voltage_buffer_idx,
                                 &total_green_voltage,
                                 voltage_data.green_voltage);
    update_voltage_running_total(blue_voltage_readings + voltage_buffer_idx,
                                 &total_blue_voltage,
                                 voltage_data.blue_voltage);
  }

  uint16_t get_avg_green_voltage() const {
    return (total_green_voltage >> averaging_shift);
  }

  uint16_t get_avg_blue_voltage() const {
    return (total_blue_voltage >> averaging_shift);
  }

  InstantVoltageData get_avg_voltage() const {
    return {get_avg_green_voltage(), get_avg_blue_voltage()};
  }

  uint16_t get_latest_green_voltage() const {
    return green_voltage_readings[voltage_buffer_idx];
  }

  uint16_t get_latest_blue_voltage() const {
    return blue_voltage_readings[voltage_buffer_idx];
  }

  size_t get_buffer_size() const { return buffer_size; }

private:
  static void update_voltage_running_total(uint16_t *buffer_item,
                                           uint32_t *running_total,
                                           uint16_t new_value) {
    (*running_total) -= *buffer_item;
    *buffer_item = new_value;
    (*running_total) += *buffer_item;
  }

  static constexpr size_t buffer_size = (((size_t)1) << averaging_shift);
  // Individual torque sensor voltage measurement buffers, necessary for
  // computing the running averages.
  uint16_t green_voltage_readings[buffer_size];
  uint16_t blue_voltage_readings[buffer_size];
  // Wraparound index of which sensor voltage measurement buffer element to use
  // next.
  size_t voltage_buffer_idx = 0;
  // Running sums of the individual voltage measurement buffers.
  uint32_t total_green_voltage = 0, total_blue_voltage = 0;
};

struct SteeringSpoofSettings {
  // Maximum extra spoof torque voltage signal for a turn command, in LSB units
  // of the DAC module.
  int16_t max_steering_magnitude = 0;

  // The spoof torque voltage is changed gradually in 1 LSB increments to avoid
  // sharp voltage jumps that the ECU may perceive as sensor faults. Spend this
  // many loop() cycles between consecutive voltage changes.
  uint16_t steps_per_adjustment_level = 1;

  // Spend this many loop() cycles at target spoof torque level, once it has
  // been reached with consecutive 1 LSB increments (with
  // steps_per_adjustment_level between each increment).
  // After this many loop() cycles pass, the spoof torque extra voltage is
  // gradually reduced back to zero.
  uint16_t steps_at_target_level = 1;

  // Only change the output voltage when the input running average is more than
  // voltage_update_hystheresis from the current output.
  uint16_t voltage_update_hystheresis = 0;
};

// Logic for gradually adjusting the spoof steering voltage offset towards a
// given target value, so as not to trigger the voltage discontinuity detector
// in the power steering ECU.
class TargetVoltageSmoother {
public:
  struct SmoothedVoltageState {
    // Hystheresis-smoothed historical averages.
    uint16_t smoothed_blue_voltage_adc_units = 0;
    uint16_t smoothed_green_voltage_adc_units = 0;
    // Effective torque voltage offset to be applied to the hystheresis-smoothed
    // historical values at current step.
    int16_t current_offset_dac_units = 0;
    // Target voltage offset to move to over time, spending a few steps on every
    // intermediate voltage point.
    int16_t target_offset_dac_units = 0;

    int make_string(char *buf, size_t buf_size) const;
  };

  // Bit resolution of the torque voltage ADC sensor and spoof voltage DAC must
  // be specified explicitly. Internal state is in ADC units for the current
  // sensors voltage and DAC units for the spoof steering offset. The units
  // suitable for each member helper are specified in the helper name.
  TargetVoltageSmoother(const SteeringSpoofSettings &steering_spoof_settings,
                        uint8_t adc_bits, uint8_t dac_bits);

  void set_target_offset_dac_units(int16_t new_target_offset);
  void step();
  void update_measurments(const InstantVoltageData &voltage_data_adc_units);
  uint16_t get_smoothed_blue_voltage_adc_units() const;
  uint16_t get_smoothed_green_voltage_adc_units() const;
  uint16_t get_target_blue_voltage_dac_units() const;
  uint16_t get_target_green_voltage_dac_units() const;
  int16_t get_current_offset_dac_units() const;
  int16_t get_target_offset_dac_units() const;
  const SmoothedVoltageState &get_voltage_state() const;

private:
  SmoothedVoltageState voltage_state_;
  uint16_t steps_spent_at_current_offset_ = 0;

  const SteeringSpoofSettings &steering_spoof_settings_;
  const uint8_t adc_bits_, dac_bits_;
};

#endif // SKETCHES_LIBS_KIA_SPOOF_STEERING_LIB_SPOOF_VOLTAGE_SMOOTHING_H_
