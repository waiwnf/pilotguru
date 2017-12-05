#include <Wire.h>

#include <kia-spoof-steering.h>
#include <mcp3008-hw-spi.h>
#include <mcp4725-lib.h>
#include <spoof-steering-serial-commands.h>
#include <spoof-voltage-smoothing.h>

#define STEPS_ECHO_EVERY 1000

// DAC settings.
constexpr uint8_t MCP4725_DAC_BLUE = 0x62;
constexpr uint8_t MCP4725_DAC_GREEN = 0x63;

// ADC settings.
constexpr uint8_t MCP3008_ADC_CHANNEL_BLUE = 7;
constexpr uint8_t MCP3008_ADC_CHANNEL_GREEN = 3;
constexpr uint8_t MCP3008_ADC_CS_PIN = 10;

// Preallocate everything
SteeringSpoofSettings steering_spoof_settings;
HistoricVoltageData<4> historic_voltage_data;
InstantVoltageData voltage_measurement, historic_avg_voltage;
TargetVoltageSmoother voltage_smoother(steering_spoof_settings,
                                       MCP3008_RESOLUTION_BITS,
                                       MCP4725_RESOLUTION_BITS);
pilotguru::kia::KiaControlCommandProcessor command_processor;
pilotguru::kia::KiaControlCommand control_command;
Mcp3008HwSpi mcp3008_adc(MCP3008_SPI_MAX_FREQUENCY_5V, MCP3008_ADC_CS_PIN);

constexpr size_t int16_max_chars = 6; // 5 digits + sign;
// Adding separating comma for every number and the terminating null character.
constexpr size_t voltage_state_buffer_size = (int16_max_chars + 1) * 4 + 1;
// Adding space for an extra first 'v' tag character to the buffer size.
char voltage_state_string[voltage_state_buffer_size + 1];

bool is_echo_on = false;
int step_idx = 0;

void TakeVoltageMeasurement() {
  voltage_measurement.green_voltage =
      mcp3008_adc.ReadSingleChannel(MCP3008_ADC_CHANNEL_GREEN);
  voltage_measurement.blue_voltage =
      mcp3008_adc.ReadSingleChannel(MCP3008_ADC_CHANNEL_BLUE);
  historic_voltage_data.take_measurement(voltage_measurement);
}

void setup() {
  steering_spoof_settings.max_steering_magnitude = 80;
  steering_spoof_settings.steps_per_adjustment_level = 2;
  steering_spoof_settings.steps_at_target_level = 300;
  steering_spoof_settings.voltage_update_hystheresis = 1;

  voltage_state_string[0] = pilotguru::kia::VOLTAGE_REPORT_TAG;

  Serial.begin(115200);

  Wire.setClock(MCP4725_I2C_FREQUENCY);
  mcp3008_adc.Setup();

  // Warm up the voltage buffers.
  for (size_t i = 0; i < historic_voltage_data.get_buffer_size(); ++i) {
    TakeVoltageMeasurement();
  }
}

void loop() {
  TakeVoltageMeasurement();
  historic_avg_voltage = historic_voltage_data.get_avg_voltage();
  voltage_smoother.update_measurments(historic_avg_voltage);

  HandleCommandProcessorState(&command_processor, &control_command,
                              &voltage_smoother, &is_echo_on);
  voltage_smoother.step();

  step_idx = (step_idx + 1) % STEPS_ECHO_EVERY;
  if (step_idx == 0 && is_echo_on) {
    SerialWriteCurrentSpoofVoltages(voltage_smoother, voltage_state_string,
                                    voltage_state_buffer_size);
  }

  SetMcp4725OutVoltageFastMode(MCP4725_DAC_BLUE,
                               voltage_smoother.get_target_blue_voltage_dac_units());
  SetMcp4725OutVoltageFastMode(
      MCP4725_DAC_GREEN, voltage_smoother.get_target_green_voltage_dac_units());
}

void serialEvent() { ProcessAvailableSerialBuffer(&command_processor); }
