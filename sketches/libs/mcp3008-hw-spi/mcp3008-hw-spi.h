// Minimal library for reading from MCP3008 DAC using Arduino hardware SPI
// implementation.
// Adapted from https://github.com/adafruit/Adafruit_MCP3008

#ifndef PILOTGURU_SKETCHES_LIBS_MCP3008_HW_SPI_H_
#define PILOTGURU_SKETCHES_LIBS_MCP3008_HW_SPI_H_

#include <Arduino.h>
#include <SPI.h>

constexpr uint32_t MCP3008_SPI_MAX_FREQUENCY_5V = 3600000;

class Mcp3008HwSpi {
public:
  Mcp3008HwSpi(uint32_t spi_max_frequency, uint8_t cs_pin);
  void Setup() const;
  int16_t ReadSingleChannel(uint8_t channel) const;

private:
  const uint8_t cs_pin_;
  const SPISettings spi_settings_;
};

#endif // PILOTGURU_SKETCHES_LIBS_MCP3008_HW_SPI_H_
