#include "mcp3008-hw-spi.h"

Mcp3008HwSpi::Mcp3008HwSpi(uint32_t spi_max_frequency, uint8_t cs_pin)
    : cs_pin_(cs_pin), spi_settings_(spi_max_frequency, MSBFIRST, SPI_MODE0) {}

void Mcp3008HwSpi::Setup() const {
  pinMode(cs_pin_, OUTPUT);
  digitalWrite(cs_pin_, HIGH);
  SPI.begin();
}

int16_t Mcp3008HwSpi::ReadSingleChannel(uint8_t channel) const {
  if (channel > 7) {
    return -1;
  }

  const uint8_t command = ((0x01 << 7) | // start bit
                           (0x01 << 6) | // single or differential
                           ((channel & 0x07) << 3));

  SPI.beginTransaction(spi_settings_);
  digitalWrite(cs_pin_, LOW);
  const uint8_t b0 = SPI.transfer(command);
  const uint8_t b1 = SPI.transfer(0x00);
  const uint8_t b2 = SPI.transfer(0x00);
  digitalWrite(cs_pin_, HIGH);
  SPI.endTransaction();

  return 0x3FF & ((b0 & 0x01) << 9 | (b1 & 0xFF) << 1 | (b2 & 0x80) >> 7);
}