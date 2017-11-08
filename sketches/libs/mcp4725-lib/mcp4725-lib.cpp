#include <mcp4725-lib.h>

#include <Wire.h>

bool SetMcp4725OutVoltageFastMode(uint8_t i2c_address, uint16_t value) {
  // Fail if the target value does not fit into the 12 bits of controller
  // resolution.
  if (value >= (1 << MCP4725_RESOLUTION_BITS)) {
    return false;
  }

  Wire.beginTransmission(i2c_address);
  // Highest 4 bits, padded with zeros.
  const uint8_t b1 = ((value >> 8) & 0xF);
  // Lowest 8 bits.
  const uint8_t b2 = (value & 0xFF);
  Wire.write(b1);
  Wire.write(b2);
  Wire.endTransmission();

  return true;
}
