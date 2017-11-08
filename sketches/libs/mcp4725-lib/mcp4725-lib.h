// Library for seeting output voltage level on MCP4725 12-bit DAC.
// Datasheet: https://www.sparkfun.com/datasheets/BreakoutBoards/MCP4725.pdf

#ifndef PILOTGURU_SKETCHES_LIBS_MCP4725_LIB_H_
#define PILOTGURU_SKETCHES_LIBS_MCP4725_LIB_H_

#include <stdint.h>

constexpr uint32_t MCP4725_I2C_FREQUENCY = 400000;
constexpr uint8_t MCP4725_RESOLUTION_BITS = 12;

// Change output voltage level in fast mode (3 total bytes sent to device,
// including transaction start byte; EEPROM is not changed).
bool SetMcp4725OutVoltageFastMode(uint8_t i2c_address, uint16_t value);

#endif // PILOTGURU_SKETCHES_LIBS_MCP4725_LIB_H_
