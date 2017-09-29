#ifndef PILOTGURU_CAR_KIA_CAN_HPP_
#define PILOTGURU_CAR_KIA_CAN_HPP_

// Helpers for communicating over KIA Cee'd C-CAN bus.

#include <cstdint>

namespace pilotguru {
namespace kia {

constexpr uint32_t STEERING_WHEEL_ANGLE_CAN_ID = 0x2B0;
constexpr uint32_t VELOCITY_CAN_ID = 0x4B0;

int16_t parse_can_int16(uint8_t* can_bytes);
}
}

#endif