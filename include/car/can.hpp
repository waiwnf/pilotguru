#ifndef PILOTGURU_CAR_CAN_HPP_
#define PILOTGURU_CAR_CAN_HPP_

// Generic helpers for working with CAN bus frames.

#include <cstdint>
#include <string>

#include <linux/can.h>

namespace pilotguru {

constexpr uint32_t CAN_ID_MASK = 0x7FF;

int connect_new_can_soket(const std::string& interface_name);
std::string can_frame_payload_to_hex_string(const can_frame& frame);

}

#endif