#include <car/kia_can.hpp>

#include <arpa/inet.h>
#include <glog/logging.h>

namespace pilotguru {
namespace kia {

int16_t parse_can_int16(uint8_t *can_bytes) {
  CHECK_NOTNULL(can_bytes);

  // CAN frames use little endian format. While on x86 we can simply memcpy()
  // the two bytes to an int16, on big-endian architectures that would produce
  // wrong results. So first convert the data to big endian (network format),
  // then use ntohs() to convert from network to host format.
  uint8_t big_endian_bytes[2];
  big_endian_bytes[0] = can_bytes[1];
  big_endian_bytes[1] = can_bytes[0];
  uint16_t value_unsigned_big_endian = 0;
  memcpy(&value_unsigned_big_endian, big_endian_bytes, 2);
  const uint16_t result_unsigned = ntohs(value_unsigned_big_endian);
  return *reinterpret_cast<const int16_t *>(&result_unsigned);
}
}
}