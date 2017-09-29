#include <car/can.hpp>

#include <cstring>

#include <net/if.h>
#include <sys/ioctl.h>
#include <sys/socket.h>

namespace pilotguru {

int connect_new_can_soket(const std::string &interface_name) {
  constexpr int CONNECTION_FAILURE = -1;

  int can_socket = socket(PF_CAN, SOCK_RAW, CAN_RAW);
  if (socket < 0) {
    return CONNECTION_FAILURE;
  }

  // Resolve CAN network interface id by name.
  ifreq interface_request;
  if (interface_name.size() >= IFNAMSIZ) {
    return CONNECTION_FAILURE;
  }
  strcpy(interface_request.ifr_name, interface_name.c_str());
  const int ioctl_result = ioctl(can_socket, SIOCGIFINDEX, &interface_request);
  if (ioctl_result < 0) {
    return CONNECTION_FAILURE;
  }

  // Bind the socket to the network interface.
  sockaddr_can can_socket_address;
  can_socket_address.can_family = AF_CAN;
  can_socket_address.can_ifindex = interface_request.ifr_ifindex;
  const int bind_result = bind(can_socket, (sockaddr *)&can_socket_address,
                               sizeof(can_socket_address));
  if (bind_result < 0) {
    return CONNECTION_FAILURE;
  }

  return can_socket;
}

std::string can_frame_payload_to_hex_string(const can_frame &frame) {
  char frame_data_hex_string[2 * CAN_MAX_DLEN + 1];
  for (uint16_t idx = 0; idx < frame.can_dlc; ++idx) {
    sprintf(frame_data_hex_string + idx * 2, "%02X", frame.data[idx]);
  }
  return std::string(frame_data_hex_string);
}
}