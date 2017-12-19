#include <car/can.hpp>

#include <algorithm>
#include <cstring>
#include <limits>

#include <net/if.h>
#include <sys/ioctl.h>
#include <sys/socket.h>

#include <linux/can/raw.h>

#include <glog/logging.h>

namespace pilotguru {

int connect_new_can_soket(const std::string &interface_name,
                          const std::vector<canid_t> &accepted_ids) {
  constexpr int CONNECTION_FAILURE = -1;

  int can_socket = socket(PF_CAN, SOCK_RAW, CAN_RAW);
  if (socket < 0) {
    return CONNECTION_FAILURE;
  }

  if (!accepted_ids.empty()) {
    can_filter accepted_ids_filter[] = {make_can_filter(accepted_ids)};
    setsockopt(can_socket, SOL_CAN_RAW, CAN_RAW_FILTER, &accepted_ids_filter,
               sizeof(accepted_ids_filter));
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

bool try_parse_can_frame(const std::string &can_frame_string,
                         can_frame *result) {
  constexpr char kBytesSeparator = ' ';

  if (result == nullptr) {
    return false;
  }

  // CAN frame ID may be up to 29 bits, soup to 8 characters hex-encoded.
  const auto frame_id_end = std::find(can_frame_string.begin(),
                                      can_frame_string.end(), kBytesSeparator);
  const std::string frame_id_str(can_frame_string.begin(), frame_id_end);
  const int id_filled = sscanf(frame_id_str.c_str(), "%X", &(result->can_id));
  if (id_filled != 1) {
    return false;
  }

  // Payload decoding.
  result->can_dlc = 0;
  // Always expect exactly 2 characters for every encoded payload byte.
  constexpr size_t BYTE_HEX_ENCODED_LENGTH = 2;
  // C string to fill for parsing.
  char payload_byte_string[BYTE_HEX_ENCODED_LENGTH + 1];
  size_t payload_byte_end_idx = frame_id_end - can_frame_string.begin();
  while (payload_byte_end_idx < can_frame_string.length()) {
    // Make sure there is exactly one separator character between encoded
    // payload bytes.
    if (can_frame_string[payload_byte_end_idx] != kBytesSeparator) {
      return false;
    }
    // Move indices to the next encoded byte.
    const size_t payload_byte_start_idx = payload_byte_end_idx + 1;
    payload_byte_end_idx = payload_byte_start_idx + BYTE_HEX_ENCODED_LENGTH;
    // There may be a trailing space in the input
    if (payload_byte_start_idx >= can_frame_string.length()) {
      break;
    }
    // Make sure we are still within the input string with the new indices.
    if (payload_byte_end_idx >= can_frame_string.length()) {
      return false;
    }
    // Make sure we are not overfilling the payload.
    if (result->can_dlc >= CAN_MAX_DLEN) {
      return false;
    }
    // Copy the encoded byte to the C string.
    for (size_t i = 0; i < BYTE_HEX_ENCODED_LENGTH; ++i) {
      payload_byte_string[i] = can_frame_string[payload_byte_start_idx + i];
    }
    payload_byte_string[BYTE_HEX_ENCODED_LENGTH] = '\0';
    // Parse the encoded byte, check success.
    const int payload_filled =
        sscanf(payload_byte_string, "%hhX", &(result->data[result->can_dlc]));
    if (payload_filled != 1) {
      return false;
    }
    ++(result->can_dlc);
  }

  return true;
}

can_filter make_can_filter(const std::vector<canid_t> &accepted_ids) {
  if (accepted_ids.empty()) {
    return {0, 0};
  }
  canid_t filter = accepted_ids.front();
  canid_t mask = std::numeric_limits<canid_t>::max();

  for (const uint32_t id : accepted_ids) {
    // Mask only retains the bits that match in filter and id.
    mask &= (filter ^ ~id);
    // To get a 'canonical' filter representation, independent of the order of
    // ids in the input, zero out all the bits in the filter that are not all
    // equal among the acceted ids.
    filter &= id;
  }
  // TODO generalize this to support 29 bits too.
  return {filter & 0x7FF, mask & 0x7FF};
}
}
