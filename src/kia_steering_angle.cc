#include <cstdlib>

#include <gflags/gflags.h>
#include <glog/logging.h>

#include <linux/can.h>
#include <linux/can/raw.h>

#include <arpa/inet.h>
#include <net/if.h>
#include <sys/ioctl.h>
#include <sys/socket.h>

DEFINE_string(can_interface, "slcan0", "");

int main(int argc, char *argv[]) {
  google::InitGoogleLogging(argv[0]);
  google::ParseCommandLineFlags(&argc, &argv, true);
  google::InstallFailureSignalHandler();

  int can_socket = socket(PF_CAN, SOCK_RAW, CAN_RAW);
  CHECK_GE(can_socket, 0);

  // Resolve CAN network interface id by name.
  ifreq interface_request;
  // TODO check flag value length here.
  strcpy(interface_request.ifr_name, FLAGS_can_interface.c_str());
  const int ioctl_result = ioctl(can_socket, SIOCGIFINDEX, &interface_request);
  CHECK_GE(ioctl_result, 0);

  // Bind the socket to the network interface.
  sockaddr_can can_socket_address;
  can_socket_address.can_family = AF_CAN;
  can_socket_address.can_ifindex = interface_request.ifr_ifindex;
  bind(can_socket, (sockaddr *)&can_socket_address, sizeof(can_socket_address));

  while (true) {
    can_frame frame;
    const int nbytes = read(can_socket, &frame, sizeof(struct can_frame));
    CHECK_GE(nbytes, 0);
    CHECK_EQ(nbytes, sizeof(can_frame));
    char frame_data_hex_string[2 * CAN_MAX_DLEN + 1];
    for (uint16_t idx = 0; idx < frame.can_dlc; ++idx) {
      sprintf(frame_data_hex_string + idx * 2, "%02X", frame.data[idx]);
    }
    LOG(INFO) << "CAN ID: " << frame.can_id
              << "; payload: " << frame_data_hex_string;

    // TODO factor out steering angle CAN frame id to a constant. 
    if ((frame.can_id & 0x7FF) == 0x2B0) {
      // TODO factor out 5 bytes payload size to a constant.
      CHECK_EQ(frame.can_dlc, 5);
      // CAN frames use little endian format. While on x86 we can simply
      // memcpy() the two bytes to an int16, on big-endian architectures that
      // would produce wrong results. So first convert the data to big
      // endian (network format), then use ntohs() to convert from network to
      // host format.
      // TODO factor out parsing the CAN data to a helper and reuse also for
      // velocity.
      uint8_t uint16_big_endian_bytes[2];
      uint16_big_endian_bytes[0] = frame.data[1];
      uint16_big_endian_bytes[1] = frame.data[0];
      uint16_t angle_unsigned_big_endian = 0;
      memcpy(&angle_unsigned_big_endian, uint16_big_endian_bytes, 2);
      const uint16_t angle_unsigned = ntohs(angle_unsigned_big_endian);
      LOG(INFO) << "Steering wheel angle: "
                << *reinterpret_cast<const int16_t *>(&angle_unsigned);
    }
  }

  return EXIT_SUCCESS;
}