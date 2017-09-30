#include <cstdlib>

#include <gflags/gflags.h>
#include <glog/logging.h>

#include <linux/can.h>
#include <sys/ioctl.h>
#include <sys/socket.h>

#include <car/can.hpp>
#include <car/kia_can.hpp>

DEFINE_string(can_interface, "slcan0", "");

int main(int argc, char *argv[]) {
  google::InitGoogleLogging(argv[0]);
  google::ParseCommandLineFlags(&argc, &argv, true);
  google::InstallFailureSignalHandler();

  // TODO library for stepwise force.
  // TODO sketch for specifying force.
  // TODO control closed loop logic.
  // TODO filtering whitelist as command line flag.
  const int can_socket = pilotguru::connect_new_can_soket(
      FLAGS_can_interface.c_str(), {0x2B0, 0x4B0});
  CHECK_GE(can_socket, 0);

  pilotguru::kia::CarMotionData motion_data(10);

  while (true) {
    can_frame frame;
    const int nbytes = read(can_socket, &frame, sizeof(struct can_frame));
    timeval tv;
    ioctl(can_socket, SIOCGSTAMP, &tv);
    CHECK_GE(nbytes, 0);
    CHECK_EQ(nbytes, sizeof(can_frame));
    LOG(INFO) << "CAN ID: " << frame.can_id << "; payload: "
              << pilotguru::can_frame_payload_to_hex_string(frame);

    if ((frame.can_id & pilotguru::CAN_ID_MASK_11_BIT) ==
        pilotguru::kia::STEERING_WHEEL_ANGLE_CAN_ID) {
      CHECK_EQ(frame.can_dlc,
               pilotguru::kia::STEERING_WHEEL_ANGLE_FRAME_PAYLOAD_SIZE);
      LOG(INFO) << "Steering wheel angle: "
                << pilotguru::kia::parse_can_int16(frame.data);
    }

    motion_data.update(frame, tv);
  }

  return EXIT_SUCCESS;
}