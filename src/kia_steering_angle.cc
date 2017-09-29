#include <cstdlib>

#include <gflags/gflags.h>
#include <glog/logging.h>

#include <linux/can.h>

#include <car/can.hpp>
#include <car/kia_can.hpp>

DEFINE_string(can_interface, "slcan0", "");

int main(int argc, char *argv[]) {
  google::InitGoogleLogging(argv[0]);
  google::ParseCommandLineFlags(&argc, &argv, true);
  google::InstallFailureSignalHandler();

  const int can_socket =
      pilotguru::connect_new_can_soket(FLAGS_can_interface.c_str());
  CHECK_GE(can_socket, 0);

  while (true) {
    can_frame frame;
    const int nbytes = read(can_socket, &frame, sizeof(struct can_frame));
    CHECK_GE(nbytes, 0);
    CHECK_EQ(nbytes, sizeof(can_frame));
    LOG(INFO) << "CAN ID: " << frame.can_id << "; payload: "
              << pilotguru::can_frame_payload_to_hex_string(frame);

    if ((frame.can_id & pilotguru::CAN_ID_MASK) ==
        pilotguru::kia::STEERING_WHEEL_ANGLE_CAN_ID) {
      // TODO factor out 5 bytes payload size to a constant.
      CHECK_EQ(frame.can_dlc, 5);
      LOG(INFO) << "Steering wheel angle: "
                << pilotguru::kia::parse_can_int16(frame.data);
    }
  }

  return EXIT_SUCCESS;
}