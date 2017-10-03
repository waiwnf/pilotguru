#include <cstdlib>
#include <iostream>
#include <thread>

#include <gflags/gflags.h>
#include <glog/logging.h>

#include <linux/can.h>
#include <sys/ioctl.h>
#include <sys/socket.h>

#include <car/can.hpp>
#include <car/kia_can.hpp>

DEFINE_string(can_interface, "slcan0", "");

namespace {
void steeringAngleLogger(const pilotguru::kia::CarMotionData *motion_data) {
  CHECK_NOTNULL(motion_data);
  pilotguru::kia::Timestamped<pilotguru::kia::SteeringAngle> steering_instance{
      {}, {0, 0}};
  while (true) {
    steering_instance = motion_data->steering_angles().wait_get_next(
        steering_instance.timestamp());
    std::cout << "Steering angle: "
              << steering_instance.data().angle_deci_degrees << std::endl;
  }
}
}

int main(int argc, char *argv[]) {
  google::InitGoogleLogging(argv[0]);
  google::ParseCommandLineFlags(&argc, &argv, true);
  google::InstallFailureSignalHandler();

  // TODO library for stepwise force.
  // TODO sketch for specifying force.
  // TODO control closed loop logic.
  // TODO filtering whitelist as command line flag.
  // const int can_socket = pilotguru::connect_new_can_soket(
  //    FLAGS_can_interface.c_str(), {0x2B0, 0x4B0});
  const int can_socket =
      pilotguru::connect_new_can_soket(FLAGS_can_interface, {0x2B0, 0x4B0});
  CHECK_GE(can_socket, 0);

  pilotguru::kia::CarMotionData motion_data(10);
  std::thread angle_logging_thread(steeringAngleLogger, &motion_data);
  pilotguru::kia::CarMotionDataUpdater updater(
      &motion_data, FLAGS_can_interface, {0x2B0, 0x4B0}, {1, 0});
  updater.start();
  sleep(20);
  updater.stop();
  //  std::thread data_update_thread(motionUpdateLoop, can_socket,
  //  &motion_data);

  angle_logging_thread.detach();
  return EXIT_SUCCESS;
}
