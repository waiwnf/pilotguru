#include <ui_steering_nn/steering_nn_main_window.h>

#include <QApplication>

#include <gflags/gflags.h>
#include <zmq.hpp>

DEFINE_string(steering_prediction_socket, "ipc:///tmp/steering-predict", "");
DEFINE_string(can_interface, "slcan0", "");
DEFINE_string(arduino_tty, "", "");
DEFINE_string(kia_log_dir, ".", "");
DEFINE_int32(max_spoof_steering_torque, 5, "In DAC LSB units.");
DEFINE_double(torque_change_step, 0.2, "Step size for fine-grained internal "
                                       "spoof steering voltage changes. Can be "
                                       "less than 1.");

int main(int argc, char *argv[]) {
  google::InitGoogleLogging(argv[0]);
  google::ParseCommandLineFlags(&argc, &argv, true);
  google::InstallFailureSignalHandler();

  zmq::context_t context(1);
  zmq::socket_t socket(context, ZMQ_SUB);
  // Set the socket to only keep the most recent message and drop the older
  // ones. We are not interested in old steering angle predictions if a fresher
  // one is available.
  int conflate_on = 1;
  socket.setsockopt(ZMQ_CONFLATE, &conflate_on, sizeof(int));
  // Set a receive timeout to make sure the reading loop does not hang forever
  // on waits.
  int prediction_timeout_ms = 50;
  socket.setsockopt(ZMQ_RCVTIMEO, &prediction_timeout_ms, sizeof(int));
  socket.connect(FLAGS_steering_prediction_socket);
  // Subscribe to all the messages from the publisher.
  socket.setsockopt(ZMQ_SUBSCRIBE, "", 0);

  pilotguru::kia::SteeringAngleHolderSettings steering_controller_settings;
  steering_controller_settings.max_torque = FLAGS_max_spoof_steering_torque;
  steering_controller_settings.torque_change_step = FLAGS_torque_change_step;

  qRegisterMetaType<int16_t>("int16_t");
  qRegisterMetaType<double>("double");

  QApplication application(argc, argv);
  SteeringNNMainWindow main_window(FLAGS_can_interface, FLAGS_arduino_tty,
                                   steering_controller_settings, &socket,
                                   FLAGS_kia_log_dir);
  main_window.show();

  return application.exec();
}
