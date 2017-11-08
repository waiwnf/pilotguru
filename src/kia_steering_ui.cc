#include "ui/kia_steering_ui_main_window.h"
#include <QApplication>

#include <gflags/gflags.h>
#include <glog/logging.h>

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

  CHECK_GT(FLAGS_max_spoof_steering_torque, 0);
  CHECK_LE(FLAGS_max_spoof_steering_torque, UINT16_MAX);
  CHECK_GT(FLAGS_torque_change_step, 0);

  pilotguru::kia::SteeringAngleHolderSettings settings;
  settings.max_torque = FLAGS_max_spoof_steering_torque;
  settings.torque_change_step = FLAGS_torque_change_step;

  QApplication a(argc, argv);
  MainWindow w(FLAGS_can_interface, FLAGS_arduino_tty, settings,
               FLAGS_kia_log_dir);
  w.show();

  return a.exec();
}
