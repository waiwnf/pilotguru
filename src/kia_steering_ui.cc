#include "ui/kia_steering_ui_main_window.h"
#include <QApplication>

#include <gflags/gflags.h>
#include <glog/logging.h>

DEFINE_string(can_interface, "slcan0", "");
DEFINE_string(arduino_tty, "", "");
DEFINE_string(steering_commands_log_json, "steering_commands.json", "");

int main(int argc, char *argv[]) {
  google::InitGoogleLogging(argv[0]);
  google::ParseCommandLineFlags(&argc, &argv, true);
  google::InstallFailureSignalHandler();

  QApplication a(argc, argv);

  pilotguru::kia::SteeringAngleHolderSettings settings;
  MainWindow w(FLAGS_can_interface, FLAGS_arduino_tty, settings,
               FLAGS_steering_commands_log_json);
  w.show();

  return a.exec();
}
