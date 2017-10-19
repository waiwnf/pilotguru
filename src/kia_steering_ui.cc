#include "ui/kia_steering_ui_main_window.h"
#include <QApplication>

#include <gflags/gflags.h>
#include <glog/logging.h>

DEFINE_string(can_interface, "slcan0", "");
DEFINE_string(arduino_tty, "", "");
DEFINE_string(kia_log_dir, ".", "");

int main(int argc, char *argv[]) {
  google::InitGoogleLogging(argv[0]);
  google::ParseCommandLineFlags(&argc, &argv, true);
  google::InstallFailureSignalHandler();

  QApplication a(argc, argv);

  pilotguru::kia::SteeringAngleHolderSettings settings;
  MainWindow w(FLAGS_can_interface, FLAGS_arduino_tty, settings,
               FLAGS_kia_log_dir);
  w.show();

  return a.exec();
}
