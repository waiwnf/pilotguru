#include "ui/kia_steering_ui_main_window.h"
#include <QApplication>

#include <gflags/gflags.h>
#include <glog/logging.h>

DEFINE_string(can_interface, "slcan0", "");

int main(int argc, char *argv[])
{
  google::InitGoogleLogging(argv[0]);
  google::ParseCommandLineFlags(&argc, &argv, true);
  google::InstallFailureSignalHandler();

  QApplication a(argc, argv);
  MainWindow w(FLAGS_can_interface);
  w.show();

  return a.exec();
}
