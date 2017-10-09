#ifndef MAINWINDOW_H
#define MAINWINDOW_H

#include <QMainWindow>
#include <memory>
#include <mutex>
#include <thread>

#include <car/arduino_comm.hpp>
#include <car/can.hpp>
#include <car/kia_can.hpp>

namespace Ui {
class MainWindow;
}

class MainWindow : public QMainWindow {
  Q_OBJECT

public:
  explicit MainWindow(const std::string &can_interface,
                      const std::string &arduino_tty, QWidget *parent = 0);
  virtual ~MainWindow();

private:
  void startMonitoring();
  void stopMonitoring();
  void sendSteering();

  Ui::MainWindow *ui;

  std::unique_ptr<pilotguru::kia::CarMotionData> car_motion_data_;
  std::unique_ptr<pilotguru::kia::CarMotionDataUpdater>
      car_motion_data_updater_;

  std::mutex ui_elements_mutex_;

  std::unique_ptr<pilotguru::ArduinoCommandChannel> arduino_command_channel_;
  std::unique_ptr<std::thread> steering_angle_read_thread_;
  std::unique_ptr<std::thread> velocity_read_thread_;
};

#endif // MAINWINDOW_H
