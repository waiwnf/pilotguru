#ifndef MAINWINDOW_H
#define MAINWINDOW_H

#include <QMainWindow>
#include <QThread>

#include <chrono>
#include <memory>

#include <glog/logging.h>

#include <car/arduino_comm.hpp>
#include <car/can.hpp>
#include <car/kia_can.hpp>
#include <car/kia_steering_angle_holder.hpp>
#include <io/timestamped_json_logger.hpp>
#include <ui/timestamped_history_qt_helpers.h>

namespace Ui {
class MainWindow;
}

class MainWindow : public QMainWindow {
  Q_OBJECT

public:
  explicit MainWindow(const std::string &can_interface,
                      const std::string &arduino_tty,
                      const pilotguru::kia::SteeringAngleHolderSettings
                          &steering_controller_settings,
                      const std::string &log_dir, QWidget *parent = 0);
  virtual ~MainWindow();

private:
  void SendSingleSteeringCommand();
  void SetTargetSteeringAngleFromInputField();
  void ClearTargetSteeringAngle();
  void TurnLeft();
  void TurnRight();
  void OnSteeringAngleChanged(int16_t angle_deci_degrees);
  void OnVelocityChanged(QString text);
  void OnSteeringTorqueChanged(QString text);

  void SetTargetSteeringAngle(double target_angle_degrees);
  void ShiftTargetSteeringAngle(double target_angle_shift_degrees);

  Ui::MainWindow *ui;

  std::unique_ptr<pilotguru::kia::CarMotionData> car_motion_data_;
  std::unique_ptr<pilotguru::kia::CarMotionDataUpdater>
      car_motion_data_updater_;
  std::unique_ptr<pilotguru::ArduinoCommandChannel> arduino_command_channel_;
  std::unique_ptr<pilotguru::kia::SteeringAngleHolderController>
      steering_controller_;

  std::unique_ptr<SteeringAngleReadThread> steering_angle_read_thread_;
  std::unique_ptr<VelocityReadThread> velocity_read_thread_;
  std::unique_ptr<SteeringTorqueOffsetReadThread>
      steering_torque_offset_read_thread_;

  std::unique_ptr<
      pilotguru::TimestampedJsonLogger<pilotguru::kia::KiaControlCommand>>
      kia_commands_logger_;
  std::unique_ptr<
      pilotguru::TimestampedJsonLogger<pilotguru::kia::SteeringAngle>>
      steering_angles_logger_;
};

#endif // MAINWINDOW_H
