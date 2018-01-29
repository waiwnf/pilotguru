#ifndef STEERINGNNMAINWINDOW_H
#define STEERINGNNMAINWINDOW_H

#include <QMainWindow>
#include <QThread>

#include <chrono>
#include <memory>

#include <glog/logging.h>
#include <zmq.hpp>

#include <car/arduino_comm.hpp>
#include <car/can.hpp>
#include <car/kia_can.hpp>
#include <car/kia_steering_angle_holder.hpp>
#include <io/timestamped_json_logger.hpp>
#include <nn_comm/nn_comm.hpp>
#include <ui/timestamped_history_qt_helpers.h>

namespace Ui {
class SteeringNNMainWindow;
}

constexpr char STEERING_COMMANDS_LOG_ROOT_ELEMENT[] = "steering_commands";
constexpr char STEERING_ANGLES_LOG_ROOT_ELEMENT[] = "steering_angles";

class SteeringPredictionReadThread
    : public TimestampedValueReadThread<
          pilotguru::SingleSteeringAnglePrediction> {
  Q_OBJECT
public:
  SteeringPredictionReadThread(
      const pilotguru::TimestampedHistory<
          pilotguru::SingleSteeringAnglePrediction> *values_history)
      : TimestampedValueReadThread<pilotguru::SingleSteeringAnglePrediction>(
            values_history) {}

  void ProcessValue(const pilotguru::Timestamped<
                    pilotguru::SingleSteeringAnglePrediction> &value) override {
    emit SteeringPredictionChanged(value.data().degrees);
  }

signals:
  void SteeringPredictionChanged(double angle_degrees);
};

class SteeringNNMainWindow : public QMainWindow {
  Q_OBJECT

public:
  explicit SteeringNNMainWindow(
      const std::string &can_interface, const std::string &arduino_tty,
      const pilotguru::kia::SteeringAngleHolderSettings
          &steering_controller_settings,
      zmq::socket_t *prediction_data_socket, const std::string &log_dir,
      QWidget *parent = 0);
  virtual ~SteeringNNMainWindow();

private:
  void OnSteeringAngleChanged(int16_t angle_deci_degrees);
  void OnVelocityChanged(QString text);
  void OnSteeringTorqueChanged(QString text);
  void OnSteeringPredictionChanged(double degrees);
  void PredictionUpdaterStart();
  void PredictionUpdaterStop();
  void SteeringStart();
  void SteeringStop();

  Ui::SteeringNNMainWindow *ui;

  // Car comms.
  std::unique_ptr<pilotguru::kia::CarMotionData> car_motion_data_;
  std::unique_ptr<pilotguru::kia::CarMotionDataUpdater>
      car_motion_data_updater_;
  std::unique_ptr<pilotguru::ArduinoCommandChannel> arduino_command_channel_;
  std::unique_ptr<pilotguru::kia::SteeringAngleHolderController>
      steering_controller_;

  // UI indicators update threads.
  std::unique_ptr<SteeringAngleReadThread> steering_angle_read_thread_;
  std::unique_ptr<VelocityReadThread> velocity_read_thread_;
  std::unique_ptr<SteeringTorqueOffsetReadThread>
      steering_torque_offset_read_thread_;
  std::unique_ptr<SteeringPredictionReadThread>
      steering_prediction_read_thread_;

  // Steering predictions module comms.
  std::unique_ptr<pilotguru::SingleSteeringAnglePredictionUpdater>
      prediction_updater_;

  // Logging.
  std::unique_ptr<
      pilotguru::TimestampedJsonLogger<pilotguru::kia::KiaControlCommand>>
      kia_commands_logger_;
  std::unique_ptr<
      pilotguru::TimestampedJsonLogger<pilotguru::kia::SteeringAngle>>
      steering_angles_logger_;
};

#endif // STEERINGNNMAINWINDOW_H
