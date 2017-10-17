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

namespace Ui {
class MainWindow;
}

// Common logic for a separate thread continuously picking new timestamped
// values from a queue and processing them.
// Uses QThread instead of C++ standard library for compatibility with the
// slots/signals framework used in the QT UI - directly updating UI elements
// from multiple threads is not supported by QT.
// This template usage must be restricted to the UI part of the program to avoid
// propagating QT dependencies any deeper to the rest of the code.
template <typename T> class TimestampedValueReadThread : public QThread {
public:
  // Does not take ownership of the pointer.
  TimestampedValueReadThread(
      const pilotguru::kia::TimestampedHistory<T> *values_history)
      : values_history_(values_history) {
    CHECK_NOTNULL(values_history_);
  }

  // Actual processing logic to be implemented by derivative classes.
  virtual void ProcessValue(const pilotguru::kia::Timestamped<T> &value) = 0;

  void run() override {
    // Need a timeout waiting for new value to be able to check must_run_
    // regularly.
    const std::chrono::microseconds wait_timeout = std::chrono::seconds(1);
    pilotguru::kia::Timestamped<T> value_instance{{}, {0, 0}};
    while (must_run_) {
      if (values_history_->wait_get_next(value_instance.timestamp(),
                                         &wait_timeout, &value_instance)) {
        ProcessValue(value_instance);
      }
    }
  }

  void RequestStop() { must_run_ = false; }

private:
  const pilotguru::kia::TimestampedHistory<T> *values_history_ = nullptr;
  bool must_run_ = true;
};

// Reads SteeringAngle values off the queue and formats them as text.
class SteeringAngleReadThread
    : public TimestampedValueReadThread<pilotguru::kia::SteeringAngle> {
  Q_OBJECT
public:
  SteeringAngleReadThread(const pilotguru::kia::TimestampedHistory<
                          pilotguru::kia::SteeringAngle> *values_history);
  void ProcessValue(
      const pilotguru::kia::Timestamped<pilotguru::kia::SteeringAngle> &value)
      override;

signals:
  void SteeringAngleChanged(QString text);
};

// Reads Velocity values off the queue and formats average wheel velocity as
// text.
class VelocityReadThread
    : public TimestampedValueReadThread<pilotguru::kia::Velocity> {
  Q_OBJECT
public:
  VelocityReadThread(const pilotguru::kia::TimestampedHistory<
                     pilotguru::kia::Velocity> *values_history);
  void ProcessValue(const pilotguru::kia::Timestamped<pilotguru::kia::Velocity>
                        &value) override;

signals:
  void VelocityChanged(QString text);
};

// Reads Velocity values off the queue and formats average wheel velocity as
// text.
class SteeringTorqueOffsetReadThread
    : public TimestampedValueReadThread<pilotguru::kia::KiaControlCommand> {
  Q_OBJECT
public:
  SteeringTorqueOffsetReadThread(
      const pilotguru::kia::TimestampedHistory<
          pilotguru::kia::KiaControlCommand> *values_history);
  void ProcessValue(const pilotguru::kia::Timestamped<
                    pilotguru::kia::KiaControlCommand> &value) override;

signals:
  void SteeringTorqueChanged(QString text);
};

class MainWindow : public QMainWindow {
  Q_OBJECT

public:
  explicit MainWindow(const std::string &can_interface,
                      const std::string &arduino_tty,
                      const pilotguru::kia::SteeringAngleHolderSettings
                          &steering_controller_settings,
                      QWidget *parent = 0);
  virtual ~MainWindow();

private:
  void SendSingleSteeringCommand();
  void SetTargetSteeringAngle();
  void OnSteeringAngleChanged(QString text);
  void OnVelocityChanged(QString text);
  void OnSteeringTorqueChanged(QString text);

  Ui::MainWindow *ui;

  std::unique_ptr<pilotguru::kia::CarMotionData> car_motion_data_;
  std::unique_ptr<pilotguru::kia::CarMotionDataUpdater>
      car_motion_data_updater_;
  std::unique_ptr<
      pilotguru::kia::TimestampedHistory<pilotguru::kia::KiaControlCommand>>
      steering_commands_history_;
  std::unique_ptr<pilotguru::ArduinoCommandChannel> arduino_command_channel_;
  std::unique_ptr<pilotguru::kia::SteeringAngleHolderController>
      steering_controller_;

  std::unique_ptr<SteeringAngleReadThread> steering_angle_read_thread_;
  std::unique_ptr<VelocityReadThread> velocity_read_thread_;
  std::unique_ptr<SteeringTorqueOffsetReadThread>
      steering_torque_offset_read_thread_;
};

#endif // MAINWINDOW_H
