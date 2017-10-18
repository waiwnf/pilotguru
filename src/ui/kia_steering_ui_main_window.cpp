#include "kia_steering_ui_main_window.h"
#include <ui_kia_steering_ui_main_window.h>

#include <spoof-steering-serial-commands.h>

using pilotguru::Timestamped;
using pilotguru::TimestampedHistory;
using pilotguru::kia::KiaControlCommand;
using pilotguru::kia::SteeringAngle;
using pilotguru::kia::Velocity;

SteeringAngleReadThread::SteeringAngleReadThread(
    const TimestampedHistory<SteeringAngle> *values_history)
    : TimestampedValueReadThread<SteeringAngle>(values_history) {}

void SteeringAngleReadThread::ProcessValue(
    const Timestamped<SteeringAngle> &value) {
  emit SteeringAngleChanged(QString::number(value.data().angle_deci_degrees));
}

VelocityReadThread::VelocityReadThread(
    const TimestampedHistory<Velocity> *values_history)
    : TimestampedValueReadThread<Velocity>(values_history) {}

void VelocityReadThread::ProcessValue(const Timestamped<Velocity> &value) {
  emit VelocityChanged(QString::number(value.data().average_wheel_speed()));
}

SteeringTorqueOffsetReadThread::SteeringTorqueOffsetReadThread(
    const TimestampedHistory<std::string> *values_history)
    : TimestampedValueReadThread<std::string>(values_history) {}

void SteeringTorqueOffsetReadThread::ProcessValue(
    const Timestamped<std::string> &value) {
  KiaControlCommand command;
  const bool parse_result = pilotguru::kia::ParseSingleKiaControlCommand(
      value.data().c_str(), value.data().length(), &command);
  if (parse_result && (command.type == KiaControlCommand::STEER)) {
    emit SteeringTorqueChanged(QString::number(command.value));
  }
}

MainWindow::MainWindow(const std::string &can_interface,
                       const std::string &arduino_tty,
                       const pilotguru::kia::SteeringAngleHolderSettings
                           &steering_controller_settings,
                       QWidget *parent)
    : QMainWindow(parent), ui(new Ui::MainWindow),
      car_motion_data_(new pilotguru::kia::CarMotionData(10)),
      car_motion_data_updater_(new pilotguru::kia::CarMotionDataUpdater(
          car_motion_data_.get(), can_interface, {0x2B0, 0x4B0}, {1, 0})),
      arduino_command_channel_(
          new pilotguru::ArduinoCommandChannel(arduino_tty)),
      steering_controller_(new pilotguru::kia::SteeringAngleHolderController(
          &(car_motion_data_->steering_angles()),
          arduino_command_channel_.get(), steering_controller_settings)) {
  ui->setupUi(this);

  car_motion_data_updater_->start();

  steering_angle_read_thread_.reset(
      new SteeringAngleReadThread(&(car_motion_data_->steering_angles())));
  connect(steering_angle_read_thread_.get(),
          &SteeringAngleReadThread::SteeringAngleChanged, this,
          &MainWindow::OnSteeringAngleChanged);
  steering_angle_read_thread_->start();

  velocity_read_thread_.reset(
      new VelocityReadThread(&(car_motion_data_->velocities())));
  connect(velocity_read_thread_.get(), &VelocityReadThread::VelocityChanged,
          this, &MainWindow::OnVelocityChanged);
  velocity_read_thread_->start();

  steering_torque_offset_read_thread_.reset(new SteeringTorqueOffsetReadThread(
      &(arduino_command_channel_->CommandsHistory())));
  connect(steering_torque_offset_read_thread_.get(),
          &SteeringTorqueOffsetReadThread::SteeringTorqueChanged, this,
          &MainWindow::OnSteeringTorqueChanged);
  steering_torque_offset_read_thread_->start();

  connect(ui->steering_torque_send_button, &QPushButton::clicked, this,
          &MainWindow::SendSingleSteeringCommand);
  connect(ui->target_angle_send_button, &QPushButton::clicked, this,
          &MainWindow::SetTargetSteeringAngle);
}

MainWindow::~MainWindow() {
  steering_angle_read_thread_->RequestStop();
  velocity_read_thread_->RequestStop();
  steering_torque_offset_read_thread_->RequestStop();
  steering_angle_read_thread_->wait();
  velocity_read_thread_->wait();
  steering_torque_offset_read_thread_->wait();
  steering_controller_->Stop();
  car_motion_data_updater_->stop();
  delete ui;
}

void MainWindow::SendSingleSteeringCommand() {
  const std::string command_in_str =
      std::string(1, KiaControlCommand::STEER) +
      ui->steering_torque_in_field->text().toStdString();
  KiaControlCommand command;
  if (KiaControlCommand::TryParse(command_in_str.c_str(), &command)) {
    // Only try to send out the command if it parsed properly.
    arduino_command_channel_->SendCommand(command);
  }
}

void MainWindow::SetTargetSteeringAngle() {
  bool parse_success = false;
  const short target_angle =
      ui->target_angle_in_field->text().toShort(&parse_success);
  if (parse_success &&
      std::abs(target_angle) <=
          steering_controller_->settings().max_angle_amplitude) {
    steering_controller_->SetTargetAngle(target_angle);
  }
}

void MainWindow::OnSteeringAngleChanged(QString text) {
  ui->steering_angle_value_label->setText(text);
}

void MainWindow::OnVelocityChanged(QString text) {
  ui->velocity_value_label->setText(text);
}

void MainWindow::OnSteeringTorqueChanged(QString text) {
  ui->torque_offset_value_label->setText(text);
}
