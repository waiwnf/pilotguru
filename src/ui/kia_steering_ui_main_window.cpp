#include "kia_steering_ui_main_window.h"
#include <ui_kia_steering_ui_main_window.h>

#include <spoof-steering-serial-commands.h>

SteeringAngleReadThread::SteeringAngleReadThread(
    const pilotguru::kia::TimestampedHistory<pilotguru::kia::SteeringAngle>
        *values_history)
    : TimestampedValueReadThread<pilotguru::kia::SteeringAngle>(
          values_history) {}

void SteeringAngleReadThread::ProcessValue(
    const pilotguru::kia::Timestamped<pilotguru::kia::SteeringAngle> &value) {
  emit SteeringAngleChanged(QString::number(value.data().angle_deci_degrees));
}

VelocityReadThread::VelocityReadThread(
    const pilotguru::kia::TimestampedHistory<pilotguru::kia::Velocity>
        *values_history)
    : TimestampedValueReadThread<pilotguru::kia::Velocity>(values_history) {}

void VelocityReadThread::ProcessValue(
    const pilotguru::kia::Timestamped<pilotguru::kia::Velocity> &value) {
  emit VelocityChanged(QString::number(value.data().average_wheel_speed()));
}

MainWindow::MainWindow(const std::string &can_interface,
                       const std::string &arduino_tty, QWidget *parent)
    : QMainWindow(parent), ui(new Ui::MainWindow),
      car_motion_data_(new pilotguru::kia::CarMotionData(10)),
      car_motion_data_updater_(new pilotguru::kia::CarMotionDataUpdater(
          car_motion_data_.get(), can_interface, {0x2B0, 0x4B0}, {1, 0})),
      arduino_command_channel_(
          new pilotguru::ArduinoCommandChannel(arduino_tty)) {
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

  connect(ui->steering_torque_send_button, &QPushButton::clicked, this,
          &MainWindow::SendSteering);
}

MainWindow::~MainWindow() {
  steering_angle_read_thread_->RequestStop();
  velocity_read_thread_->RequestStop();
  steering_angle_read_thread_->wait();
  velocity_read_thread_->wait();
  car_motion_data_updater_->stop();
  delete ui;
}

void MainWindow::SendSteering() {
  using pilotguru::kia::KiaControlCommand;

  const std::string command_in_str =
      std::string(1, KiaControlCommand::STEER) +
      ui->steering_torque_in_field->text().toStdString();
  KiaControlCommand command;
  if (KiaControlCommand::TryParse(command_in_str.c_str(), &command)) {
    // Only try to send out the command if it parsed properly.
    arduino_command_channel_->SendCommand(command);
  }
}

void MainWindow::OnSteeringAngleChanged(QString text) {
  ui->steering_angle_value_label->setText(text);
}

void MainWindow::OnVelocityChanged(QString text) {
  ui->velocity_value_label->setText(text);
}
