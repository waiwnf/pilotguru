#include "kia_steering_ui_main_window.h"
#include <io/kia_json_loggers.hpp>
#include <ui_kia_steering_ui_main_window.h>

#include <spoof-steering-serial-commands.h>

using pilotguru::kia::KiaControlCommand;
using pilotguru::kia::SteeringAngle;

MainWindow::MainWindow(const std::string &can_interface,
                       const std::string &arduino_tty,
                       const pilotguru::kia::SteeringAngleHolderSettings
                           &steering_controller_settings,
                       const std::string &log_dir, QWidget *parent)
    : QMainWindow(parent), ui(new Ui::MainWindow),
      car_motion_data_(new pilotguru::kia::CarMotionData(10)),
      car_motion_data_updater_(new pilotguru::kia::CarMotionDataUpdater(
          car_motion_data_.get(), can_interface, {0x2B0, 0x4B0}, {1, 0})),
      arduino_command_channel_(
          new pilotguru::ArduinoCommandChannel(arduino_tty)),
      steering_controller_(new pilotguru::kia::SteeringAngleHolderController(
          &(car_motion_data_->steering_angles()),
          arduino_command_channel_.get(), steering_controller_settings)),
      kia_commands_logger_(
          new pilotguru::TimestampedJsonLogger<KiaControlCommand>(
              log_dir, pilotguru::STEERING_COMMANDS_LOG_ROOT_ELEMENT,
              std::unique_ptr<pilotguru::JsonSteamWriter<KiaControlCommand>>(
                  new pilotguru::SteeringCommandsJsonWriter()),
              &(arduino_command_channel_->CommandsHistory()))),
      steering_angles_logger_(
          new pilotguru::TimestampedJsonLogger<SteeringAngle>(
              log_dir, pilotguru::STEERING_ANGLES_LOG_ROOT_ELEMENT,
              std::unique_ptr<pilotguru::JsonSteamWriter<SteeringAngle>>(
                  new pilotguru::SteeringAngleJsonWriter()),
              &(car_motion_data_->steering_angles()))) {
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
          &MainWindow::SetTargetSteeringAngleFromInputField);
  connect(ui->target_angle_clear_button, &QPushButton::clicked, this,
          &MainWindow::ClearTargetSteeringAngle);
  connect(ui->left_turn_button, &QPushButton::clicked, this,
          &MainWindow::TurnLeft);
  connect(ui->right_turn_button, &QPushButton::clicked, this,
          &MainWindow::TurnRight);
}

MainWindow::~MainWindow() {
  steering_angle_read_thread_->RequestStop();
  velocity_read_thread_->RequestStop();
  steering_torque_offset_read_thread_->RequestStop();
  steering_angle_read_thread_->wait();
  velocity_read_thread_->wait();
  steering_torque_offset_read_thread_->wait();
  kia_commands_logger_->Stop();
  steering_angles_logger_->Stop();
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

void MainWindow::SetTargetSteeringAngleFromInputField() {
  bool parse_success = false;
  const short target_angle_degrees =
      ui->target_angle_in_field->text().toShort(&parse_success);
  if (parse_success) {
    SetTargetSteeringAngle(target_angle_degrees);
  }
}

void MainWindow::ClearTargetSteeringAngle() {
  steering_controller_->ClearTargetAngle();
  ui->target_angle_value_label->setText("not set");
}

void MainWindow::OnSteeringAngleChanged(int16_t angle_deci_degrees) {
  const double angle_degrees = static_cast<double>(angle_deci_degrees) / 10.0;
  LOG(INFO) << "Set angle: " << angle_degrees;
  ui->steering_angle_value_label->setText(QString::number(angle_degrees));
}

void MainWindow::OnVelocityChanged(QString text) {
  ui->velocity_value_label->setText(text);
}

void MainWindow::OnSteeringTorqueChanged(QString text) {
  ui->torque_offset_value_label->setText(text);
}

constexpr double turns_angle_change_degrees = 5.0;

void MainWindow::TurnLeft() {
  ShiftTargetSteeringAngle(turns_angle_change_degrees);
}

void MainWindow::TurnRight() {
  ShiftTargetSteeringAngle(-turns_angle_change_degrees);
}

void MainWindow::SetTargetSteeringAngle(double target_angle_degrees) {
  if (std::abs(target_angle_degrees) <=
      steering_controller_->settings().max_target_angle_amplitude) {
    steering_controller_->SetTargetAngle(target_angle_degrees);
    ui->target_angle_value_label->setText(
        QString::number(target_angle_degrees));
  }
}

void MainWindow::ShiftTargetSteeringAngle(double target_angle_shift_degrees) {
  if (steering_controller_->IsTargetAngleSet()) {
    SetTargetSteeringAngle(steering_controller_->GetTargetAngle() +
                           target_angle_shift_degrees);
  }
}
