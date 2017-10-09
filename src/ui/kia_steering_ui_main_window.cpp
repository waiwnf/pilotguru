#include <cstdio>
#include <iostream>

#include "kia_steering_ui_main_window.h"
#include <ui_kia_steering_ui_main_window.h>

#include <glog/logging.h>

namespace {
void runSteeringAngleRead(pilotguru::kia::CarMotionData *car_motion_data,
                          QLabel *steering_angle_value_label,
                          std::mutex *ui_elements_mutex) {
  constexpr size_t buffer_size = 64;
  char text[buffer_size];
  pilotguru::kia::Timestamped<pilotguru::kia::SteeringAngle> steering_instance(
      {}, {0, 0});
  while (true) {
    steering_instance = car_motion_data->steering_angles().wait_get_next(
        steering_instance.timestamp());
    snprintf(text, buffer_size, "%d",
             steering_instance.data().angle_deci_degrees);
    std::unique_lock<std::mutex> lock(*ui_elements_mutex);
    steering_angle_value_label->setText(text);
  }
}

void runVelocityRead(pilotguru::kia::CarMotionData *car_motion_data,
                     QLabel *velocity_value_label,
                     std::mutex *ui_elements_mutex) {
  constexpr size_t buffer_size = 64;
  char text[buffer_size];
  pilotguru::kia::Timestamped<pilotguru::kia::Velocity> velocity_instance(
      {}, {0, 0});
  while (true) {
    velocity_instance = car_motion_data->velocities().wait_get_next(
        velocity_instance.timestamp());
    snprintf(text, buffer_size, "%d",
             velocity_instance.data().average_wheel_speed());
    std::unique_lock<std::mutex> lock(*ui_elements_mutex);
    velocity_value_label->setText(text);
  }
}
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

  steering_angle_read_thread_.reset(
      new std::thread(runSteeringAngleRead, car_motion_data_.get(),
                      ui->steering_angle_value_label, &ui_elements_mutex_));
  velocity_read_thread_.reset(
      new std::thread(runVelocityRead, car_motion_data_.get(),
                      ui->velocity_value_label, &ui_elements_mutex_));

  connect(ui->monitoring_start_button, &QPushButton::clicked, this,
          &MainWindow::startMonitoring);
  connect(ui->monitoring_stop_button, &QPushButton::clicked, this,
          &MainWindow::stopMonitoring);
  connect(ui->steering_torque_send_button, &QPushButton::clicked, this,
          &MainWindow::sendSteering);
}

MainWindow::~MainWindow() {
  car_motion_data_updater_->stop();
  steering_angle_read_thread_->detach();
  velocity_read_thread_->detach();
  delete ui;
}

void MainWindow::startMonitoring() { car_motion_data_updater_->start(); }

void MainWindow::stopMonitoring() { car_motion_data_updater_->stop(); }

void MainWindow::sendSteering() {
  // TODO parse value, refactor to use the command struct.
  const std::string command =
      std::string("s") + ui->steering_torque_in_field->text().toStdString();
  arduino_command_channel_->SendCommand(command);
}
