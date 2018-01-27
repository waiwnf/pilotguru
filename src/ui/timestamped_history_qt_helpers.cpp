#include "timestamped_history_qt_helpers.h"

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
  emit SteeringAngleChanged(value.data().angle_deci_degrees);
}

VelocityReadThread::VelocityReadThread(
    const TimestampedHistory<Velocity> *values_history)
    : TimestampedValueReadThread<Velocity>(values_history) {}

void VelocityReadThread::ProcessValue(const Timestamped<Velocity> &value) {
  emit VelocityChanged(QString::number(value.data().average_wheel_speed()));
}

SteeringTorqueOffsetReadThread::SteeringTorqueOffsetReadThread(
    const TimestampedHistory<KiaControlCommand> *values_history)
    : TimestampedValueReadThread<KiaControlCommand>(values_history) {}

void SteeringTorqueOffsetReadThread::ProcessValue(
    const Timestamped<KiaControlCommand> &command) {
  if (command.data().type == KiaControlCommand::STEER) {
    emit SteeringTorqueChanged(QString::number(command.data().value));
  }
}
