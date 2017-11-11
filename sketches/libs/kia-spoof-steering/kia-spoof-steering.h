// Helpers for the steering control loop on the Arduino side.

#ifndef PILOTGURU_KIA_SPOOF_STEERING_LIB_
#define PILOTGURU_KIA_SPOOF_STEERING_LIB_

#include <stddef.h>

#include <spoof-steering-serial-commands.h>
#include <spoof-voltage-smoothing.h>

constexpr uint8_t LW232_RET_ASCII_OK = 0x0D;
constexpr uint8_t LW232_RET_ASCII_ERROR = 0x07;

bool ExecuteKiaControlCommand(
    const pilotguru::kia::KiaControlCommand &control_command,
    TargetVoltageSmoother *voltage_smoother, bool *is_echo_on);

void HandleCommandProcessorState(
    pilotguru::kia::KiaControlCommandProcessor *command_processor,
    pilotguru::kia::KiaControlCommand *control_command,
    TargetVoltageSmoother *voltage_smoother, bool *is_echo_on);

void ProcessAvailableSerialBuffer(
    pilotguru::kia::KiaControlCommandProcessor *command_processor);

void SerialWriteCurrentSpoofVoltages(
    const TargetVoltageSmoother &voltage_smoother, char *voltage_state_string,
    size_t voltage_state_buffer_size);

#endif // PILOTGURU_KIA_SPOOF_STEERING_LIB_
