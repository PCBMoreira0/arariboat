#pragma once
#include <cstdint>

// Enum for each MAVLink message type that needs independent throttling.
// This is shared between the CAN and Radio transmitter tasks.
enum ThrottledMessage {
    MSG_BMS,
    MSG_BMS_STATUS,
    MSG_MOTOR_I_LEFT,
    MSG_MOTOR_II_LEFT,
    MSG_MOTOR_I_RIGHT,
    MSG_MOTOR_II_RIGHT,
    MSG_MPPT,
    MSG_MPPT_STATE,
    MSG_GPS,
    MSG_INSTRUMENTATION,
    MSG_TEMPERATURES,
    MSG_PROPULSION,
    NUM_THROTTLED_MESSAGES // Must be last, provides the size of the enum.
};

// Global array defining the minimum interval in milliseconds between sending
// messages for each corresponding type in the ThrottledMessage enum.
const uint32_t send_intervals_ms[] = {
    [MSG_BMS]               = 1000,  // 1 Hz
    [MSG_BMS_STATUS]        = 5000,  // 0.2 Hz
    [MSG_MOTOR_I_LEFT]      = 500,   // 2 Hz
    [MSG_MOTOR_II_LEFT]     = 2000,  // 0.5 Hz
    [MSG_MOTOR_I_RIGHT]     = 500,   // 2 Hz
    [MSG_MOTOR_II_RIGHT]    = 2000,  // 0.5 Hz
    [MSG_MPPT]              = 1000,  // 1 Hz
    [MSG_MPPT_STATE]        = 5000,  // 0.2 Hz
    [MSG_GPS]               = 1000,  // 1 Hz
    [MSG_INSTRUMENTATION]   = 1000,  // 1 Hz
    [MSG_TEMPERATURES]      = 10000, // 0.1 Hz
    [MSG_PROPULSION]        = 200    // 5 Hz
};
