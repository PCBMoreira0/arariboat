#include "data.hpp"
#include <Arduino.h>

SystemData::SystemData() {
    all_info = {
        .battery_voltage = 0.0f,
        .motor_current_left = 0.0f,
        .motor_current_right = 0.0f,
        .mppt_current = 0.0f,
        .temperature_battery_left = -127.0f,
        .temperature_battery_right = -127.0f,
        .temperature_mppt = -127.0f,
        .latitude = 0.0f,
        .longitude = 0.0f,
        .rpm_left = 0.0f,
        .rpm_right = 0.0f,
        .timestamp = 0
    };
}

void SystemData::WriteToSerial() {
    mavlink_all_info_t info = all_info;
    mavlink_message_t message;
    mavlink_msg_all_info_encode(1, 200, &message, &info);
    uint8_t buffer[MAVLINK_MAX_PACKET_LEN];
    uint16_t len = mavlink_msg_to_send_buffer(buffer, &message);
    Serial.write(buffer, len);
}

SystemData& SystemData::getInstance() {
    static SystemData instance;
    return instance;
}