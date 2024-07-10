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

String SystemData::GetLineProtocol() {
    mavlink_all_info_t info = all_info;
    char buffer[512];
    memset(buffer, 0, sizeof(buffer));
   
    sprintf(buffer, "Yonah battery_voltage=%.2f,"
                    "motor_current_left=%.2f,"
                    "motor_current_right=%.2f,"
                    "mppt_current=%.2f,"
                    "temp_bat_left=%.2f,"
                    "temp_bat_right=%.2f,"
                    "temp_mppt=%.2f,"
                    "latitude=%f,"
                    "longitude=%f,"
                    "rpm_left=%.2f,"
                    "rpm_right=%.2f",
                    info.battery_voltage,
                    info.motor_current_left,
                    info.motor_current_right,
                    info.mppt_current,
                    info.temperature_battery_left,
                    info.temperature_battery_right,
                    info.temperature_mppt,
                    info.latitude,
                    info.longitude,
                    info.rpm_left,
                    info.rpm_right);
                    
    if (info.timestamp != 0) {
        sprintf(buffer + strlen(buffer), " %lu", info.timestamp);
    }

    return String(buffer);
}

SystemData& SystemData::getInstance() {
    static SystemData instance;
    return instance;
}