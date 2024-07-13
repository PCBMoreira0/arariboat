#include "MavlinkUtilities.hpp"
#define NO_TIMESTAMP 0

void PrintMavlinkMessageInfo(mavlink_message_t message) {
    switch (message.msgid) {
        case MAVLINK_MSG_ID_HEARTBEAT: {
            DEBUG_PRINTF("[RX]Received heartbeat\n", NULL);
            break;
        }
        case MAVLINK_MSG_ID_INSTRUMENTATION: {
            DEBUG_PRINTF("[RX]Received instrumentation\n", NULL);
            break;
        }
        case MAVLINK_MSG_ID_TEMPERATURES: {
            DEBUG_PRINTF("[RX]Received temperatures\n", NULL);
            break;
        }
        case MAVLINK_MSG_ID_GPS_INFO: {
            DEBUG_PRINTF("[RX]Received GPS info\n", NULL);
            break;
        }
        case MAVLINK_MSG_ID_RPM_INFO: {
            DEBUG_PRINTF("[RX]Received RPM info\n", NULL);
            break;
        }
        case MAVLINK_MSG_ID_ALL_INFO: {
            DEBUG_PRINTF("[RX]Received all info\n", NULL);
            break;
        }

        default: {
            DEBUG_PRINTF("[RX]Received message with ID #%d\n", message.msgid);
            break;
        }     
    }
}

bool GenerateMavlinkMessage(mavlink_message_t& message, int message_id) {
    switch (message_id) {
        case MAVLINK_MSG_ID_HEARTBEAT: {
            mavlink_msg_heartbeat_pack(1, 200, &message, MAV_TYPE_GENERIC, MAV_AUTOPILOT_INVALID, MAV_MODE_FLAG_DECODE_POSITION_MANUAL, 0, MAV_STATE_ACTIVE);
            return true;
        }

        case MAVLINK_MSG_ID_TEMPERATURES: {
            mavlink_msg_temperatures_pack(1, 200, &message, 25.0, 30.0, 35.0, NO_TIMESTAMP);
            return true;
        }

        case MAVLINK_MSG_ID_INSTRUMENTATION: {
            mavlink_msg_instrumentation_pack(1, 200, &message, 25.2, 10.0, 20.0, 5.0, NO_TIMESTAMP);
            return true;
        }

        case MAVLINK_MSG_ID_GPS_INFO: {
            mavlink_msg_gps_info_pack(1, 200, &message, -22.0, -47.0, 5.0, 90.0, 9, NO_TIMESTAMP);
            return true;
        }

        case MAVLINK_MSG_ID_RPM_INFO: {
            mavlink_msg_rpm_info_pack(1, 200, &message, 45.0, 73.0, NO_TIMESTAMP);
            return true;
        }

        case MAVLINK_MSG_ID_ALL_INFO: {
            mavlink_msg_all_info_pack(1, 200, &message, 25.2, 10.0, 20.0, 5.0, -22.0, -47.0, 5.0, 90.0, 9, 45.0, 73.0, NO_TIMESTAMP);
            return true;
        }

        default: {
            return false;
        }
    }
}

String GetMavlinkMessageName(mavlink_message_t message) {
    switch (message.msgid) {
        case MAVLINK_MSG_ID_HEARTBEAT: {
            return "HEARTBEAT";
        }
        case MAVLINK_MSG_ID_INSTRUMENTATION: {
            return "INSTRUMENTATION";
        }
        case MAVLINK_MSG_ID_TEMPERATURES: {
            return "TEMPERATURES";
        }
        case MAVLINK_MSG_ID_GPS_INFO: {
            return "GPS_INFO";
        }
        case MAVLINK_MSG_ID_RPM_INFO: {
            return "RPM_INFO";
        }
        case MAVLINK_MSG_ID_ALL_INFO: {
            return "ALL_INFO";
        }
        default: {
            return "UNKNOWN";
        }
    }
}

void WriteMessageToSerial(mavlink_message_t message) {
    uint8_t buffer[MAVLINK_MAX_PACKET_LEN];
    uint16_t len = mavlink_msg_to_send_buffer(buffer, &message);
    Serial.write(buffer, len);
}


String MavlinkToLineProtocol(mavlink_message_t message) {

    char buffer[5112];
    memset(buffer, 0, sizeof(buffer));
    switch (message.msgid) {
        case MAVLINK_MSG_ID_HEARTBEAT: {
            mavlink_heartbeat_t heartbeat;
            mavlink_msg_heartbeat_decode(&message, &heartbeat);
            sprintf(buffer, "Yonah heartbeat=%d", heartbeat.custom_mode);
            break;
        }
        case MAVLINK_MSG_ID_INSTRUMENTATION: {
            mavlink_instrumentation_t instrumentation;
            mavlink_msg_instrumentation_decode(&message, &instrumentation);
            sprintf(buffer, "Yonah battery_voltage=%.2f,"
                            "motor_current_left=%.2f,"
                            "motor_current_right=%.2f,"
                            "mppt_current=%.2f",
                            instrumentation.battery_voltage,
                            instrumentation.motor_current_left,
                            instrumentation.motor_current_right,
                            instrumentation.mppt_current);
            if (instrumentation.timestamp != NO_TIMESTAMP) {
                sprintf(buffer + strlen(buffer), " %lu", instrumentation.timestamp);
            }
            break;
        }
        case MAVLINK_MSG_ID_TEMPERATURES: {
            mavlink_temperatures_t temperatures;
            mavlink_msg_temperatures_decode(&message, &temperatures);
            sprintf(buffer, "Yonah temp_bat_left=%.2f,"
                            "temp_bat_right=%.2f,"
                            "temp_mppt=%.2f",
                            temperatures.temperature_battery_left,
                            temperatures.temperature_battery_right,
                            temperatures.temperature_mppt);
            if (temperatures.timestamp != NO_TIMESTAMP) {
                sprintf(buffer + strlen(buffer), " %lu", temperatures.timestamp);
            }
            break;
        }
        case MAVLINK_MSG_ID_GPS_INFO: {
            mavlink_gps_info_t gps_info;
            mavlink_msg_gps_info_decode(&message, &gps_info);
            sprintf(buffer, "Yonah latitude=%.2f,"
                            "longitude=%.2f,"
                            "course=%.2f,"
                            "speed=%.2f,"
                            "satellites=%d",
                            gps_info.latitude,
                            gps_info.longitude,
                            gps_info.course,
                            gps_info.speed,
                            gps_info.satellites_visible);
            if (gps_info.timestamp != NO_TIMESTAMP) {
                sprintf(buffer + strlen(buffer), " %lu", gps_info.timestamp);
            }
            break;
        }
        case MAVLINK_MSG_ID_RPM_INFO: {
            mavlink_rpm_info_t rpm_info;
            mavlink_msg_rpm_info_decode(&message, &rpm_info);
            sprintf(buffer, "Yonah rpm_left=%.2f,"
                            "rpm_right=%.2f",
                            rpm_info.rpm_left,
                            rpm_info.rpm_right);
            if (rpm_info.timestamp != NO_TIMESTAMP) {
                sprintf(buffer + strlen(buffer), " %lu", rpm_info.timestamp);
            }
            break;
        }
        case MAVLINK_MSG_ID_ALL_INFO: {
            mavlink_all_info_t all_info;
            mavlink_msg_all_info_decode(&message, &all_info);
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
                            all_info.battery_voltage,
                            all_info.motor_current_left,
                            all_info.motor_current_right,
                            all_info.mppt_current,
                            all_info.temperature_battery_left,
                            all_info.temperature_battery_right,
                            all_info.temperature_mppt,
                            all_info.latitude,
                            all_info.longitude,
                            all_info.rpm_left,
                            all_info.rpm_right);
            if (all_info.timestamp != NO_TIMESTAMP) {
                sprintf(buffer + strlen(buffer), " %lu", all_info.timestamp);
            }
            break;
        }
        default: {
            sprintf(buffer, "");
            break;
        }
    }
    return String(buffer);
}