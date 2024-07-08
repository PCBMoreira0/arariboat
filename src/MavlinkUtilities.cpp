#include "MavlinkUtilities.hpp"
#define NO_TIMESTAMP -1

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