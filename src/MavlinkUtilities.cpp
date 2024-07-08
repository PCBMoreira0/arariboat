#include "MavlinkUtilities.hpp"

void PrintMavlinkMessageInfo(mavlink_message_t message) {
    switch (message.msgid) {
        case MAVLINK_MSG_ID_HEARTBEAT: {
            DEBUG_PRINTF("[RX]Received heartbeat\n", NULL);
            break;
        }
        case MAVLINK_MSG_ID_CONTROL_SYSTEM: {
            DEBUG_PRINTF("[RX]Received control system\n", NULL);
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
        case MAVLINK_MSG_ID_AUX_SYSTEM: {
            DEBUG_PRINTF("[RX]Received aux system\n", NULL);
            break;
        }
        case MAVLINK_MSG_ID_LORA_PARAMS: {
            DEBUG_PRINTF("[RX]Received lora params\n", NULL);
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
            mavlink_msg_temperatures_pack(1, 200, &message, 25.0, 30.0, 35.0);
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
        case MAVLINK_MSG_ID_CONTROL_SYSTEM: {
            return "CONTROL_SYSTEM";
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
        case MAVLINK_MSG_ID_AUX_SYSTEM: {
            return "AUX_SYSTEM";
        }
        case MAVLINK_MSG_ID_LORA_PARAMS: {
            return "LORA_PARAMS";
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