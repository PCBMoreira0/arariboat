#include <Arduino.h>
#include <event_loop.hpp>
#include "arariboat/mavlink.h"
#include "Utilities.hpp"
#include "radio.hpp"

esp_event_base_t SERIAL_PARSER_EVENT_BASE = "SERIAL_PARSER";
//ESP_EVENT_DECLARE_BASE(SERIAL_PARSER_EVENT_BASE);

void ProcessMavlinkMessage(mavlink_message_t message) {
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

        //Post mavlink message to queue
        EnqueueMavlinkMessage(message, radioQueue);      
    }
}

bool TryParseMavlinkMessage(uint8_t input, mavlink_channel_t channel) {
    mavlink_message_t message;
    mavlink_status_t status;
    if (mavlink_parse_char(channel, input, &message, &status)) {
        ProcessMavlinkMessage(message);
        return true;
    }
    return false;
}

bool TryParseASCIICommand(char input) {
    constexpr int inputBufferLength = 256;
    static char inputBuffer[inputBufferLength];
    static int bufferIndex = 0;

    if (input == '\r') return false;
    if (input == '\n') {
        inputBuffer[bufferIndex] = '\0';
        bufferIndex = 0;
        esp_event_post_to(eventLoop, SERIAL_PARSER_EVENT_BASE, 0, inputBuffer, strlen(inputBuffer) + 1, portMAX_DELAY);
        return true;   
    }

    inputBuffer[bufferIndex++] = input;
    if (bufferIndex >= inputBufferLength) {
        bufferIndex = 0;
    }
    return false;
}

/// @brief Parses the serial input from the user char-by-char and posts the result to the event loop.
/// The use of the event loop allows the serial parser to be decoupled from the receiver of the parsed data.
void SerialReaderTask(void *parameter) {

    while (true) {
        if (!Serial.available()) {
            vTaskDelay(pdMS_TO_TICKS(100));
            continue;
        }

        char input = Serial.read();
        TryParseMavlinkMessage(input, MAVLINK_COMM_0);
        TryParseASCIICommand(input);
    }
}


