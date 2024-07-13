#include <Arduino.h>
#include <event_loop.hpp>
#include "arariboat/mavlink.h"
#include "Utilities.hpp"
#include "radio.hpp"
#include "MavlinkUtilities.hpp"
#include "storage.hpp"

esp_event_base_t SERIAL_PARSER_EVENT_BASE = "SERIAL_PARSER";
//ESP_EVENT_DECLARE_BASE(SERIAL_PARSER_EVENT_BASE);

void ProcessMavlinkMessage(mavlink_message_t message) {
  
    PrintMavlinkMessageInfo(message);
    EnqueueMavlinkMessage(message, radioQueue);      
    SaveMavlinkMessage(message);
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


