#include <Arduino.h>
#include <event_loop.hpp>

esp_event_base_t SERIAL_PARSER_EVENT_BASE = "SERIAL_PARSER";

/// @brief Parses the serial input from the user char-by-char and posts the result to the event loop.
/// The use of the event loop allows the serial parser to be decoupled from the receiver of the parsed data.
void SerialReaderTask(void *parameter) {

    Serial.begin(BAUD_RATE);
    constexpr int inputBufferLength = 256;
    char inputBuffer[inputBufferLength];
    int bufferIndex = 0;
    
    while (true) {
        if (!Serial.available()) {
            vTaskDelay(pdMS_TO_TICKS(100));
            continue;
        }

        char input = Serial.read();
        if (input == 'r') continue;
        if (input == '\n') {
            inputBuffer[bufferIndex] = '\0';
            bufferIndex = 0;
            esp_event_post_to(eventLoop, SERIAL_PARSER_EVENT_BASE, 0, inputBuffer, strlen(inputBuffer) + 1, portMAX_DELAY);
            continue;
        } 
        
        inputBuffer[bufferIndex++] = input;
        if (bufferIndex >= inputBufferLength) {
            bufferIndex = 0;
        }
        
    }
}


