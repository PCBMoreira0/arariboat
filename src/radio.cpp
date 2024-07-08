#include <Arduino.h>
#include "Utilities.hpp"
#include "arariboat/mavlink.h" // Custom mavlink dialect for the boat generated using Mavgen tool.
#include <LoRa.h> // Lora physical layer library
#include "BoardDefinitions.h" // SX1276, SDCard and OLED display pin definitions
#include "radio.hpp" // Radio operation header file
#include "LoraConfigManager.hpp" // Non-volatile storage for system parameters
#include "event_loop.hpp" // Event loop for handling events

static void SerialCommandCallback(void* handler_args, esp_event_base_t base, int32_t id, void* event_data) {
    
    const char* command = (const char*)event_data;

    if (strncmp(command, "lora", 4) == 0) {
        LoraConfiguration config;
        if (!GetLoraConfiguration(config)) {
            DEBUG_PRINTF("[RADIO]Failed to get Lora configuration\n", NULL);
            return;
        }

        DEBUG_PRINTF("[RADIO]Lora configuration: Freq: %ld, BW: %ld, SF: %d, CR: %d, Power: %d, SyncWord: %d, CRC: %d\n",
            config.frequency, config.bandwidth, config.spreadingFactor, config.codingRate, config.power, config.syncWord, config.crcEnabled);
    }

    if (strncmp(command, "mavtemp", 7) == 0) {
        DEBUG_PRINTF("[LORA]Sending temperatures\n", NULL);
        mavlink_message_t message;
        mavlink_msg_temperatures_pack(1, 200, &message, 25.0f, 26.0f, 27.0f, -1);
        EnqueueMavlinkMessage(message, radioQueue);
    }
}

void EnqueueMavlinkMessage(mavlink_message_t message, QueueHandle_t queue) {

    constexpr TickType_t timeToWait = pdMS_TO_TICKS(100);

    if (queue == nullptr) {
        DEBUG_PRINTF("[SERIAL_PARSER]Queue is null\n", NULL);
        return;
    }

    if (xQueueSend(queue, &message, timeToWait) != pdTRUE) {
        DEBUG_PRINTF("[SERIAL_PARSER]Failed to send mavlink message to queue\n", NULL);
    }
}

QueueHandle_t radioQueue;

void InitializeRadio() {
    
    Lora.setPins(CONFIG_CS, CONFIG_RST, CONFIG_DIO0);

    LoraConfiguration config;
    if (!GetLoraConfiguration(config)) {
        Serial.println("Failed to get Lora configuration.");
        vTaskDelete(NULL);
    }

    while (!Lora.begin(config.frequency)) { 
        Serial.println("Starting Lora failed!");
        vTaskDelay(pdMS_TO_TICKS(1000));
    }
    Serial.println("Starting Lora succeeded!");

    Lora.setSpreadingFactor(config.spreadingFactor);
    Lora.setSignalBandwidth(config.bandwidth);
    Lora.setCodingRate4(config.codingRate);
    Lora.setTxPower(config.power);
    Lora.setSyncWord(config.syncWord);
    config.crcEnabled ? Lora.enableCrc() : Lora.disableCrc();

    Serial.printf("LoRa configuration: Freq: %ld, BW: %ld, SF: %d, CR: %d, Power: %d, SyncWord: %d, CRC: %d\n",
        config.frequency, config.bandwidth, config.spreadingFactor, config.codingRate, config.power, config.syncWord, config.crcEnabled);
}

void InitializeRadioQueue() {
    constexpr int radioQueueLength = 30;
    radioQueue = xQueueCreate(radioQueueLength, sizeof(mavlink_message_t));

    if (radioQueue == NULL) {
        Serial.println("Failed to create radio queue!");
        vTaskDelete(NULL);
    }
}

void RadioTask(void* parameter) {
    
    InitializeRadio();
    InitializeRadioQueue();

    esp_event_handler_register_with(eventLoop, SERIAL_PARSER_EVENT_BASE, ESP_EVENT_ANY_ID, SerialCommandCallback, NULL);

    while (true) {
        mavlink_message_t message;
        if (xQueueReceive(radioQueue, &message, portMAX_DELAY) == pdTRUE) {
            uint8_t buffer[MAVLINK_MAX_PACKET_LEN];
            uint16_t length = mavlink_msg_to_send_buffer(buffer, &message);
            if (!Lora.beginPacket()) {
                DEBUG_PRINTF("[RADIO]Failed to begin packet\n", NULL);
                continue;
            }
            Lora.write(buffer, length);
            Lora.endPacket();
        }
    }
}