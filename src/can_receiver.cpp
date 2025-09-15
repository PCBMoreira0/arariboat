// Copyright (c) Sandeep Mistry. All rights reserved.
// Licensed under the MIT license. See LICENSE file in the project root for full license information.

#include <Arduino.h>
#include "driver/twai.h"
#include "esp_log.h"
#include "motor_can_manager.h"
#include "bms_can_manager.h"
#include "led.hpp"
#include "Utilities.hpp"
#include "queues.hpp"
#include "arariboat/mavlink.h"
#include "mavlink_data_conversion.h"
#include "throttling_config.h"

#ifdef DEBUG
#undef DEBUG // Uncomment to enable debug messages locally in this file
#endif

static const char* TAG = "CAN";

MotorCANManager motor_can_manager;
BMSCANManager bms_can_manager;

// --- Constants ---
#define MAVLINK_STREAM_CAN_ID 0xABC // The single CAN ID for streaming all MAVLink messages

/**
 * @brief Serializes a MAVLink message and transmits it as a stream of CAN frames.
 * All frames are sent with the same CAN ID (MAVLINK_STREAM_CAN_ID). The receiver
 * is expected to reassemble the bytes from these frames to parse the complete MAVLink message.
 * @param msg The MAVLink message to send.
 */
static void send_mavlink_can_stream(const mavlink_message_t* msg) {
    // 1. Serialize the entire MAVLink message (header, payload, checksum) into a buffer.
    uint8_t mav_buffer[MAVLINK_MAX_PACKET_LEN];
    uint16_t mav_len = mavlink_msg_to_send_buffer(mav_buffer, msg);
    
    // char output_buffer[2 * MAVLINK_MAX_PACKET_LEN + 1] = {0};
    // if (!bytes_to_hex_string(output_buffer, sizeof(output_buffer), mav_buffer, mav_len)) {
    //     DEBUG_PRINTF("[CAN] Failed to convert MAVLink message to hex string for MSG ID: %d\n", msg->msgid);
    //     return; // Abort sending if conversion fails
    // }
    // Serial.printf("[CAN][DEBUG] Packet = %s\n", output_buffer);

    // 2. Prepare the CAN frame structure. It will be reused for each chunk.
    twai_message_t can_msg;
    can_msg.flags = TWAI_MSG_FLAG_EXTD;
    can_msg.identifier = MAVLINK_STREAM_CAN_ID;

    // 3. Loop through the serialized buffer and transmit it in 8-byte chunks.
    uint16_t bytes_sent = 0;
    while (bytes_sent < mav_len) {
        uint8_t chunk_size = (mav_len - bytes_sent > 8) ? 8 : (mav_len - bytes_sent);
        can_msg.data_length_code = chunk_size;
        memcpy(can_msg.data, &mav_buffer[bytes_sent], chunk_size);

        // Transmit the current chunk.
        if (twai_transmit(&can_msg, pdMS_TO_TICKS(50)) != ESP_OK) {
            DEBUG_PRINTF("[CAN] Failed to send stream chunk for MAVLink MSG ID: %d\n", msg->msgid);
            return; // Abort sending the rest of this packet on failure.
        }
        bytes_sent += chunk_size;
    }
    
    // The ceiling of (mav_len / 8) gives the number of frames sent.
    DEBUG_PRINTF("[CAN] Sent MAVLink MSG ID %d (%u bytes) in %u CAN frames.\n", msg->msgid, mav_len, (mav_len + 7) / 8);
}

void can_receive_task(void* parameter) {
    while (true) {
        twai_message_t message;
        if (twai_receive(&message, pdMS_TO_TICKS(portMAX_DELAY)) != ESP_OK) continue;

        led_command_t led_command {
            .pattern = LED_PATTERN_DATA_XMIT,
            .priority = 1,
            .duration_ms = 0
        };

        static unsigned long last_led_command_time = 0;
        if (millis() - last_led_command_time > 200) { // Avoid spamming the LED command
            last_led_command_time = millis();
            if (!led_manager_request_pattern(led_command)) {
                DEBUG_PRINTF("[CAN] Failed to request LED pattern for data transmission\n");
            }
        }

        if (bms_can_manager.handle_can_frame(message)) continue;
        if (motor_can_manager.handle_can_frame(message)) continue;
        #ifdef DEBUG
        // If no handler processed the message, print it
        char buffer[256] = {0};
        sprintf(buffer, "Received message: ID=0x%X, Length=%d\n", message.identifier, message.data_length_code);
        for (int i = 0; i < message.data_length_code; i++) {
            sprintf(buffer + strlen(buffer), "0x%02X ", message.data[i]);
        }
        strcat(buffer, "\n");
        Serial.printf("%s", buffer);
        #endif
    }
}

// This task is responsible for polling BMS data and transmitting messages from the queue.
void can_transmit_task(void* parameter) {

    // Timers for each specific MAVLink message type.
    static uint32_t last_sent_time[NUM_THROTTLED_MESSAGES] = {0};

    while (true) {
        
        static unsigned long last_bms_poll_time = 0;
        if (millis() - last_bms_poll_time >= 500) {
            last_bms_poll_time = millis();
            bms_can_manager.poll_bms_data();
        }

        message_t message;
        if (xQueueReceive(can_queue, &message, pdMS_TO_TICKS(20)) == pdTRUE) {
            // System messages must be converted to CAN messages using conversion functions
            mavlink_message_t mavlink_msg;
            if (!mavlink_msg_from_message_t(message, &mavlink_msg)) {
                DEBUG_PRINTF("[CAN] Failed to convert message from source %s to MAVLink format\n", DATA_SOURCE_NAMES[message.source]);
                continue; // Skip this message if conversion failed
            }

            switch (message.source) {
                case DATA_SOURCE_BMS:
                case DATA_SOURCE_MOTOR_LEFT:
                case DATA_SOURCE_MOTOR_RIGHT: {
                    break; //Skip native CAN messages
                }  
                case DATA_SOURCE_GPS: {
                    if (millis() - last_sent_time[MSG_GPS] >= send_intervals_ms[MSG_GPS]) {
                        send_mavlink_can_stream(&mavlink_msg);
                        last_sent_time[MSG_GPS] = millis();
                    }
                    break;
                }
                 case DATA_SOURCE_MPPT: {
                    if (millis() - last_sent_time[MSG_MPPT] >= send_intervals_ms[MSG_MPPT]) {
                        send_mavlink_can_stream(&mavlink_msg);
                        last_sent_time[MSG_MPPT] = millis();
                    }
                    if (millis() - last_sent_time[MSG_MPPT_STATE] >= send_intervals_ms[MSG_MPPT_STATE]) {
                        send_mavlink_can_stream(&mavlink_msg);
                        last_sent_time[MSG_MPPT_STATE] = millis();
                    }
                    break;
                }
                case DATA_SOURCE_INSTRUMENTATION: {
                    if (millis() - last_sent_time[MSG_INSTRUMENTATION] >= send_intervals_ms[MSG_INSTRUMENTATION]) {
                        send_mavlink_can_stream(&mavlink_msg);
                        last_sent_time[MSG_INSTRUMENTATION] = millis();
                    }
                    break;
                }
                case DATA_SOURCE_TEMPERATURES: {
                    if (millis() - last_sent_time[MSG_TEMPERATURES] >= send_intervals_ms[MSG_TEMPERATURES]) {
                        send_mavlink_can_stream(&mavlink_msg);
                        last_sent_time[MSG_TEMPERATURES] = millis();
                    }
                    break;
                }
                case DATA_SOURCE_PROPULSION: {
                    if (millis() - last_sent_time[MSG_PROPULSION] >= send_intervals_ms[MSG_PROPULSION]) {
                        send_mavlink_can_stream(&mavlink_msg);
                        last_sent_time[MSG_PROPULSION] = millis();
                    }
                    break;
                }
            }
        }
    }
}


void simple_reception_test() {
    twai_message_t message;
    if (twai_receive(&message, pdMS_TO_TICKS(100)) == ESP_OK) {
        char buffer[256] = {0};
        sprintf(buffer, "Received message: ID=0x%X, Length=%d\n", message.identifier, message.data_length_code);
        for (int i = 0; i < message.data_length_code; i++) {
            sprintf(buffer + strlen(buffer), "0x%02X ", message.data[i]);
        }
        strcat(buffer, "\n");
        Serial.print(buffer);
    }
}

void queue_listener_task(void* parameter) {
    QueueHandle_t motor_queue = xQueueCreate(10, sizeof(MotorCANManager::motor_data_t));
    if (motor_queue == NULL) {
        Serial.println("\nFailed to create motor queue");
        vTaskDelete(NULL);
    }
    motor_can_manager.set_data_queue(motor_queue);

    QueueHandle_t bms_queue = xQueueCreate(10, sizeof(bms_data_t));
    if (bms_queue == NULL) {
        Serial.println("\nFailed to create BMS queue");
        vTaskDelete(NULL);
    }

    bms_can_manager.set_data_queue(bms_queue);

    while (true) {
        MotorCANManager::motor_data_t motor_data;
        if (xQueueReceive(motor_queue, &motor_data, pdMS_TO_TICKS(100)) == pdTRUE) {
            Serial.printf("\n[LISTEN]Motor Data: Bus Voltage=%.1fV, Bus Current=%.1fA, Phase Current=%.1fA, RPM=%d\n",
                motor_data.electrical_data.bus_voltage_dV / 10.f, motor_data.electrical_data.bus_current_dA / 10.f, motor_data.electrical_data.phase_current_dA / 10.f, motor_data.electrical_data.rpm);
        }

        bms_data_t bms_data;
        if (xQueueReceive(bms_queue, &bms_data, pdMS_TO_TICKS(100)) == pdTRUE) {
            Serial.printf("\n[LISTEN]BMS Data: Voltage=%.1fV, Current=%.1fA, SOC=%.1f%%\n",
                bms_data.voltage_data.cumulative_voltage_decivolts / 10.f,
                bms_data.voltage_data.current_deciamps / 10.f,
                bms_data.voltage_data.soc_decipercent / 10.f);
        }
    }
    vTaskDelete(NULL); // Delete the task to free up resources
}

void can_task(void* parameter) {
    twai_general_config_t g_config = TWAI_GENERAL_CONFIG_DEFAULT(GPIO_NUM_16, GPIO_NUM_17, TWAI_MODE_NORMAL);
    g_config.rx_queue_len = 10;
    twai_timing_config_t t_config = TWAI_TIMING_CONFIG_250KBITS();
    twai_filter_config_t f_config = TWAI_FILTER_CONFIG_ACCEPT_ALL();

    twai_driver_install(&g_config, &t_config, &f_config);
    twai_start();

    xTaskCreate(can_receive_task, "CANReceiveTask", 4096, NULL, 3, NULL);
    xTaskCreate(can_transmit_task, "CANTransmitTask", 4096, NULL, 2, NULL);
    xTaskCreate(queue_listener_task, "QueueListenerTask", 4096, NULL, 1, NULL); // Create the queue listener task
    vTaskDelete(NULL); // Delete the bootstrap task to free up resources
}