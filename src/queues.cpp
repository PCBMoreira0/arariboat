#include "queues.hpp"
#include "freertos/FreeRTOS.h"

QueueHandle_t broker_queue;
QueueHandle_t main_radio_queue;
QueueHandle_t auxiliary_radio_queue;
QueueHandle_t internet_queue;
QueueHandle_t logger_queue;
QueueHandle_t can_queue; 
QueueHandle_t propulsion_queue;

#define message_queue_length 10
#define main_radio_queue_length 10
#define auxiliary_radio_queue_length 10
#define internet_queue_length 10
#define logger_queue_length 10
#define can_queue_length 10

void initialize_queues() {
    broker_queue = xQueueCreate(message_queue_length, sizeof(message_t));
    main_radio_queue = xQueueCreate(main_radio_queue_length, sizeof(message_t));
    auxiliary_radio_queue = xQueueCreate(auxiliary_radio_queue_length, sizeof(message_t));
    internet_queue = xQueueCreate(internet_queue_length, sizeof(message_t));
    logger_queue = xQueueCreate(logger_queue_length, sizeof(message_t));
    can_queue = xQueueCreate(can_queue_length, sizeof(message_t));
    propulsion_queue = xQueueCreate(message_queue_length, sizeof(message_t)); 

    if (broker_queue == NULL || main_radio_queue == NULL || auxiliary_radio_queue == NULL ||
        internet_queue == NULL || logger_queue == NULL || can_queue == NULL || propulsion_queue == NULL) {
        printf("Failed to create one or more queues.\n");
        //Halt the system
        while (true) {
            vTaskDelay(pdMS_TO_TICKS(1000)); // Wait indefinitely
        }
    }
}



