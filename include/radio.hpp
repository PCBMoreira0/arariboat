#include <Arduino.h>

//Queue for sending messages to the radio task
extern QueueHandle_t radioQueue;

void EnqueueMavlinkMessage(mavlink_message_t message, QueueHandle_t queue);