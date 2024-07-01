#include "blinker.hpp"

QueueHandle_t blinkerQueue;

/// @brief Responds to a BlinkCommand by blinking the LED the specified number of times.
/// @param ledPin 
void BlinkerTask(void* ledPin) {

    int statusPin = GPIO_NUM_2;
    pinMode(statusPin, OUTPUT);

    blinkerQueue = xQueueCreate(10, sizeof(BlinkCommand));
    BlinkCommand command;
    while (true) {
        int previousState;
        if (xQueueReceive(blinkerQueue, &command, portMAX_DELAY)) {
            previousState = digitalRead(statusPin);
            for (int i = 0; i < command.repeats; i++) {
                digitalWrite(statusPin, HIGH);
                vTaskDelay(pdMS_TO_TICKS(command.onTime));
                digitalWrite(statusPin, LOW);
                vTaskDelay(pdMS_TO_TICKS(command.offTime));
            }
            digitalWrite(statusPin, previousState);
        }
    }
}

/// @brief Sends a BlinkCommand to the blinker queue.
/// @param onTime 
/// @param offTime 
/// @param repeats 
void Blink(int onTime = 100, int offTime = 100, int repeats = 1) {
    BlinkCommand command;
    command.onTime = onTime;
    command.offTime = offTime;
    command.repeats = repeats;
    if (blinkerQueue == NULL) return;
    xQueueSend(blinkerQueue, &command, portMAX_DELAY);
}