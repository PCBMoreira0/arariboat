#include <Arduino.h>
#include "Utilities.hpp"

static void serialCommandCallback(void* handler_args, esp_event_base_t base, int32_t id, void* event_data) {
    
    const char* command = (const char*)event_data;

    if (strncmp(command, "blinkfast", 9) == 0) {
        xTaskNotify(ledBlinkerHandle, BlinkRate::Fast, eSetValueWithOverwrite);
    } else if (strncmp(command, "blinkmedium", 11) == 0) {
        xTaskNotify(ledBlinkerHandle, BlinkRate::Medium, eSetValueWithOverwrite);
    } else if (strncmp(command, "blinkslow", 9) == 0) {
        xTaskNotify(ledBlinkerHandle, BlinkRate::Slow, eSetValueWithOverwrite);
    } else if (strncmp(command, "pulse", 5) == 0) {
        xTaskNotify(ledBlinkerHandle, BlinkRate::Pulse, eSetValueWithOverwrite);
    }
}

void FastBlinkPulse(int pin) {
    for (int i = 0; i < 4; i++) {
        digitalWrite(pin, HIGH); vTaskDelay(pdMS_TO_TICKS(50));
        digitalWrite(pin, LOW);  vTaskDelay(pdMS_TO_TICKS(50));
    }
}

// Function to beep the buzzer using PWM
void Beep(int pin, int channel, int pulses, int frequency = 1000, int duration = 100) {

    constexpr int resolution = 8;

    ledcSetup(channel, frequency, resolution);
    ledcAttachPin(pin, channel);
    for (int i = 0; i < pulses; i++) {
        ledcWriteTone(channel, frequency); vTaskDelay(pdMS_TO_TICKS(duration));
        ledcWriteTone(channel, 0);         vTaskDelay(pdMS_TO_TICKS(duration));
    }
}

// Tasks can send notifications here to change the blink rate of the LED in order to communicate the status of the boat.
void LedBlinkerTask(void* parameter) {

    constexpr uint8_t pinLed = GPIO_NUM_25;
    constexpr uint8_t pinBuzzer = GPIO_NUM_4;
    pinMode(pinLed, OUTPUT);
    pinMode(pinBuzzer, OUTPUT);

    uint32_t blink_rate = BlinkRate::Slow;
    uint32_t previous_blink_rate = blink_rate;

    //Register serial callback commands
    esp_event_handler_register_with(eventLoop, SERIAL_PARSER_EVENT_BASE, ESP_EVENT_ANY_ID, serialCommandCallback, nullptr);

    while (true) {

        static uint32_t previous_blink_time = millis();
        if (millis() - previous_blink_time > blink_rate) {
            previous_blink_time = millis();
            digitalWrite(pinLed, !digitalRead(pinLed));
        }
           
        // Set blink rate to the value received from the notification
        static uint32_t received_value = BlinkRate::Slow;
        if (xTaskNotifyWait(0, 0, (uint32_t*)&received_value, 100)) {
            if (received_value == BlinkRate::Pulse) {
                FastBlinkPulse(pinLed);
                Beep(pinBuzzer, 0, 2);
            } else {
                blink_rate = received_value;
            }
        }
    }
}
