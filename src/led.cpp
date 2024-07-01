#include <Arduino.h>


enum BlinkRate : uint32_t {
    Slow = 2000,
    Medium = 1000,
    Fast = 300,
    Pulse = 100 // Pulse is a special value that will make the LED blink fast and then return to the previous blink rate.
};

enum GPSPrintOptions : uint32_t {
    Off = '0',
    Raw,
    Parsed
};

// Tasks can send notifications here to change the blink rate of the LED in order to communicate the status of the boat.
void FastBlinkPulse(int pin);
void LedBlinkerTask(void* parameter) {

    constexpr uint8_t ledPin = GPIO_NUM_2; // Built-in LED pin for the ESP32 DevKit board.
    pinMode(ledPin, OUTPUT);
    uint32_t blink_rate = BlinkRate::Slow;
    uint32_t previous_blink_rate = blink_rate;

    auto FastBlinkPulse = [](int pin) {
        for (int i = 0; i < 4; i++) {
            digitalWrite(pin, HIGH); vTaskDelay(pdMS_TO_TICKS(50));
            digitalWrite(pin, LOW);  vTaskDelay(pdMS_TO_TICKS(50));
        }
    };

    auto BuzzerWrite = [&]() {
        constexpr uint8_t buzzer_pin = GPIO_NUM_26;
        static uint8_t counter = 0;

        // Define the rhythm pattern
        constexpr uint8_t pattern[] = {1, 0, 1, 0, 1, 1, 0, 0}; // Example pattern

        // Calculate the position within the pattern
        uint8_t patternSize = sizeof(pattern) / sizeof(pattern[0]);
        uint8_t patternPosition = counter % patternSize;

        // Determine the buzzer state based on the pattern
        bool buzzerState = pattern[patternPosition] != 0;

        if (blink_rate == BlinkRate::Fast) {
            dacWrite(buzzer_pin, buzzerState ? 150 : 0);
        } else {
            dacWrite(buzzer_pin, 0);
        }
        // Increment the counter
        counter++;
    };
  
    while (true) {

        static uint32_t previous_blink_time = millis();
        if (millis() - previous_blink_time > blink_rate) {
            previous_blink_time = millis();
            BuzzerWrite();
            digitalWrite(ledPin, !digitalRead(ledPin));
        }
           
        // Set blink rate to the value received from the notification
        static uint32_t received_value = BlinkRate::Slow;
        if (xTaskNotifyWait(0, 0, (uint32_t*)&received_value, 100)) {
            if (received_value == BlinkRate::Pulse) {
                FastBlinkPulse(ledPin);
            } else {
                blink_rate = received_value;
            }
        }
    }
}