#include <Arduino.h> // Main Arduino library, required for projects that use the Arduino framework.
#include "Utilities.hpp" // Utility functions for the project
#include "event_loop.hpp" // Event loop for handling events between tasks.

//TODO: Move I2C instrumentation here to read from the ADS1115 ADC to fix I2C screen conflict
//TODO: Use IDF event loops for serial communication and intertask communication
//TODO: Adaptative bandwidth and spreading factor based on packet loss and distance


// Declare a handle for each task to allow manipulation of the task from other tasks, such as sending notifications, resuming or suspending.
// The handle is initialized to nullptr to avoid the task being created before the setup() function.
// Each handle is then assigned to the task created in the setup() function.

TaskHandle_t ledBlinkerHandle = nullptr;
TaskHandle_t wifiTaskHandle = nullptr;
TaskHandle_t serverHandle = nullptr;
TaskHandle_t serialReaderHandle = nullptr;

void InitializeFlashMemory();

void DisplayScreenTask(void* parameter);
void LedBlinkerTask(void* parameter);
void WifiTask(void* parameter);
void ServerTask(void* parameter);
void SerialReaderTask(void* parameter);
void RadioTask(void* parameter);
void FlashCardReaderTask(void *parameter);

void setup() {
    
    Serial.begin(BAUD_RATE);

    InitializeEventLoop(&eventLoop);
    xTaskCreate(LedBlinkerTask, "ledBlinker", 2048, NULL, 1, &ledBlinkerHandle);
    xTaskCreate(SerialReaderTask, "serialReader", 16384, NULL, 1, &serialReaderHandle);
    //xTaskCreate(WifiTask, "wifiConnection", 4096, NULL, 1, &wifiTaskHandle);
    //xTaskCreate(ServerTask, "server", 4096, NULL, 1, &serverHandle);
    //xTaskCreate(DisplayScreenTask, "displayScreen", 4096, NULL, 1, NULL);
    //InitializeFlashMemory();
    //xTaskCreate(RadioTask, "radio", 4096, NULL, 1, NULL);
    xTaskCreate(FlashCardReaderTask, "flashCardReader", 8192, NULL, 1, NULL);
}

void loop() {
    vTaskDelete(NULL);
}


