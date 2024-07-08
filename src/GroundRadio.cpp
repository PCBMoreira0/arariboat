#include <Arduino.h> // Main Arduino library, required for projects that use the Arduino framework.
#include "Utilities.hpp" // Custom header file that contains utility functions and macros.

extern void InitializeFlashMemory();

// Declare a handle for each task to allow manipulation of the task from other tasks, such as sending notifications, resuming or suspending.
// The handle is initialized to nullptr to avoid the task being created before the setup() function.
// Each handle is then assigned to the task created in the setup() function.

TaskHandle_t ledBlinkerHandle = nullptr;
TaskHandle_t displayScreenHandle = nullptr;
TaskHandle_t wifiHandle = nullptr;
TaskHandle_t serverHandle = nullptr;
TaskHandle_t serialReaderHandle = nullptr;
TaskHandle_t radioHandle = nullptr;


void setup() {

    Serial.begin(BAUD_RATE);

    InitializeEventLoop(&eventLoop);
    InitializeFlashMemory();

    xTaskCreate(LedBlinkerTask, "ledBlinker", 2048, NULL, 1, &ledBlinkerHandle);
    xTaskCreate(SerialReaderTask, "serial", 4096, NULL, 1, &serialReaderHandle);
    xTaskCreate(DisplayScreenTask, "displayScreen", 4096, NULL, 1, &displayScreenHandle);
    //xTaskCreate(WifiTask, "wifi", 4096, NULL, 1, &wifiHandle);
    //xTaskCreate(ServerTask, "server", 4096, NULL, 1, &serverHandle);
    xTaskCreate(RadioTask, "radio", 4096, NULL, 3, &radioHandle);
}

void loop() {
    vTaskDelete(NULL);
}

