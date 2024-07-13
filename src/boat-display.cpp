#include <Arduino.h> // Main Arduino library, required for projects that use the Arduino framework.
#include "DisplaySetup.hpp"  // Display settings
#include "Utilities.hpp" // Custom utility macros and functions.

// Declare a handle for each task to allow manipulation of the task from other tasks, such as sending notifications, resuming or suspending.
// The handle is initialized to nullptr to avoid the task being created before the setup() function.
// Each handle is then assigned to the task created in the setup() function.

TaskHandle_t ledBlinkerHandle = nullptr;
TaskHandle_t wifiConnectionHandle = nullptr;
TaskHandle_t serverTaskHandle = nullptr;
TaskHandle_t serialReaderHandle = nullptr;
TaskHandle_t cockpitDisplayHandle = nullptr;


void setup() {

    Serial.begin(BAUD_RATE);
    InitializeEventLoop(&eventLoop);
    xTaskCreate(LedBlinkerTask, "ledBlinker", 2048, NULL, 1, &ledBlinkerHandle);
    xTaskCreate(WifiTask, "wifiConnection", 4096, NULL, 1, &wifiConnectionHandle);
    xTaskCreate(ServerTask, "server", 4096, NULL, 4, &serverTaskHandle);
    xTaskCreate(SerialReaderTask, "serialReader", 4096, NULL, 1, &serialReaderHandle);
    DisplayInit();
    xTaskCreate(CockpitDisplayTask, "cockpitDisplay", 4096, NULL, 1, &cockpitDisplayHandle);  
}

void loop() {
    vTaskDelete(NULL);
}


