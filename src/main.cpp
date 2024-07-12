#include <Arduino.h> // Main Arduino library, required for projects that use the Arduino framework.
#include "Utilities.hpp" // Custom utility macros and functions.

//TODO: Improve server interface for configuration and debug purposes.
//TODO: Implement auxiliary battery and pumps readings using some I2C system
//TODO: Send data directly to InfluxDB instead of using Husarnet
//TODO: Save all measurements to a file in the SPIFFS file system or some microSD card, then flush it to InfluxDB when the connection is available
//TODO: Implement RPM measurements code
//TODO: Assign better weights to task priorities via benchmarks


// Declare a handle for each task to allow manipulation of the task from other tasks, such as sending notifications, resuming or suspending.
// The handle is initialized to nullptr to avoid the task being created before the setup() function.
// Each handle is then assigned to the task created in the setup() function.

TaskHandle_t ledBlinkerHandle = nullptr;
TaskHandle_t wifiTaskHandle = nullptr;
TaskHandle_t serverTaskHandle = nullptr;
TaskHandle_t serialReaderTaskHandle = nullptr;
TaskHandle_t temperatureReaderTaskHandle = nullptr;
TaskHandle_t gpsReaderTaskHandle = nullptr;
TaskHandle_t instrumentationReaderTaskHandle = nullptr;
TaskHandle_t timeReaderTaskHandle = nullptr;
TaskHandle_t frequencyCounterTaskHandle = nullptr;

void setup() {

    InitializeEventLoop(&eventLoop); // Initialize the event loop to handle events between tasks.
    xTaskCreate(LedBlinkerTask, "ledBlinker", 2048, NULL, 1, &ledBlinkerHandle);
    xTaskCreate(WifiTask, "wifiConnection", 4096, NULL, 2, &wifiTaskHandle);
    xTaskCreate(ServerTask, "server", 8096, NULL, 3, &serverTaskHandle);
    xTaskCreate(TimeReaderTask, "timeReader", 4096, NULL, 2, &timeReaderTaskHandle);
    xTaskCreate(SerialReaderTask, "serialReader", 4096, NULL, 1, &serialReaderTaskHandle);
    xTaskCreate(TemperatureReaderTask, "temperatureReader", 4096, NULL, 1, &temperatureReaderTaskHandle);
    xTaskCreate(GPSReaderTask, "gpsReader", 4096, NULL, 1, &gpsReaderTaskHandle);
    xTaskCreate(InstrumentationReaderTask, "instrumentationReader", 4096, NULL, 3, &instrumentationReaderTaskHandle);
    xTaskCreate(FrequencyCounterTask, "frequencyCounter", 4096, NULL, 1, &frequencyCounterTaskHandle);
}

void loop() {
    SystemData::getInstance().WriteToSerial();
    vTaskDelay(1000);
}


