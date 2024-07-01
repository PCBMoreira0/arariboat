#include <Arduino.h> // Main Arduino library, required for projects that use the Arduino framework.
#include <Wire.h> // Required for the ADS1115 ADC and communication with the LoRa board.
#include "Utilities.hpp" // Custom utility macros and functions.

//TODO: Improve server interface for configuration and debug purposes.
//TODO: Implement auxiliary battery and pumps readings using some I2C system
//TODO: Send data directly to InfluxDB instead of using Husarnet
//TODO: Save all measurements to a file in the SPIFFS file system or some microSD card, then flush it to InfluxDB when the connection is available
//TODO: Use IDF event loop for serial parsing and intertask communication
//TODO: Implement RPM measurements code
//TODO: Assign better weights to task priorities via benchmarks


// Declare a handle for each task to allow manipulation of the task from other tasks, such as sending notifications, resuming or suspending.
// The handle is initialized to nullptr to avoid the task being created before the setup() function.
// Each handle is then assigned to the task created in the setup() function.

TaskHandle_t ledBlinkerTaskHandle = nullptr;
TaskHandle_t wifiTaskHandle = nullptr;
TaskHandle_t serverTaskHandle = nullptr;
TaskHandle_t VPNTaskHandle = nullptr;
TaskHandle_t serialReaderTaskHandle = nullptr;
TaskHandle_t temperatureReaderTaskHandle = nullptr;
TaskHandle_t gpsReaderTaskHandle = nullptr;
TaskHandle_t instrumentationReaderTaskHandle = nullptr;
TaskHandle_t timeReaderTaskHandle = nullptr;

void setup() {

    Serial.begin(9600); // Lower baud rates are required as cable length increases.
    InitializeEventLoop(&eventLoop); // Initialize the event loop to handle events between tasks.

    Wire.begin(); // I2C master mode to communicate with the ADS1115 ADC
    xTaskCreate(LedBlinkerTask, "ledBlinker", 2048, NULL, 1, &ledBlinkerTaskHandle);
    xTaskCreate(WifiConnectionTask, "wifiConnection", 4096, NULL, 1, &wifiTaskHandle);
    //xTaskCreate(ServerTask, "server", 4096, NULL, 1, &serverTaskHandle);
    xTaskCreate(SerialReaderTask, "serialReader", 4096, NULL, 1, &serialReaderTaskHandle);
    xTaskCreate(TemperatureReaderTask, "temperatureReader", 4096, NULL, 1, &temperatureReaderTaskHandle);
    //xTaskCreate(GPSReaderTask, "gpsReader", 4096, NULL, 1, &gpsReaderTaskHandle);
    //xTaskCreate(InstrumentationReaderTask, "instrumentationReader", 4096, NULL, 3, &instrumentationReaderTaskHandle);
    xTaskCreate(TimeReaderTask, "timeReader", 4096, NULL, 1, &timeReaderTaskHandle);
}

void loop() {
    vTaskDelete(NULL);
}


