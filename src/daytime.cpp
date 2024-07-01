#include <Arduino.h> // Main Arduino library, required for projects that use the Arduino framework.
#include <ESP32Time.h> // Internal RTC timer of ESP32 to keep track of HH:MM:SS
#include <WiFi.h> // WiFi library
#include "Utilities.hpp" // Custom utilities for the project

static void serialCommandCallback(void* handler_args, esp_event_base_t base, int32_t id, void* event_data) {
    
    const char* command = (const char*)event_data;

    if (strncmp(command, "time", 4) == 0) {
        ESP32Time* RTC = (ESP32Time*)handler_args;
        String timestamp = RTC->getTime("%Y%m%d%H%M%S");
        unsigned long local_epoch_seconds = RTC->getEpoch();
        Serial.printf("[TIME] %s\tLocal Epoch: %lu\n", timestamp.c_str(), local_epoch_seconds);

    }
}


void TimeReaderTask(void *parameter) {

    auto logger = Logger("TimeReaderTask started");

    while (WiFi.status() != WL_CONNECTED) {
        logger.log("[TIME]Waiting for WiFi connection...", 3000);
        vTaskDelay(pdMS_TO_TICKS(1000));
    }
    logger.stop();

    //Change those values when moving to another timezone
    ESP32Time RTC = ESP32Time();
    constexpr long gmtSecOffset = 0;
    constexpr long daylightSecOffset = 0;

    /*---------set with NTP---------------*/
    configTime(gmtSecOffset, daylightSecOffset, "south-america.pool.ntp.org");
    struct tm timeinfo;
    while (!getLocalTime(&timeinfo)) {
		logger.log("[TIME]Failed to obtain network time", 12000);
		vTaskDelay(pdMS_TO_TICKS(3000));
	}
    logger.stop();
    logger.log("[TIME]Network time obtained");

	RTC.setTimeStruct(timeinfo); 
    //xEventGroupSetBits(systemEventGroup, TIMESTAMP_ACQUIRED_BIT); // Data acquisition can only start after the timestamp is acquired.

    esp_event_handler_register_with(eventLoop, SERIAL_PARSER_EVENT_BASE, ESP_EVENT_ANY_ID, serialCommandCallback, &RTC);

	/*------------------------------------*/

    while (true) {

        String timestamp = RTC.getTime("%Y%m%d%H%M%S");
        unsigned long local_epoch_seconds = RTC.getEpoch() + gmtSecOffset;
        //systemData.SetTimestamp(String(local_epoch_seconds));
        Serial.printf("[TIME] %s\tLocal Epoch: %lu\n", timestamp.c_str(), local_epoch_seconds);
        vTaskDelay(pdMS_TO_TICKS(5000));
    }
}
