#include <Arduino.h> // Main Arduino library, required for projects that use the Arduino framework.
#include <ESP32Time.h> // Internal RTC timer of ESP32 to keep track of HH:MM:SS
#include <WiFi.h> // WiFi library

void TimeReaderTask(void *parameter) {

    while (WiFi.status() != WL_CONNECTED) {
        vTaskDelay(pdMS_TO_TICKS(1000));
    }

    //Change those values when moving to another timezone
    ESP32Time RTC = ESP32Time();
    constexpr long gmtSecOffset = 0;
    constexpr long daylightSecOffset = 0;

    /*---------set with NTP---------------*/
    configTime(gmtSecOffset, daylightSecOffset, "south-america.pool.ntp.org");
    struct tm timeinfo;
    while (!getLocalTime(&timeinfo)) {
		Serial.println("[TIME]Failed to obtain network time");
		vTaskDelay(pdMS_TO_TICKS(15000));
	}

    Serial.println("[TIME]Network time obtained");
	RTC.setTimeStruct(timeinfo); 
    //xEventGroupSetBits(systemEventGroup, TIMESTAMP_ACQUIRED_BIT); // Data acquisition can only start after the timestamp is acquired.

	/*------------------------------------*/

    while (true) {

        String timestamp = RTC.getTime("%Y%m%d%H%M%S");
        unsigned long local_epoch_seconds = RTC.getEpoch() + gmtSecOffset;
        //systemData.SetTimestamp(String(local_epoch_seconds));
        //Serial.printf("[TIME] %s\tLocal Epoch: %lu\n", timestamp.c_str(), local_epoch_seconds);
        vTaskDelay(pdMS_TO_TICKS(1000));
    }
}
