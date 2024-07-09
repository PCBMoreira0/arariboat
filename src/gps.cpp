#include <Arduino.h> // Main Arduino library, required for projects that use the Arduino framework.
#include "TinyGPSPlus.h" // GPS NMEA sentence parser.
#include "arariboat/mavlink.h" // Custom mavlink dialect for the boat generated using Mavgen tool.
#include "Utilities.hpp" // Custom utility macros and functions.

static void serialCommandCallback(void* handler_args, esp_event_base_t base, int32_t id, void* event_data) {
    
    const char* command = (const char*)event_data;

    if (strncmp(command, "gps", 3) == 0) {
        Serial.printf("\n[GPS]Reading GPS data\n");
        TinyGPSPlus* gps = (TinyGPSPlus*)handler_args;
        constexpr float invalid_value = -1.0f; // Begin the fields with arbitrated invalid value and update them if the gps data is valid.
        float latitude = invalid_value;
        float longitude = invalid_value;
        float speed = invalid_value;
        float course = invalid_value;
        uint8_t satellites = 0;

        if (gps->location.isValid()) {
            latitude = gps->location.lat();
            longitude = gps->location.lng();
            Serial.printf("[GPS]Latitude: %f, Longitude: %f\n", latitude, longitude);
        }
        if (gps->speed.isValid()) { 
            speed = gps->speed.kmph();
            Serial.printf("[GPS]Speed: %f\n", speed);
        }
        if (gps->course.isValid()) {
            course = gps->course.deg();
            Serial.printf("[GPS]Course: %f\n", course);
        }
        if (gps->satellites.isValid()) {
            satellites = gps->satellites.value();
            Serial.printf("[GPS]Satellites: %d\n", satellites);
        }

    }
}

void GPSReaderTask(void* parameter) {

    // Example of latitude: 40.741895 (north is positive)
    // Example of longitude: -73.989308 (west is negative)
    // The fifth decimal place is worth up to 1.1 m. The sixth decimal place is worth up to 11cm. And so forth.
    
    TinyGPSPlus gps; // Object that parses NMEA sentences from the NEO-6M GPS module
    constexpr uint8_t gps_rx_pin = GPIO_NUM_16;  
    constexpr uint8_t gps_tx_pin = GPIO_NUM_17; 
    constexpr int32_t baud_rate = 9600; // Fixed baud rate used by NEO-6M GPS module
    Serial2.begin(baud_rate, SERIAL_8N1, gps_rx_pin, gps_tx_pin);

    //Register serial callback commands
    esp_event_handler_register_with(eventLoop, SERIAL_PARSER_EVENT_BASE, ESP_EVENT_ANY_ID, serialCommandCallback, &gps); 

    while (true) {
        while (Serial2.available()) {
            // Reads the serial stream from the NEO-6M GPS module and parses it into TinyGPSPlus object if a valid NMEA sentence is received
            if (gps.encode(Serial2.read())) { 
                constexpr float invalid_value = -1.0f; // Begin the fields with arbitrated invalid value and update them if the gps data is valid.
                float latitude = invalid_value;
                float longitude = invalid_value;

                if (gps.location.isValid()) {
                    latitude = gps.location.lat();
                    longitude = gps.location.lng();
                    DEBUG_PRINTF("[GPS]Latitude: %f, Longitude: %f\n", latitude, longitude);
                }
                
                SystemData::getInstance().all_info.latitude = latitude;
                SystemData::getInstance().all_info.longitude = longitude;
            }
        }           
        vTaskDelay(pdMS_TO_TICKS(500));
    }
}