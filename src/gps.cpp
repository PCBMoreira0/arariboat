#include <Arduino.h> // Main Arduino library, required for projects that use the Arduino framework.
#include "TinyGPSPlus.h" // GPS NMEA sentence parser.
#include "arariboat/mavlink.h" // Custom mavlink dialect for the boat generated using Mavgen tool.
#include "Utilities.hpp" // Custom utility macros and functions.

void GPSReaderTask(void* parameter) {

    // Example of latitude: 40.741895 (north is positive)
    // Example of longitude: -73.989308 (west is negative)
    // The fifth decimal place is worth up to 1.1 m. The sixth decimal place is worth up to 11cm. And so forth.
    
    // Three hardware serial ports are available on the ESP32 with configurable GPIOs.
    // Serial0 is used for debugging and is connected to the USB-to-serial converter. Therefore, Serial1 and Serial2 are available.
    
    TinyGPSPlus gps; // Object that parses NMEA sentences from the NEO-6M GPS module
    constexpr uint8_t gps_rx_pin = GPIO_NUM_16;  
    constexpr uint8_t gps_tx_pin = GPIO_NUM_17; 
    constexpr int32_t baud_rate = 9600; // Fixed baud rate used by NEO-6M GPS module
    Serial2.begin(baud_rate, SERIAL_8N1, gps_rx_pin, gps_tx_pin); // Initialize Serial2 with the chosen baud rate and pins
   
    while (true) {
        while (Serial2.available()) {
            // Reads the serial stream from the NEO-6M GPS module and parses it into TinyGPSPlus object if a valid NMEA sentence is received
            if (gps.encode(Serial2.read())) { 
                constexpr float invalid_value = -1.0f; // Begin the fields with arbitrated invalid value and update them if the gps data is valid.
                float latitude = invalid_value;
                float longitude = invalid_value;
                float speed = invalid_value;
                float course = invalid_value;
                uint8_t satellites = 0;

                if (gps.location.isValid()) {
                    latitude = gps.location.lat();
                    longitude = gps.location.lng();
                    DEBUG_PRINTF("[GPS]Latitude: %f, Longitude: %f\n", latitude, longitude);
                }
                if (gps.speed.isValid()) {
                    speed = gps.speed.kmph();
                    DEBUG_PRINTF("[GPS]Speed: %f\n", speed);
                }
                if (gps.course.isValid()) {
                    course = gps.course.deg();
                    DEBUG_PRINTF("[GPS]Course: %f\n", course);
                }
                if (gps.satellites.isValid()) {
                    satellites = gps.satellites.value();
                    if (!satellites) break; // If no satellites are visible, the gps data is invalid. Break from the function.
                    DEBUG_PRINTF("[GPS]Satellites: %d\n", satellites);
                }

                // Prepare and send mavlink message by encoding the payload into a struct, then encoding the struct into a mavlink message below.
                mavlink_message_t message;
                mavlink_gps_info_t gps_info = {
                    latitude,
                    longitude,
                    speed,
                    course,
                    satellites
                };
                    
                mavlink_msg_gps_info_encode_chan(1, MAV_COMP_ID_ONBOARD_COMPUTER, MAVLINK_COMM_0, &message, &gps_info);
                uint8_t buffer[MAVLINK_MAX_PACKET_LEN];
                uint16_t length = mavlink_msg_to_send_buffer(buffer, &message);
                Serial.write(buffer, length);
            }
        }           
        vTaskDelay(pdMS_TO_TICKS(6000));
    }
}