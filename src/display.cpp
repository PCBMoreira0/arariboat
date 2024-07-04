#include <Arduino.h>
#include <Wire.h> 
#include "SSD1306Wire.h"  // Library for controlling the 128x64 OLED display.
#include "WiFiManager.hpp" // Proxy class to get Wifi data without exposing implementation details
#include "arariboat/mavlink.h" // Mavlink library for communication with the ground station

/// @brief Task that controls the the 128x64 OLED display via I2C communication.
/// @param parameter 
void DisplayScreenTask(void* parameter) {

    //Begins the integrated 128x64 OLED display with the I2C Wire interface.
    //The display is cleared and an introduction message is displayed.
    //The screen is also capable of drawing lines, rectangles, circles, and bitmaps.

    constexpr uint8_t address = 0x3C;
    constexpr uint8_t sda_pin = 21;
    constexpr uint8_t scl_pin = 22;
    SSD1306Wire screen(address, sda_pin, scl_pin); // SSD1306 128x64 OLED display with I2C Wire interface

    vTaskDelay(5000);

    enum Page {
        Home,
        Wifi,
        Lora,
        NumberPages
    };

    auto ShowHomeScreen = [&]() {
        constexpr uint8_t vertical_offset = 16; // Adjust for the height of the font so that the text is centered
        screen.clear();
        screen.setFont(ArialMT_Plain_16);
        screen.setTextAlignment(TEXT_ALIGN_CENTER);
        screen.drawString(screen.getWidth() / 2, (screen.getHeight() - vertical_offset) / 2, "Lora Transmitter");
        screen.display();
    };

    auto ShowWifiScreen = [&]() {

        String ssid = wifiManager.GetSSID();
        String ip = wifiManager.GetIP();
        int8_t rssi = wifiManager.GetRSSI();
        String rssi_state = "";
        switch(rssi) {
            case -30 ... 0:
                rssi_state = "Amazing";
                break;
            case -55 ... -31:
                rssi_state = "Very good signal";
                break;
            case -67 ... -56:
                rssi_state = "Fairly Good";
                break;
            case -70 ... -68:
                rssi_state = "Okay";
                break;
            case -80 ... -71:
                rssi_state = "Not good";
                break;
            case -90 ... -81:
                rssi_state = "Extremely weak signal (unusable)";
                break;
            default:
                rssi_state = "Unknown";
                break;
        }

        screen.clear();
        screen.setFont(ArialMT_Plain_10);
        screen.setTextAlignment(TEXT_ALIGN_LEFT);
        // Display WiFi status
        screen.drawString((screen.getWidth() - 40) / 2, 0, "[WiFi]");
        screen.drawString(0, 15, "SSID: " + ssid);
        screen.drawString(0, 25, "IP: " + ip);
        screen.drawString(0, 35, "RSSI: " + String(rssi));
        screen.drawString(0, 45, "Signal: " + rssi_state);
        screen.display();
    };

    auto ShowMavlinkChannel = [&]() {
        screen.clear();
        screen.setFont(ArialMT_Plain_10);
        screen.setTextAlignment(TEXT_ALIGN_LEFT);
        // Display Lora status
        screen.drawString((screen.getWidth() - 40) / 2, 0, "[Mavlink]");
        screen.drawString(0, 15, "Channel 0");
        // Draw last id and sequence from mavlink_channel
        screen.drawString(0, 25, "Seq: " + String(mavlink_get_channel_status(MAVLINK_COMM_0)->current_rx_seq));
        screen.display();
    };

    screen.init();
    screen.flipScreenVertically(); // Rotate screen to get correct orientation
    
    uint32_t page_interval;
    static uint32_t last_update_time = millis();
    constexpr int32_t update_rate = 400;
    
    while (true) {
        for (int i = 0; i < NumberPages; i++) {
            switch (i) {
                case Home:
                    page_interval = 1500;
                    break;
                case Wifi:
                    page_interval = 3000;
                    break;
                case Lora:
                    page_interval = 5000;
                    break;
            }

            while (millis() - last_update_time < page_interval) {
                switch (i) {
                    case Home:
                        ShowHomeScreen();
                        break;
                    case Wifi:
                        ShowWifiScreen();
                        break;
                    case Lora:
                        ShowMavlinkChannel();
                        break;
                }
                vTaskDelay(pdMS_TO_TICKS(update_rate));
            }
            last_update_time = millis();
        }
    }
}