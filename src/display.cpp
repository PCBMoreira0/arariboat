#include <Arduino.h>
#include <Wire.h> 
#include "SSD1306Wire.h"  // Library for controlling the 128x64 OLED display.
#include "WiFiManager.hpp" // Proxy class to get Wifi data without exposing implementation details
#include "RadioManager.hpp" // Proxy class to get Radio data without exposing implementation details
#include "arariboat/mavlink.h" // Mavlink library for communication with the ground station1

enum Page {
    Home,
    Wifi,
    Lora,
    NumberPages
};

static void ShowHomeScreen(SSD1306Wire& screen) {
    constexpr uint8_t vertical_offset = 16; // Adjust for the height of the font so that the text is centered
    screen.clear();
    screen.setFont(ArialMT_Plain_16);
    screen.setTextAlignment(TEXT_ALIGN_CENTER);
    screen.drawString(screen.getWidth() / 2, (screen.getHeight() - vertical_offset) / 2, "Lora Receiver");
    screen.display();
}

static void ShowWifiScreen(SSD1306Wire& screen) {

    String ssid = WifiManager::GetInstance().GetSSID(); 
    String ip = WifiManager::GetInstance().GetIP(); 
    int8_t rssi = WifiManager::GetInstance().GetRSSI(); 
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
        case -100 ... -81:
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
}

static void ShowLoraScreen(SSD1306Wire& screen) {
    screen.clear();
    screen.setFont(ArialMT_Plain_10);
    screen.setTextAlignment(TEXT_ALIGN_LEFT);

    String message = RadioManager::GetInstance().GetLastMessageName();
    String rssi = String(RadioManager::GetInstance().GetRSSI());
    String sequence = String(RadioManager::GetInstance().GetSequence());

    screen.drawString((screen.getWidth() - 40) / 2, 0, "[Lora]");
    screen.drawString(0, 15, "Message: " + message);
    screen.drawString(0, 25, "RSSI: " + rssi);
    screen.drawString(0, 35, "RX Seq: " + sequence);
    screen.display();
}

static int SetPageInterval(Page page) {
    int interval;
    switch (page) {
        case Home:
            interval = 2000;
            break;
        case Wifi:
            interval = 2000;
            break;
        case Lora:
            interval = 45000;
            break;
    }
    return interval;
}

static void SetPage(SSD1306Wire& screen, Page page) {
    switch (page) {
        case Home:
            ShowHomeScreen(screen);
            break;
        case Wifi:
            ShowWifiScreen(screen);
            break;
        case Lora:
            ShowLoraScreen(screen);
            break;
    }
}

/// @brief Task that controls the the 128x64 OLED display via I2C communication.
/// @param parameter 
void DisplayScreenTask(void* parameter) {

    //Begins the integrated 128x64 OLED display with the I2C Wire interface.
    //The display is cleared and an introduction message is displayed.
    //The screen is also capable of drawing lines, rectangles, circles, and bitmaps.

    constexpr uint8_t address = 0x3C;
    constexpr uint8_t sda_pin = GPIO_NUM_21;
    constexpr uint8_t scl_pin = GPIO_NUM_22;
    SSD1306Wire screen(address, sda_pin, scl_pin); // SSD1306 128x64 OLED display with I2C Wire interface

    screen.init();
    screen.flipScreenVertically(); // Rotate screen to get correct orientation
    constexpr uint16_t update_rate = 500;
    static uint32_t last_update_time = 0;
    int page_interval;
    
    while (true) {
        for (int i = 0; i < NumberPages; i++) {
            page_interval = SetPageInterval(static_cast<Page>(i));
            
            while (millis() - last_update_time < page_interval) {
                SetPage(screen, static_cast<Page>(i));
                vTaskDelay(pdMS_TO_TICKS(update_rate));
            }
            last_update_time = millis();
        }
    }
}