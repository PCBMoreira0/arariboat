#include <Arduino.h>
#include <WiFi.h> // Main library for WiFi connectivity, also used by AsyncWebServer.
#include <unordered_map> // Hashtable for storing WiFi credentials.
#include "ESPAsyncWebServer.h" // Make sure to include Husarnet before this.
#include "AsyncElegantOTA.h" // Over the air updates for the ESP32.
#include "Utilities.hpp" // Custom utility macros and functions.

void WifiConnectionTask(void* parameter) {
    
    // Store WiFi credentials in a hashtable.
    std::unordered_map<const char*, const char*> wifiCredentials;
    wifiCredentials["Ursula"] = "biaviad36";
    wifiCredentials["EMobil 1"] = "faraboia";
    wifiCredentials["Innorouter"] = "innomaker";
    wifiCredentials["NITEE"] = "nitee123";
    
    while (true) {
        if (WiFi.status() != WL_CONNECTED) {
            WiFi.mode(WIFI_STA);
            if (ledBlinkerTaskHandle != nullptr) {
                xTaskNotify(ledBlinkerTaskHandle, BlinkRate::Fast, eSetValueWithOverwrite);
            }

            for (auto& wifi : wifiCredentials) {
                WiFi.begin(wifi.first, wifi.second);
                Serial.printf("\n[WIFI]Trying to connect to %s\n", wifi.first);
                int i = 0;
                while (WiFi.status() != WL_CONNECTED) {
                    vTaskDelay(pdMS_TO_TICKS(500));
                    Serial.print(".");
                    i++;
                    if (i > 5) {
                        Serial.printf("\n[WIFI]Failed to connect to %s\n", wifi.first);
                        break;
                    }
                }
                if (WiFi.status() == WL_CONNECTED) {
                    Serial.println("\n[WIFI]Connected to WiFi");
                    if (ledBlinkerTaskHandle != nullptr) {
                        xTaskNotify(ledBlinkerTaskHandle, BlinkRate::Slow, eSetValueWithOverwrite);
                    }
                    break;
                }
            }          
        }
        vTaskDelay(pdMS_TO_TICKS(5000));
    }
}

void ServerTask(void* parameter) {

    // Create an async web server on port 80. This is the default port for HTTP. 
    // Async server can handle multiple requests at the same time without blocking the task.
    AsyncWebServer server(80);
    
    // Setup URL routes and attach callback methods to them. A callback method is called when a request is made to the URL.
    // The callbacks must have the signature void(AsyncWebServerRequest *request). Any function with this signature can be used.
    // Preferably, use lambda functions to keep the code in the same place.
    server.on("/", HTTP_GET, [](AsyncWebServerRequest *request) {
        request->send(200, "text/html", "<h1>Boat32</h1><p>WiFi connected: " + WiFi.SSID() + "</p><p>IP address: " + WiFi.localIP().toString() + "</p>");
    });

    server.on("/reset", HTTP_GET, [](AsyncWebServerRequest *request) {
        // log reset message
        request->send(200, "text/html", "<h1>Boat32</h1><p>Resetting...</p>");
        vTaskDelay(pdMS_TO_TICKS(1000));
        ESP.restart();
    });

    server.on("/instrumentation", HTTP_GET, [](AsyncWebServerRequest *request) {
        // Send system instrumentation data from singleton class
        float current_motor = SystemData::getInstance().instrumentation.motor_current;
        float current_battery = SystemData::getInstance().instrumentation.battery_current;
        float current_mppt = SystemData::getInstance().instrumentation.mppt_current;
        float voltage_battery = SystemData::getInstance().instrumentation.battery_voltage;
        request->send(200, "text/html", "<h1>Boat32</h1><p>Current motor: " + String(current_motor) + "</p><p>Current battery: " + String(current_battery) + "</p><p>Current MPPT: " + String(current_mppt) + "</p><p>Voltage battery: " + String(voltage_battery) + "</p>");
    });
    
    server.on("/gps", HTTP_GET, [](AsyncWebServerRequest *request) {
        // Send GPS data from singleton class
        float latitude = SystemData::getInstance().gps.latitude;
        float longitude = SystemData::getInstance().gps.longitude;
        request->send(200, "text/html", "<h1>Boat32</h1><p>Latitude: " + String(latitude) + "</p><p>Longitude: " + String(longitude) + "</p>");
    });


    // Wait for notification from VPN connection task before starting the server.
    //ulTaskNotifyTake(pdTRUE, portMAX_DELAY);
    
    // Attach g update handler to the server and initialize the server.
    AsyncElegantOTA.begin(&server); // Available at http://[esp32ip]/update or http://[esp32hostname]/update
    server.begin();

    while (true) {
        ulTaskNotifyTake(pdTRUE, 500);
    }
}
