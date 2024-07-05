#include <Arduino.h>
#include <WiFi.h> // Main library for WiFi connectivity, also used by AsyncWebServer.
#include <unordered_map> // Hashtable for storing WiFi credentials.
#include "ESPAsyncWebServer.h" // Make sure to include Husarnet before this.
#include "AsyncElegantOTA.h" // Over the air updates for the ESP32.
#include "ESPmDNS.h" // Required for mDNS service discovery.
#include "Utilities.hpp" // Custom utility macros and functions.

static void serialCommandCallback(void* handler_args, esp_event_base_t base, int32_t id, void* event_data) {
    
    const char* command = (const char*)event_data;

    if (strncmp(command, "wifi", 4) == 0) {
        Serial.printf("\n[WIFI]Reading WiFi data\n");
        Serial.printf("\n[WIFI]Connected to %s with IP address %s\n", WiFi.SSID().c_str(), WiFi.localIP().toString().c_str());
    }
}

void WifiTask(void* parameter) {
    
    std::unordered_map<const char*, const char*> wifiCredentials;
    wifiCredentials["EMobil 1"] = "faraboia";
    wifiCredentials["Innorouter"] = "innomaker";
    wifiCredentials["NITEE"] = "nitee123";

    // Register a callback function to handle WiFi events. This function is called when the WiFi status changes.
    WiFi.onEvent([](WiFiEvent_t event, WiFiEventInfo_t info) {
        switch (event) {
        case SYSTEM_EVENT_STA_GOT_IP:
            Serial.printf("\n[WIFI]Connected to %s with IP address %s\n", WiFi.SSID().c_str(), WiFi.localIP().toString().c_str());
            break;
        default:
            break;
        }
    });

    //Register serial callback commands
    esp_event_handler_register_with(eventLoop, SERIAL_PARSER_EVENT_BASE, ESP_EVENT_ANY_ID, serialCommandCallback, nullptr); 
    
    while (true) {

        if (WiFi.status() == WL_CONNECTED) {
            vTaskDelay(pdMS_TO_TICKS(400));
            continue;
        }

        WiFi.mode(WIFI_STA);
        if (ledBlinkerHandle != nullptr) {
            xTaskNotify(ledBlinkerHandle, BlinkRate::Fast, eSetValueWithOverwrite);
        }

        for (auto& wifi : wifiCredentials) {
            WiFi.begin(wifi.first, wifi.second);
            Serial.printf("\n[WIFI]Trying to connect to %s\n", wifi.first);
            int attempts = 0;
            while (WiFi.status() != WL_CONNECTED) {
                vTaskDelay(pdMS_TO_TICKS(500));
                if (attempts++ > 5) break;
            }
            if (WiFi.status() == WL_CONNECTED) {
                if (ledBlinkerHandle != nullptr) {
                    xTaskNotify(ledBlinkerHandle, BlinkRate::Slow, eSetValueWithOverwrite);
                }
                break;
            }
        }
    }
}

// Configure the mDNS responder to allow the ESP32 to be discovered on the network by its hostname.
void ConfigureMDNS() {
    
    // Start the mDNS responder. This allows the ESP32 to be discovered on the network by its hostname.
    if (!MDNS.begin(STRINGIFY(MDNS_HOSTNAME))) {
        Serial.println("Error setting up MDNS responder!");
        while (true) {
            vTaskDelay(1000);
        }
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

    while (WiFi.status() != WL_CONNECTED) {
        vTaskDelay(pdMS_TO_TICKS(1000));
    }

    ConfigureMDNS();
    
    // Initialize the OTA update service. This service allows the ESP32 to be updated over the air.
    AsyncElegantOTA.begin(&server); // Available at http://[esp32ip]/update or http://[esp32hostname]/update
    server.begin();

    while (true) {
        vTaskDelay(500);
    }
}
