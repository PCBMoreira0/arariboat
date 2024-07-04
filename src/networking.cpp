#include <Arduino.h>
#include <WiFi.h> // Main library for WiFi connectivity, also used by AsyncWebServer.
#include <unordered_map> // Hashtable for storing WiFi credentials.
#include "ESPAsyncWebServer.h" // Make sure to include Husarnet before this.
#include "AsyncElegantOTA.h" // Over the air updates for the ESP32.
#include "ESPmDNS.h" // Required for mDNS service discovery.
#include "Utilities.hpp" // Custom utility macros and functions.
#include "LoraConfigManager.hpp" // Non-volatile storage for system parameters
#include "LoraRequestHandler.hpp" // HTTP request handler for LoRa configuration
#include "WiFiManager.hpp" // Wrapper class for WiFi management

static void serialCommandCallback(void* handler_args, esp_event_base_t base, int32_t id, void* event_data) {
    
    const char* command = (const char*)event_data;

    if (strncmp(command, "wifi", 4) == 0) {
        Serial.printf("\n[WIFI]Reading WiFi data\n");
        Serial.printf("\n[WIFI]Connected to %s with IP address %s\n", WiFi.SSID().c_str(), WiFi.localIP().toString().c_str());
    }
}

static void ConfigureWifiCallbacks(WifiManager& wifiManager) {
    wifiManager.SetSSIDCallback([&]() {
        return WiFi.SSID();
    });

    wifiManager.SetIPCallback([&]() {
        return WiFi.localIP().toString();
    });

    wifiManager.SetRSSICallback([&]() {
        return WiFi.RSSI();
    });
}

void WifiTask(void* parameter) {
    
    std::unordered_map<const char*, const char*> wifiCredentials;
    wifiCredentials["EMobil 1"] = "faraboia";
    wifiCredentials["Ararirouter"] = "arariboia";
    wifiCredentials["NITEE"] = "nitee123";
    wifiCredentials["Viana"] = "1040441000";

    ConfigureWifiCallbacks(wifiManager);

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

void addRoutes(AsyncWebServer& server) {
    server.on("/", HTTP_GET, [](AsyncWebServerRequest *request) {
        request->send(200, "text/html", "<h1>Boat-Lora</h1><p>WiFi connected: " + WiFi.SSID() + "</p><p>IP address: " + WiFi.localIP().toString() + "</p>");
    });

    server.on("/reset", HTTP_GET, [](AsyncWebServerRequest *request) {
        // log reset message
        request->send(200, "text/html", "<h1>Boat-Lora</h1><p>Resetting...</p>");
        vTaskDelay(pdMS_TO_TICKS(1000));
        ESP.restart();
    });

    server.on("/lora-config", HTTP_POST, handleLoraConfigRequest);
}

void ServerTask(void* parameter) {

    // Create an async web server on port 80. This is the default port for HTTP. 
    // Async server can handle multiple requests at the same time without blocking the task.
    AsyncWebServer server(80);
    
    // Setup URL routes and attach callback methods to them. A callback method is called when a request is made to the URL.
    // The callbacks must have the signature void(AsyncWebServerRequest *request). Any function with this signature can be used.
    // Preferably, use lambda functions to keep the code in the same place.
    addRoutes(server);

    while (WiFi.status() != WL_CONNECTED) {
        vTaskDelay(pdMS_TO_TICKS(1000));
    }

    while (!MDNS.begin("boat-radio")) {
        Serial.println("Error setting up MDNS responder!");
        vTaskDelay(1000);
    }
    
    // Initialize the OTA update service. This service allows the ESP32 to be updated over the air.
    AsyncElegantOTA.begin(&server); // Available at http://[esp32ip]/update or http://[esp32hostname]/update
    server.begin();

    while (true) {
        vTaskDelay(500);
    }
}
