#include <Arduino.h>
#include <WiFi.h> // Main library for WiFi connectivity, also used by AsyncWebServer.
#include <unordered_map> // Hashtable for storing WiFi credentials.
#include "ESPAsyncWebServer.h" // Make sure to include Husarnet before this.
#include "AsyncElegantOTA.h" // Over the air updates for the ESP32.
#include "ESPmDNS.h" // Required for mDNS service discovery.
#include "Utilities.hpp" // Custom utility macros and functions.
#include "InfluxDB.hpp" // Custom InfluxDB client for sending data to InfluxDB.

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
        float voltage_battery = SystemData::getInstance().all_info.battery_voltage;
        float current_motor_left = SystemData::getInstance().all_info.motor_current_left;
        float current_motor_right = SystemData::getInstance().all_info.motor_current_right;
        float current_mppt = SystemData::getInstance().all_info.mppt_current;
        float temperature_battery_left = SystemData::getInstance().all_info.temperature_battery_left;
        float temperature_battery_right = SystemData::getInstance().all_info.temperature_battery_right;
        float temperature_mppt = SystemData::getInstance().all_info.temperature_mppt;
        float latitude = SystemData::getInstance().all_info.latitude;
        float longitude = SystemData::getInstance().all_info.longitude;
        float rpm_left = SystemData::getInstance().all_info.rpm_left;
        float rpm_right = SystemData::getInstance().all_info.rpm_right;
        uint64_t timestamp = SystemData::getInstance().all_info.timestamp;

        String response = "<h1>Boat32</h1>";
        response += "<p>Voltage Battery: " + String(voltage_battery) + " V</p>";
        response += "<p>Current Motor Left: " + String(current_motor_left) + " A</p>";
        response += "<p>Current Motor Right: " + String(current_motor_right) + " A</p>";
        response += "<p>Current MPPT: " + String(current_mppt) + " A</p>";
        response += "<p>Temperature Battery Left: " + String(temperature_battery_left) + " °C</p>";
        response += "<p>Temperature Battery Right: " + String(temperature_battery_right) + " °C</p>";
        response += "<p>Temperature MPPT: " + String(temperature_mppt) + " °C</p>";
        response += "<p>Latitude: " + String(latitude) + "</p>";
        response += "<p>Longitude: " + String(longitude) + "</p>";
        response += "<p>RPM Left: " + String(rpm_left) + "</p>";
        response += "<p>RPM Right: " + String(rpm_right) + "</p>";
        response += "<p>Timestamp: " + String(timestamp) + "</p>";

        request->send(200, "text/html", response);
    });

    while (WiFi.status() != WL_CONNECTED) {
        vTaskDelay(pdMS_TO_TICKS(1000));
    }

    ConfigureMDNS();
    
    // Initialize the OTA update service. This service allows the ESP32 to be updated over the air.
    AsyncElegantOTA.begin(&server); // Available at http://[esp32ip]/update or http://[esp32hostname]/update
    server.begin();

    while (true) {
        String line_protocol = SystemData::getInstance().GetLineProtocol();
        //DEBUG_PRINTF("\n[HTTP]]Sending Data: \n%s", line_protocol.c_str());
        postToInfluxDB(line_protocol);
        vTaskDelay(2000);
    }
}
