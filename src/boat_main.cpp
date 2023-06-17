#include <Arduino.h>
#include <WiFi.h> // Async Web Server
#include "HTTPClient.h"
#include "HttpClientFunctions.hpp"
#include <Husarnet.h>
#include <ESPAsyncWebServer.h> // Make sure to include Husarnet before this.
#include <AsyncElegantOTA.h>
#include <unordered_map>
#include "DallasTemperature.h"
#include "TinyGPSPlus.h"


TaskHandle_t ledBlinkerTask = nullptr;
TaskHandle_t wifiConnectionTask = nullptr;
TaskHandle_t serverTask = nullptr;
TaskHandle_t vpnConnectionTask = nullptr;
TaskHandle_t serialReaderTask = nullptr;
TaskHandle_t temperatureReaderTask = nullptr;
TaskHandle_t gpsReaderTask = nullptr;
TaskHandle_t highWaterMeasurerTask = nullptr;

TaskHandle_t* taskHandles[] = { &ledBlinkerTask, &wifiConnectionTask, &serverTask, &vpnConnectionTask, &serialReaderTask, &temperatureReaderTask, &gpsReaderTask, &highWaterMeasurerTask};
constexpr auto taskHandlesSize = sizeof(taskHandles) / sizeof(taskHandles[0]);

constexpr int ledPin = 2;

bool canRequest = false;

enum BlinkRate : uint32_t {
    Slow = 1000,
    Medium = 500,
    Fast = 100,
    Pulse = 100
};

enum GPSPrintOptions {
    Off = '0',
    Raw,
    Parsed
};

void FastBlinkPulse();
void LedBlinker(void* parameter) {

    pinMode(ledPin, OUTPUT);
    uint32_t blinkRate = *((uint32_t*)parameter);
    uint32_t previousBlinkRate = blinkRate;
    
    while (true) {
        digitalWrite(ledPin, HIGH);
        vTaskDelay(pdMS_TO_TICKS(blinkRate));
        digitalWrite(ledPin, LOW);
        vTaskDelay(pdMS_TO_TICKS(blinkRate));
        
        // Set blink rate to the value received from the notification
        if (xTaskNotifyWait(0, 0, (uint32_t*)&blinkRate, 0) == pdTRUE) {
            Serial.printf("Received notification to change blink rate to %d\n", blinkRate);
            if (blinkRate == BlinkRate::Pulse) {
                FastBlinkPulse();
                blinkRate = previousBlinkRate;
            }
        }
    }
}

void FastBlinkPulse() {
    // 10 pulses
    for (int i = 0; i < 10; i++) {
        digitalWrite(ledPin, HIGH);
        vTaskDelay(pdMS_TO_TICKS(50));
        digitalWrite(ledPin, LOW);
        vTaskDelay(pdMS_TO_TICKS(50));
    }
}

// WifiConnection Task
void WifiConnectionTask(void* parameter) {

    std::unordered_map<const char*, const char*> wifiCredentials;
    wifiCredentials["Ursula"] = "biaviad36";
    wifiCredentials["EMobil 1"] = "faraboia";
    wifiCredentials["Innorouter"] = "innomaker";
    wifiCredentials["NITEE"] = "nitee123";
    
    while (true) {
        if (WiFi.status() != WL_CONNECTED) {
            WiFi.mode(WIFI_STA);
            xTaskNotify(ledBlinkerTask, BlinkRate::Fast, eSetValueWithOverwrite);
            for (auto& wifi : wifiCredentials) {
                WiFi.begin(wifi.first, wifi.second);
                Serial.printf("Trying to connect to %s\n", wifi.first);
                int i = 0;
                while (WiFi.status() != WL_CONNECTED) {
                    vTaskDelay(pdMS_TO_TICKS(500));
                    Serial.print(".");
                    i++;
                    if (i > 5) {
                        Serial.printf("Failed to connect to %s\n", wifi.first);
                        break;
                    }
                }
                if (WiFi.status() == WL_CONNECTED) {
                    Serial.println("Connected to WiFi");
                    xTaskNotify(ledBlinkerTask, BlinkRate::Slow, eSetValueWithOverwrite);
                    xTaskNotifyGive(serverTask);
                    break;
                }
            }          
        }
        vTaskDelay(pdMS_TO_TICKS(5000));
    }
}

void ServerTask(void* parameter) {

    AsyncWebServer server(80);
    server.on("/", HTTP_GET, [](AsyncWebServerRequest *request) {
        request->send(200, "text/html", "<h1>Boat32</h1><p>WiFi connected: " + WiFi.SSID() + "</p><p>IP address: " + WiFi.localIP().toString() + "</p>");
    });

    server.on("/reset", HTTP_GET, [](AsyncWebServerRequest *request) {
        // log reset message
        request->send(200, "text/html", "<h1>Boat32</h1><p>Resetting...</p>");
        vTaskDelay(pdMS_TO_TICKS(1000));
        ESP.restart();
    });

    // Wait for notification from WifiConnection task that wifi is connected in order to begin the server
    ulTaskNotifyTake(pdTRUE, portMAX_DELAY);
    
    AsyncElegantOTA.begin(&server); // Available at http://[esp32ip]/update
    server.begin();
    xTaskNotifyGive(vpnConnectionTask); // Notify VPN connection task that server is running

    while (true) {
        if (canRequest) {
            canRequest = false;
            // Get home host from husarnet list of peers
            long randomTestValue = random(0, 10);
            String targetURL = "/ScadaBR/httpds?voltage=" + String(randomTestValue);
            IPv6Address ipv6;
            String hostname = "";
            auto peers = Husarnet.listPeers();

            for (const auto& peer : peers) {
                ipv6 = peer.first;
                hostname = peer.second;
                Serial.printf("Peer: %s, %s\n", ipv6.toString().c_str(), hostname.c_str());
                if (hostname == String("home")) {
                    break;
                }
            }
            
            if (hostname == "" || hostname == nullptr) {
                Serial.println("Home host not found");
                vTaskDelay(pdMS_TO_TICKS(500));
                continue;
            }

            AsyncClient* client = new AsyncClient();
            
            client->onError([](void* arg, AsyncClient* client, int error) {
                Serial.printf("Error: %s\n", client->errorToString(error));
                client->close();
            }, nullptr);

            client->onConnect([targetURL, ipv6](void* arg, AsyncClient* client) {
                Serial.println("Connected");
                String fullURL = "GET " + targetURL + " HTTP/1.1\r\nHost: [" + ipv6.toString() + "]\r\nConnection: close\r\n\r\n";
                Serial.printf("Sending request: %s\n", fullURL.c_str());
                client->write(fullURL.c_str(), strlen(fullURL.c_str()));
            }, nullptr);

            client->onDisconnect([](void* arg, AsyncClient* client) {
                Serial.println("Disconnected");
                client->close();
                delete client;
            }, nullptr);

            client->onTimeout([](void* arg, AsyncClient* client, int32_t time) {
                Serial.printf("Timeout: %d\n", time);
                client->close();
            }, nullptr);

            client->onAck([](void* arg, AsyncClient* client, size_t len, int32_t time) {
                Serial.printf("Ack: %d\n", time);
                
            }, nullptr);

            client->onData([](void* arg, AsyncClient* client, void* data, size_t len) {
                Serial.printf("Data: %s\n", (char*)data);
            }, nullptr);

            client->connect(ipv6, 80);

        }
        vTaskDelay(pdMS_TO_TICKS(500));
    }
}

void VpnConnectionTask(void* parameter) {

    const char* hostName = "boat32";
    const char* husarnetJoinCode = "fc94:b01d:1803:8dd8:b293:5c7d:7639:932a/YNqd5m2Bjp65Miucf9R95p";
    const char* dashboardURL = "default";

    // Wait for notification from Server task that server is running in order to begin the VPN connection
    ulTaskNotifyTake(pdTRUE, portMAX_DELAY);
    Husarnet.selfHostedSetup(dashboardURL);
    Husarnet.join(husarnetJoinCode, hostName);
    Husarnet.start();
    while (true) {
        vTaskDelay(pdMS_TO_TICKS(2000));
    }
}

template <std::size_t N>
void ProcessSerialMessage(const std::array<uint8_t, N> &buffer);
void SerialReaderTask(void* parameter) {
    
    std::array<uint8_t, 32> buffer = { 0 };
    static size_t bufferIndex = 0;
    while (true) {
        if (Serial.available()) {
            uint8_t receivedChar = Serial.read();
            switch (receivedChar) {
                case '\r':
                case '\n':
                    ProcessSerialMessage(buffer);
                    bufferIndex = 0;
                    buffer.fill(0);
                    break;
                default:
                    if (bufferIndex == buffer.size()) {
                        ProcessSerialMessage(buffer);
                        bufferIndex = 0;
                        buffer.fill(0);
                        break;
                    }
                    buffer[bufferIndex++] = receivedChar;
                    buffer[bufferIndex] = 0;
                    break;
            }
        }
        vTaskDelay(pdMS_TO_TICKS(10));
    }
}

template <std::size_t N>
void ProcessSerialMessage(const std::array<uint8_t, N> &buffer) {

    char command = buffer[0];
    char value = buffer[1];

    switch (command) {

        case 'L' : {
 
            if (value == 1) {
                digitalWrite(ledPin, HIGH);
            }
            else if (value == 0) {
                digitalWrite(ledPin, LOW);
            }
            break;
        }

        case 'B' : {
                
                static const std::unordered_map<char, BlinkRate> blinkRateMap = {
                    {'0', BlinkRate::Slow},
                    {'1', BlinkRate::Medium},
                    {'2', BlinkRate::Fast}
                };

                auto it = blinkRateMap.find(buffer[1]);
                if (it != blinkRateMap.end()) {
                    xTaskNotify(ledBlinkerTask, (uint32_t)it->second, eSetValueWithOverwrite);
                    Serial.printf("Blink rate set to %c\n", buffer[1]);
                    break;
                }
                else {
                    Serial.printf("Invalid blink rate: %c\n", buffer[1]);
                    break;
                }
            break;
        }

        case 'C' : {

            if (value == 1) {
                canRequest = true;
            }
            else if (value == 0) {
                canRequest = false;
            }
            break;
        }

        case 'R' : {

            if (canRequest) {
                Serial.printf("Sending request to %s\n", (const char*)&buffer[1]);
                HTTPClient http;
                http.begin((const char*)&buffer[1]);
                int httpCode = http.GET();
                if (httpCode > 0) {
                    String payload = http.getString();
                    Serial.println(payload);
                }
                else {
                    Serial.printf("Request failed, error: %s\n", http.errorToString(httpCode).c_str());
                }
                http.end();
            }
            else {
                Serial.println("Cannot send request, C0 not received");
            }
            break;
        }

        case 'T' : {

            xTaskNotify(temperatureReaderTask, 1, eSetValueWithOverwrite);
            break;
        }

        case 'G' : {
            
            xTaskNotify(gpsReaderTask, value, eSetValueWithOverwrite);
            break;
        }

        case '\r':
        case '\n':
            break;

        default:
            break;
    }
}

void DallasDeviceScanIndex(DallasTemperature& sensors);
void DallasTemperatureSetup(DallasTemperature &sensors, DeviceAddress &thermalProbeOne, DeviceAddress &thermalProbeTwo);
void TemperatureReaderTask(void* parameter) {

    constexpr uint8_t temperaturePin = 4;
    OneWire oneWire(temperaturePin);
    DallasTemperature sensors(&oneWire);
    DeviceAddress thermalProbeOne = { 0x28, 0xFF, 0x25, 0x61, 0xA3, 0x16, 0x05, 0x16 };

    while (true) {
        sensors.requestTemperatures(); // Send the command to get temperatures
        Serial.printf("Thermal probe one: %f\n", sensors.getTempC(thermalProbeOne));
        if (ulTaskNotifyTake(pdTRUE, pdMS_TO_TICKS(1000))) { // Wait for notification from serial reader task
            DallasDeviceScanIndex(sensors);
        }
    }
}

void PrintDallasAddress(DeviceAddress device_address) {

    uint8_t device_address_length = 8; // The length of the device address is 8 bytes
    for (uint8_t i = 0; i < device_address_length; i++) { // Loop through each byte in the eight-byte address
        if (device_address[i] < 15) Serial.print("0"); // If byte is less than 0x10, add a leading zero to maintain 2-digit format
        Serial.print(device_address[i], HEX);
    }
    Serial.printf("\n");
}

void DallasDeviceScanIndex(DallasTemperature &sensors) {
    sensors.begin();
    Serial.printf("Found %d devices\n", sensors.getDeviceCount());
    for (uint8_t i = 0; i < sensors.getDeviceCount(); i++) {
        DeviceAddress device_address;
        if (!sensors.getAddress(device_address, i)) {
            Serial.printf("Unable to find address for Device %d\n", i);
        } else {
            Serial.printf("Device %d Address: ", i);
            PrintDallasAddress(device_address);
        }
    }
}

void DallasTemperatureSetup(DallasTemperature &sensors, DeviceAddress &thermalProbeOne, DeviceAddress &thermalProbeTwo) {

    sensors.begin();
    Serial.printf("Found %d devices\n", sensors.getDeviceCount());
    if (!sensors.getAddress(thermalProbeOne, 0)) {
      Serial.printf("Unable to find address for Device 0\n");
    } else {
      Serial.printf("Device 0 Address: ");
      PrintDallasAddress(thermalProbeOne);
    }
    if (!sensors.getAddress(thermalProbeTwo, 1)) {
      Serial.printf("Unable to find address for Device 1\n");
    } else {
      Serial.printf("Device 1 Address: ");
      PrintDallasAddress(thermalProbeTwo);
    }

}

void DallasRequestTemperatures(DallasTemperature &sensors) {

    Serial.printf("Number of devices: %d\n", sensors.getDeviceCount());
    sensors.requestTemperatures();
    for (uint8_t i = 0; i < sensors.getDeviceCount(); i++) {
      Serial.printf("Device %d Temperature: %f\n", i, sensors.getTempCByIndex(i));
    }
    
}

void GpsReaderTask(void* parameter) {
    constexpr uint8_t gpsRxPin = 19;
    constexpr uint8_t gpsTxPin = 18;
    constexpr int32_t baudRate = 9600;
    
    TinyGPSPlus gps;
    uint32_t gpsMode = GPSPrintOptions::Parsed;

    Serial2.begin(baudRate, SERIAL_8N1, gpsRxPin, gpsTxPin);
    while (true) {
        while (Serial2.available()) {
            switch (gpsMode) {
                
                case GPSPrintOptions::Off:
                    Serial2.read();
                    break;

                case GPSPrintOptions::Raw:
                    Serial.print((char)Serial2.read());
                    break;

                case GPSPrintOptions::Parsed:
                    if (gps.encode(Serial2.read())) {

                        if (gps.location.isValid()) {
                            Serial.printf("[GPS]Latitude: %f, Longitude: %f\n", gps.location.lat(), gps.location.lng());
                        }
                        if (gps.altitude.isValid()) {
                            Serial.printf("[GPS]Altitude: %f\n", gps.altitude.meters());
                        }
                        if (gps.speed.isValid()) {
                            Serial.printf("[GPS]Speed: %f\n", gps.speed.kmph());
                        }
                        if (gps.course.isValid()) {
                            Serial.printf("[GPS]Course: %f\n", gps.course.deg());
                        }
                    }
                    break;

                default:
                    Serial.printf("Unknown GPS mode: %d\n", gpsMode);
                    gpsMode = GPSPrintOptions::Parsed;
                    break;
            }           
        }
        xTaskNotifyWait(0, 0, &gpsMode, 2000);
    }
}

void HighWaterMeasurerTask(void* parameter) {
    while (true) {
        Serial.printf("\n");
        for (int i = 0; i < taskHandlesSize; i++) {
            Serial.printf("Task %s has %d bytes of free stack\n", pcTaskGetTaskName(*taskHandles[i]), uxTaskGetStackHighWaterMark(*taskHandles[i]));
        }
        // free heap
        Serial.printf("Free heap: %d\n", esp_get_free_heap_size());
        Serial.printf("\n");
        vTaskDelay(pdMS_TO_TICKS(10000));
    }
}

void setup() {
    Serial.begin(115200);
    uint32_t interval = 1000; // I left it here as an example of passing a parameter to a task instead of using a global variable
    xTaskCreate(LedBlinker, "ledBlinker", 1500, (void*)&interval, 1, &ledBlinkerTask);
    xTaskCreate(WifiConnectionTask, "wifiConnection", 4096, NULL, 1, &wifiConnectionTask);
    xTaskCreate(ServerTask, "server", 4096, NULL, 1, &serverTask);
    xTaskCreate(VpnConnectionTask, "vpnConnection", 4096, NULL, 1, &vpnConnectionTask);
    xTaskCreate(SerialReaderTask, "serialReader", 4096, NULL, 1, &serialReaderTask);
    xTaskCreate(TemperatureReaderTask, "temperatureReader", 4096, NULL, 1, &temperatureReaderTask);
    xTaskCreate(GpsReaderTask, "gpsReader", 4096, NULL, 2, &gpsReaderTask);
    xTaskCreate(HighWaterMeasurerTask, "measurer", 2048, NULL, 1, NULL);  
}

void loop() {

}


