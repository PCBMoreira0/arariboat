#include <Arduino.h>
#include <WiFi.h> // Async Web Server
#include "HTTPClient.h"
#include "HttpClientFunctions.hpp"
#include <Husarnet.h> // IPV6 for ESP32 to enable peer-to-peer communication between devices inside a Husarnet network.
#include <ESPAsyncWebServer.h> // Make sure to include Husarnet before this.
#include <AsyncElegantOTA.h> // Over the air updates for the ESP32.
#include <unordered_map> // Hashtable for storing WiFi credentials.
#include "DallasTemperature.h" // For the DS18B20 temperature probes.
#include "TinyGPSPlus.h" // GPS NMEA sentence parser.
#include "arariboat\mavlink.h" // Custom mavlink dialect for the boat generated using Mavgen tool.

// Declare a handle for each task to allow manipulation of the task from other tasks, such as sending notifications, resuming or suspending.
// The handle is initialized to nullptr to avoid the task being created before the setup() function.
// Each handle is then assigned to the task created in the setup() function.

TaskHandle_t ledBlinkerTask = nullptr;
TaskHandle_t wifiConnectionTask = nullptr;
TaskHandle_t serverTask = nullptr;
TaskHandle_t vpnConnectionTask = nullptr;
TaskHandle_t serialReaderTask = nullptr;
TaskHandle_t temperatureReaderTask = nullptr;
TaskHandle_t gpsReaderTask = nullptr;
TaskHandle_t highWaterMeasurerTask = nullptr;

// Array of pointers to the task handles. This allows to iterate over the array and perform operations on all tasks.
TaskHandle_t* taskHandles[] = { &ledBlinkerTask, &wifiConnectionTask, &serverTask, &vpnConnectionTask, &serialReaderTask, &temperatureReaderTask, &gpsReaderTask, &highWaterMeasurerTask};
constexpr auto taskHandlesSize = sizeof(taskHandles) / sizeof(taskHandles[0]); // Get the number of elements in the array.

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

// Tasks can send notifications here to change the blink rate of the LED in order to communicate the status of the boat.
void FastBlinkPulse(int pin);
void LedBlinker(void* parameter) {

    constexpr int ledPin = 2;
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
                FastBlinkPulse(ledPin);
                blinkRate = previousBlinkRate;
            }
        }
    }
}

void FastBlinkPulse(int pin) {
    // 10 pulses
    for (int i = 0; i < 10; i++) {
        digitalWrite(pin, HIGH);
        vTaskDelay(pdMS_TO_TICKS(50));
        digitalWrite(pin, LOW);
        vTaskDelay(pdMS_TO_TICKS(50));
    }
}

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

    // Wait for notification from WifiConnection task that WiFi is connected in order to begin the server
    ulTaskNotifyTake(pdTRUE, portMAX_DELAY);
    
    // Attach OTA update handler to the server and initialize the server.
    AsyncElegantOTA.begin(&server); // Available at http://[esp32ip]/update or http://[esp32hostname]/update
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

    // Husarnet VPN configuration parameters
    const char* hostName = "boat32"; // Host name can be used to access the device instead of typing IPV6 address
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
void DallasTemperatureSetup(DallasTemperature &sensors, DeviceAddress &thermal_probe_zero, DeviceAddress &thermal_probe_one);
void TemperatureReaderTask(void* parameter) {

    constexpr uint8_t temperaturePin = 4; // GPIO used for OneWire communication
    OneWire oneWire(temperaturePin); // Setup a oneWire instance to communicate with any devices that use the OneWire protocol
    DallasTemperature sensors(&oneWire); // Pass our oneWire reference to Dallas Temperature sensor, which uses the OneWire protocol.
    
    // Each probe has a unique 8-byte address. Use the scanIndex method to initially find the addresses of the probes. 
    // Then hardcode the addresses into the program. This is done to avoid the overhead of scanning for the addresses every time the function is called.
    // You should then physically label the probes with tags or stripes as to differentiate them.
    DeviceAddress thermal_probe_zero = { 0x28, 0xFF, 0x25, 0x61, 0xA3, 0x16, 0x05, 0x16 }; 
    DeviceAddress thermal_probe_one = {0}; // If you have more than one probe, add the address here
    
    while (true) {
        sensors.requestTemperatures(); // Send the command to update temperature readings
        float temperature_motor = sensors.getTempC(thermal_probe_zero);
        float temperature_mppt = sensors.getTempC(thermal_probe_one);
        Serial.printf("[Temperature][%x]Motor: %f\n", thermal_probe_zero[7], temperature_motor); // [Temperature][last byte of probe address] = value is the format
        Serial.printf("[Temperature][%x]MPPT: %f\n", thermal_probe_one[7], temperature_mppt);
        
        // Prepare and send a mavlink message
        mavlink_message_t message;
        mavlink_msg_temperatures_pack(1, 200, &message, temperature_motor, temperature_mppt);
        uint8_t buffer[MAVLINK_MAX_PACKET_LEN];
        uint16_t len = mavlink_msg_to_send_buffer(buffer, &message);
        Serial.write(buffer, len);

        if (ulTaskNotifyTake(pdTRUE, pdMS_TO_TICKS(1000))) { // Wait for notification from serial reader task to scan for new probes
            DallasDeviceScanIndex(sensors); 
        }
    }
}

/// @brief Prints the 8-byte address of a Dallas Thermal Probe to the serial monitor
/// @param device_address 
void PrintDallasAddress(DeviceAddress device_address) {

    uint8_t device_address_length = 8; // The length of the device address is 8 bytes
    for (uint8_t i = 0; i < device_address_length; i++) { // Loop through each byte in the eight-byte address
        if (device_address[i] < 15) Serial.print("0"); // If byte is less than 0x10, add a leading zero to maintain 2-digit format
        Serial.print(device_address[i], HEX);
    }
    Serial.printf("\n");
}

/// @brief Scans for Dallas Thermal Probes and prints their addresses to the serial monitor
/// After adding a new probe, run this function to find the address of the probe. Then hardcode the address into the program
/// for faster performance.
/// @param sensors 
void DallasDeviceScanIndex(DallasTemperature &sensors) {
    sensors.begin(); // Scan for devices on the OneWire bus.
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

void DallasTemperatureSetup(DallasTemperature &sensors, DeviceAddress &thermal_probe_zero, DeviceAddress &thermal_probe_one) {

    sensors.begin();
    Serial.printf("Found %d devices\n", sensors.getDeviceCount());
    if (!sensors.getAddress(thermal_probe_zero, 0)) {
      Serial.printf("Unable to find address for Device 0\n");
    } else {
      Serial.printf("Device 0 Address: ");
      PrintDallasAddress(thermal_probe_zero);
    }
    if (!sensors.getAddress(thermal_probe_one, 1)) {
      Serial.printf("Unable to find address for Device 1\n");
    } else {
      Serial.printf("Device 1 Address: ");
      PrintDallasAddress(thermal_probe_one);
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

    // Example of latitude: 40.741895 (north is positive)
    // Example of longitude: -73.989308 (west is negative)
    // The fifth decimal place is worth up to 1.1 m. The sixth decimal place is worth up to 11cm. And so forth.
    
    constexpr uint8_t gps_rx_pin = 19; // Chosen RX pin of ESP32 to be connected to TX pin of GPS module
    constexpr uint8_t gps_tx_pin = 18; // Chosen TX pin of ESP32 to be connected to RX pin of GPS module
    constexpr int32_t baud_rate = 9600; // Fixed baud rate used by NEO-6M GPS module
    
    TinyGPSPlus gps;
    uint32_t gpsMode = GPSPrintOptions::Parsed;

    // Three hardware serial ports are available on the ESP32 with configurable GPIOs.
    // Serial0 is used for debugging and is connected to the USB-to-serial converter. Therefore, Serial1 and Serial2 are available.
    Serial2.begin(baud_rate, SERIAL_8N1, gps_rx_pin, gps_tx_pin); // Initialize Serial2 with the chosen baud rate and pins
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

                        constexpr float invalid_value = -1.f; // Begin the fields with arbitrated invalid value and update them if the gps data is valid.
                        float latitude = invalid_value;
                        float longitude = invalid_value;
                        float speed = invalid_value;
                        float course = invalid_value;
                        uint8_t satellites = 0;

                        if (gps.location.isValid()) {
                            latitude = gps.location.lat();
                            longitude = gps.location.lng();
                            Serial.printf("[GPS]Latitude: %f, Longitude: %f\n", latitude, longitude);
                        }
                        if (gps.speed.isValid()) {
                            speed = gps.speed.kmph();
                            Serial.printf("[GPS]Speed: %f\n", speed);
                        }
                        if (gps.course.isValid()) {
                            course = gps.course.deg();
                            Serial.printf("[GPS]Course: %f\n", course);
                        }
                        if (gps.satellites.isValid()) {
                            satellites = gps.satellites.value();
                            Serial.printf("[GPS]Satellites: %d\n", satellites);
                        }

                        // Prepare and send mavlink message
                        mavlink_message_t message;
                        mavlink_msg_gps_info_pack(1, 200, &message, latitude, longitude, speed, course, satellites);
                        uint8_t buffer[MAVLINK_MAX_PACKET_LEN];
                        uint16_t length = mavlink_msg_to_send_buffer(buffer, &message);
                        Serial.write(buffer, length);
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


