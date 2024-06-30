#include <Arduino.h> // Main Arduino library, required for projects that use the Arduino framework.
#include <WiFi.h> // Main library for WiFi connectivity, also used by AsyncWebServer.
#include <unordered_map> // Hashtable for storing WiFi credentials.
#include "HTTPClient.h" // HTTP client for sending requests to a listening server.
#include "HttpClientFunctions.hpp" // Auxiliary functions for sending HTTP requests.
#include "Husarnet.h" // IPV6 for ESP32 to enable peer-to-peer communication between devices inside a Husarnet network.
#include "ESPAsyncWebServer.h" // Make sure to include Husarnet before this.
#include "AsyncElegantOTA.h" // Over the air updates for the ESP32.
#include "TinyGPSPlus.h" // GPS NMEA sentence parser.
#include "arariboat/mavlink.h" // Custom mavlink dialect for the boat generated using Mavgen tool.
#include <Wire.h> // Required for the ADS1115 ADC and communication with the LoRa board.
#include <Preferences.h> // Non-volatile storage for storing the state of the boat.
#include "Utilities.hpp" // Custom utility macros and functions.

//TODO: Improve server interface for configuration and debug purposes.
//TODO: Implement auxiliary battery and pumps readings using some I2C system
//TODO: Send data directly to InfluxDB instead of using Husarnet
//TODO: Save all measurements to a file in the SPIFFS file system or some microSD card
//TODO: Use IDF event loop for serial parsing and intertask communication
//TODO: Implement RPM measurements code

// Singleton class for storing system-data that needs to be accessed by multiple tasks.
class SystemData {

public:
    static SystemData& getInstance() {
        static SystemData instance;
        return instance;
    }

    enum debug_print_flags : uint16_t {
        None = 0b0000000000,
        Wifi = 0b0000000001,
        Server = 0b0000000010,
        Vpn = 0b0000000100,
        Serial = 0b0000001000,
        Temperature = 0b0000010000,
        Gps = 0b0000100000,
        Instrumentation = 0b0001000000,
        High_water = 0b1000000000,
        All = 0b1111111111
    };

    debug_print_flags debug_print = debug_print_flags::All;
    mavlink_instrumentation_t instrumentation;
    mavlink_gps_info_t gps;
    mavlink_temperatures_t temperature;
    mavlink_control_system_t controlSystem;

    
private:
    SystemData() { // Private constructor to avoid multiple instances.
        instrumentation = { 0 };
        gps = { 0 };
        temperature = { 0 };
        controlSystem = { 0 };
    };
    SystemData(SystemData const&) = delete; // Delete copy constructor.
    SystemData& operator=(SystemData const&) = delete; // Delete assignment operator.
    SystemData(SystemData&&) = delete; // Delete move constructor.
};

// Declare a handle for each task to allow manipulation of the task from other tasks, such as sending notifications, resuming or suspending.
// The handle is initialized to nullptr to avoid the task being created before the setup() function.
// Each handle is then assigned to the task created in the setup() function.

TaskHandle_t ledBlinkerTaskHandle = nullptr;
TaskHandle_t wifiConnectionTaskHandle = nullptr;
TaskHandle_t serverTaskHandle = nullptr;
TaskHandle_t vpnConnectionTaskHandle = nullptr;
TaskHandle_t serialReaderTaskHandle = nullptr;
TaskHandle_t temperatureReaderTaskHandle = nullptr;
TaskHandle_t gpsReaderTaskHandle = nullptr;
TaskHandle_t instrumentationReaderTaskHandle = nullptr;
TaskHandle_t highWaterMeasurerTaskHandle = nullptr;

// Array of pointers to the task handles. This allows to iterate over the array and perform operations on all tasks, such as resuming, suspending or reading free stack memory.
TaskHandle_t* taskHandles[] = { &ledBlinkerTaskHandle, &wifiConnectionTaskHandle, &serverTaskHandle, &vpnConnectionTaskHandle, &serialReaderTaskHandle, 
                                &temperatureReaderTaskHandle, &gpsReaderTaskHandle, &instrumentationReaderTaskHandle,
                                &highWaterMeasurerTaskHandle};

constexpr auto taskHandlesSize = sizeof(taskHandles) / sizeof(taskHandles[0]); // Get the number of elements in the array.

enum BlinkRate : uint32_t {
    Slow = 2000,
    Medium = 1000,
    Fast = 300,
    Pulse = 100 // Pulse is a special value that will make the LED blink fast and then return to the previous blink rate.
};

enum GPSPrintOptions : uint32_t {
    Off = '0',
    Raw,
    Parsed
};

// Tasks can send notifications here to change the blink rate of the LED in order to communicate the status of the boat.
void FastBlinkPulse(int pin);
void LedBlinkerTask(void* parameter) {

    constexpr uint8_t ledPin = 2; // Built-in LED pin for the ESP32 DevKit board.
    pinMode(ledPin, OUTPUT);
    uint32_t blink_rate = BlinkRate::Slow;
    uint32_t previous_blink_rate = blink_rate;

    auto FastBlinkPulse = [](int pin) {
        for (int i = 0; i < 4; i++) {
            digitalWrite(pin, HIGH); vTaskDelay(pdMS_TO_TICKS(50));
            digitalWrite(pin, LOW);  vTaskDelay(pdMS_TO_TICKS(50));
        }
    };

    auto BuzzerWrite = [&]() {
        constexpr uint8_t buzzer_pin = 26;
        static uint8_t counter = 0;

        // Define the rhythm pattern
        constexpr uint8_t pattern[] = {1, 0, 1, 0, 1, 1, 0, 0}; // Example pattern

        // Calculate the position within the pattern
        uint8_t patternSize = sizeof(pattern) / sizeof(pattern[0]);
        uint8_t patternPosition = counter % patternSize;

        // Determine the buzzer state based on the pattern
        bool buzzerState = pattern[patternPosition] != 0;

        if (blink_rate == BlinkRate::Fast) {
            dacWrite(buzzer_pin, buzzerState ? 150 : 0);
        } else {
            dacWrite(buzzer_pin, 0);
        }
        // Increment the counter
        counter++;
    };
  
    while (true) {

        static uint32_t previous_blink_time = millis();
        if (millis() - previous_blink_time > blink_rate) {
            previous_blink_time = millis();
            BuzzerWrite();
            digitalWrite(ledPin, !digitalRead(ledPin));
        }
           
        // Set blink rate to the value received from the notification
        static uint32_t received_value = BlinkRate::Slow;
        if (xTaskNotifyWait(0, 0, (uint32_t*)&received_value, 100)) {
            if (received_value == BlinkRate::Pulse) {
                FastBlinkPulse(ledPin);
            } else {
                blink_rate = received_value;
            }
        }
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
            xTaskNotify(ledBlinkerTaskHandle, BlinkRate::Fast, eSetValueWithOverwrite);
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
                    xTaskNotify(ledBlinkerTaskHandle, BlinkRate::Slow, eSetValueWithOverwrite);
                    xTaskNotifyGive(vpnConnectionTaskHandle);
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
        //#define USE_ASYNC_CLIENT 
        #ifdef USE_ASYNC_CLIENT
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

        #endif
    }
}

void VPNConnectionTask(void* parameter) {

    // The use of a VPN is to allow this device to be accessed from the public internet without port forwarding or paying for a static IPV4 address.
    // It accomplishes this by creating a virtual network between all devices that are connected to it, taking advantage of IPv6 addressing.
    // Each device is assigned a unique IPv6 address that can be used to access it from anywhere in the world.
    // The Husarnet VPN is free for up to 5 devices.
    // By attaching a router with a SIM slot on the boat, messages can be exchanged by both the internet, using HTTP or WebSockets, and the LoRa radio.

    // Husarnet VPN configuration parameters
    const char* hostName = "boat32"; // Host name can be used to access the device instead of typing IPV6 address
    const char* husarnetJoinCode = "fc94:b01d:1803:8dd8:b293:5c7d:7639:932a/YNqd5m2Bjp65Miucf9R95p";
    const char* dashboardURL = "default";

    // Wait for notification that WiFi is connected before starting the VPN.
    ulTaskNotifyTake(pdTRUE, portMAX_DELAY);
    Husarnet.selfHostedSetup(dashboardURL);
    Husarnet.join(husarnetJoinCode, hostName);
    Husarnet.start();
    xTaskNotifyGive(serverTaskHandle); // Notify Server task that VPN is connected
    vTaskDelete(NULL); // Delete this task after VPN is connected
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
                    xTaskNotify(ledBlinkerTaskHandle, (uint32_t)it->second, eSetValueWithOverwrite);
                    break;
                }
                else {
                    Serial.printf("\nInvalid blink rate: %c\n", buffer[1]);
                    break;
                }
            break;
        }

        case 'R' : {
                Serial.printf("\nSending request to %s\n", (const char*)&buffer[1]);
                HTTPClient http;
                http.begin((const char*)&buffer[1]);
                int httpCode = http.GET();
                if (httpCode > 0) {
                    String payload = http.getString();
                    Serial.println(payload);
                }
                else {
                    Serial.printf("\nRequest failed, error: %s\n", http.errorToString(httpCode).c_str());
                }
                http.end();
                 break;
        }

        case 'T' : {
            xTaskNotify(temperatureReaderTaskHandle, 1, eSetValueWithOverwrite);
            break;
        }
        case 'G' : {
            
            xTaskNotify(gpsReaderTaskHandle, value, eSetValueWithOverwrite);
            break;
        }
        
        case '\r':
        case '\n':
            break;
        default:
            break;
    }
}

void GpsReaderTask(void* parameter) {

    // Example of latitude: 40.741895 (north is positive)
    // Example of longitude: -73.989308 (west is negative)
    // The fifth decimal place is worth up to 1.1 m. The sixth decimal place is worth up to 11cm. And so forth.
    
    // Three hardware serial ports are available on the ESP32 with configurable GPIOs.
    // Serial0 is used for debugging and is connected to the USB-to-serial converter. Therefore, Serial1 and Serial2 are available.
    
    TinyGPSPlus gps; // Object that parses NMEA sentences from the NEO-6M GPS module
    constexpr uint8_t gps_rx_pin = 16;  
    constexpr uint8_t gps_tx_pin = 17; 
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
                    //DEBUG_PRINTF("[GPS]Latitude: %f, Longitude: %f\n", latitude, longitude);
                }
                if (gps.speed.isValid()) {
                    speed = gps.speed.kmph();
                    //DEBUG_PRINTF("[GPS]Speed: %f\n", speed);
                }
                if (gps.course.isValid()) {
                    course = gps.course.deg();
                    //DEBUG_PRINTF("[GPS]Course: %f\n", course);
                }
                if (gps.satellites.isValid()) {
                    satellites = gps.satellites.value();
                    if (!satellites) break; // If no satellites are visible, the gps data is invalid. Break from the function.
                    //DEBUG_PRINTF("[GPS]Satellites: %d\n", satellites);
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
                xTaskNotify(ledBlinkerTaskHandle, BlinkRate::Pulse, eSetValueWithOverwrite); 
            }
        }           
        vTaskDelay(pdMS_TO_TICKS(6000));
    }
}

/// @brief Auxiliary task to measure free stack memory of each task and free heap of the system.
/// Useful to detect possible stack overflows on a task and allocate more stack memory for it if necessary.
/// @param parameter Unused. Just here to comply with the task function signature.
void StackHighWaterMeasurerTask(void* parameter) {
    while (true) {
        if (SystemData::getInstance().debug_print & SystemData::debug_print_flags::High_water) {
            Serial.printf("\n");
            for (int i = 0; i < taskHandlesSize; i++) {
                Serial.printf("[Task]%s has %d bytes of free stack\n", pcTaskGetTaskName(*taskHandles[i]), uxTaskGetStackHighWaterMark(*taskHandles[i]));
            }
            Serial.printf("[Task]System free heap: %d\n", esp_get_free_heap_size());            
        }
        vTaskDelay(pdMS_TO_TICKS(25000));
    }
}

extern void InstrumentationReaderTask(void* parameter);
extern void TemperatureReaderTask(void* parameter);

void setup() {

    Serial.begin(9600);
    Wire.begin(); // Master mode
    xTaskCreate(LedBlinkerTask, "ledBlinker", 2048, NULL, 1, &ledBlinkerTaskHandle);
    //xTaskCreate(WifiConnectionTask, "wifiConnection", 4096, NULL, 1, &wifiConnectionTaskHandle);
    //xTaskCreate(VPNConnectionTask, "vpnConnection", 4096, NULL, 1, &vpnConnectionTaskHandle);
    //xTaskCreate(ServerTask, "server", 4096, NULL, 1, &serverTaskHandle);
    xTaskCreate(SerialReaderTask, "serialReader", 4096, NULL, 1, &serialReaderTaskHandle);
    xTaskCreate(TemperatureReaderTask, "temperatureReader", 4096, NULL, 1, &temperatureReaderTaskHandle);
    //xTaskCreate(GpsReaderTask, "gpsReader", 4096, NULL, 1, &gpsReaderTaskHandle);
    xTaskCreate(InstrumentationReaderTask, "instrumentationReader", 4096, NULL, 3, &instrumentationReaderTaskHandle);
    //xTaskCreate(StackHighWaterMeasurerTask, "measurer", 2048, NULL, 1, NULL);  
}

void loop() {
    
}


