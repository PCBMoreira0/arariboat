#include <Arduino.h> // Main Arduino library, required for projects that use the Arduino framework.
#include <WiFi.h> // Main library for WiFi connectivity, also used by AsyncWebServer.
#include <ArduinoJson.h> // Library for parsing and generating JSON for data exchange between the boat and the ground station via HTTP.
#include <unordered_map> // Hashtable for storing WiFi credentials.
#include "HTTPClient.h" // HTTP client for sending requests to a listening server.
#include "HttpClientFunctions.hpp" // Auxiliary functions for sending HTTP requests.
#include "Husarnet.h" // IPV6 for ESP32 to enable peer-to-peer communication between devices inside a Husarnet network.
#include "ESPAsyncWebServer.h" // Make sure to include Husarnet before this.
#include <ESPmDNS.h> // Allows to resolve hostnames to IP addresses within a local network.
#include "AsyncElegantOTA.h" // Over the air updates for the ESP32.
#include "DallasTemperature.h" // For the DS18B20 temperature probes.
#include "TinyGPSPlus.h" // GPS NMEA sentence parser.
#include "arariboat\mavlink.h" // Custom mavlink dialect for the boat generated using Mavgen tool.
#include "arariboat\SystemData.hpp" // Singleton class to hold system wide data
#include "Adafruit_ADS1X15.h" // 16-bit high-linearity with programmable gain amplifier Analog-Digital Converter for measuring current and voltage.
#include <SPI.h> // Required for the ADS1115 ADC.
#include <Wire.h> // Required for the ADS1115 ADC and communication with the LoRa board.
#include <Encoder.h> // Rotary encoder library.
#include <Preferences.h> // Non-volatile storage for storing the state of the boat.

#define DEBUG // Uncomment to enable debug messages.
#ifdef DEBUG
#define DEBUG_PRINTF(message, ...) Serial.printf(message, __VA_ARGS__)
#else
#define DEBUG_PRINTF(message, ...)
#endif

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
TaskHandle_t auxiliaryReaderTaskHandle = nullptr;
TaskHandle_t encoderControlTaskHandle = nullptr;
TaskHandle_t highWaterMeasurerTaskHandle = nullptr;

// Array of pointers to the task handles. This allows to iterate over the array and perform operations on all tasks, such as resuming, suspending or reading free stack memory.
TaskHandle_t* taskHandles[] = { &ledBlinkerTaskHandle, &wifiConnectionTaskHandle, &serverTaskHandle, &vpnConnectionTaskHandle, &serialReaderTaskHandle, 
                                &temperatureReaderTaskHandle, &gpsReaderTaskHandle, &instrumentationReaderTaskHandle, 
                                &auxiliaryReaderTaskHandle, &encoderControlTaskHandle, &highWaterMeasurerTaskHandle};

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

    constexpr uint8_t led_pin = 2; // Built-in LED pin for the ESP32 DevKit board.
    pinMode(led_pin, OUTPUT);
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
            digitalWrite(led_pin, !digitalRead(led_pin));
        }
           
        // Set blink rate to the value received from the notification
        static uint32_t received_value = BlinkRate::Slow;
        if (xTaskNotifyWait(0, 0, (uint32_t*)&received_value, 100)) {
            if (received_value == BlinkRate::Pulse) {
                FastBlinkPulse(led_pin);
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
                    xTaskNotifyGive(serverTaskHandle);
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

        // Function to send the combined HTML response
        auto sendResponse = [&](const String &htmlContent) {
            request->send(200, "text/html", htmlContent);
        };

        // Create the HTML content with raw CSS styles
        String htmlContent = "<html><head><title>Boat-Companion</title>"
                            "<style>"
                            "body { font-family: Arial, sans-serif; background-color: #f7f7f7; margin: 0; padding: 0; }"
                            ".container { padding: 10px; display: flex; flex-wrap: wrap; }"
                            ".card { flex: 1 0 calc(50% - 20px); margin: 10px; padding: 10px; border-radius: 10px; box-shadow: 0 2px 5px rgba(0, 0, 0, 0.1); }"
                            ".blue-card { background-color: #0088cc; color: #fff; }"
                            ".orange-card { background-color: #ff9800; color: #fff; }"
                            ".spacer { flex-basis: 100%; height: 10px; }" // Added style for grey spacer
                            "h1, h2 { color: black; font-weight: bold; margin: 0; padding: 0; width: 100%; }"
                            "h2 { font-size: 18px; }"
                            "p { color: #333; }"
                            "</style>"
                            "</head><body>"
                            "<div class='container'>";

        // Create blue card for WiFi info
        htmlContent += "<div class='card blue-card'>";
        htmlContent += "<h1>Boat Companion</h1>";
        htmlContent += "<p>WiFi connected: " + WiFi.SSID() + "</p>";
        htmlContent += "<p>IP address: " + WiFi.localIP().toString() + "</p>";
        htmlContent += "</div>";

        // Create orange card for Control System data
        htmlContent += "<div class='card orange-card'>";
        htmlContent += "<h2>Control System Data</h2>";
        htmlContent += "<p>DAC Output: " + String(systemData.controlSystem.dac_output) + "</p>";
        htmlContent += "<p>Potentiometer Signal: " + String(systemData.controlSystem.potentiometer_signal) + "</p>";
        htmlContent += "</div>";

        // Create blue card for Instrumentation System data
        htmlContent += "<div class='card blue-card'>";
        htmlContent += "<h2>Instrumentation System Data</h2>";
        htmlContent += "<p>Battery Voltage: " + String(systemData.instrumentationSystem.battery_voltage) + "</p>";
        htmlContent += "<p>Motor Current: " + String(systemData.instrumentationSystem.motor_current) + "</p>";
        htmlContent += "<p>Battery Current: " + String(systemData.instrumentationSystem.battery_current) + "</p>";
        htmlContent += "<p>MPPT Current: " + String(systemData.instrumentationSystem.mppt_current) + "</p>";
        htmlContent += "</div>";

        // Create orange card for GPS System data
        htmlContent += "<div class='card orange-card'>";
        htmlContent += "<h2>GPS System Data</h2>";
        htmlContent += "<p>Latitude: " + String(systemData.gpsSystem.latitude) + "</p>";
        htmlContent += "<p>Longitude: " + String(systemData.gpsSystem.longitude) + "</p>";
        htmlContent += "<p>Speed: " + String(systemData.gpsSystem.speed) + "</p>";
        htmlContent += "<p>Course: " + String(systemData.gpsSystem.course) + "</p>";
        htmlContent += "<p>Satellites: " + String(systemData.gpsSystem.satellites_visible) + "</p>";
        htmlContent += "</div>";

        // Create blue card for Auxiliary System data
        htmlContent += "<div class='card blue-card'>";
        htmlContent += "<h2>Auxiliary System Data</h2>";
        htmlContent += "<p>Pump Mask: " + String(systemData.auxiliarySystem.pumps) + "</p>";
        htmlContent += "<p>Auxiliary Current: " + String(systemData.auxiliarySystem.current) + "</p>";
        htmlContent += "<p>Auxiliary Voltage: " + String(systemData.auxiliarySystem.voltage) + "</p>";
        htmlContent += "</div>";

        // Create orange card for Temperature System data
        htmlContent += "<div class='card orange-card'>";
        htmlContent += "<h2>Temperature System Data</h2>";
        htmlContent += "<p>Motor Temperature: " + String(systemData.temperatureSystem.temperature_motor) + "</p>";
        htmlContent += "<p>Battery Temperature: " + String(systemData.temperatureSystem.temperature_battery) + "</p>";
        htmlContent += "<p>MPPT Temperature: " + String(systemData.temperatureSystem.temperature_mppt) + "</p>";
        htmlContent += "</div>";

        // Add grey spacer between cards
        htmlContent += "<div class='spacer'></div>";

        // Close the container and HTML content
        htmlContent += "</div></body></html>";

        // Send the final HTML response to the client
        sendResponse(htmlContent);
    });
        
    server.on("/reset", HTTP_GET, [](AsyncWebServerRequest *request) {
        // log reset message
        request->send(200, "text/html", "<h1>Boat-Companion</h1><p>Resetting...</p>");
        vTaskDelay(pdMS_TO_TICKS(1000));
        ESP.restart();
    });

    server.on("/control-system", HTTP_GET, [](AsyncWebServerRequest *request) {
        
        float dac_output = systemData.controlSystem.dac_output;
        float potentiometer_signal = systemData.controlSystem.potentiometer_signal;

        constexpr uint16_t doc_size = 64;
        StaticJsonDocument<doc_size> doc;
        doc["dac_output"] = dac_output;
        doc["potentiometer_signal"] = potentiometer_signal;

        // Send json using char array
        char output[doc_size];
        serializeJson(doc, output);
        request->send(200, "application/json", output);
    });

    server.on("/instrumentation-system", HTTP_GET, [](AsyncWebServerRequest *request) {
        
        // Send system instrumentation data
        float battery_voltage = systemData.instrumentationSystem.battery_voltage;
        float motor_current = systemData.instrumentationSystem.motor_current;
        float battery_current = systemData.instrumentationSystem.battery_current;
        float mppt_current = systemData.instrumentationSystem.mppt_current;
        
        constexpr uint16_t doc_size = 128;
        StaticJsonDocument<doc_size> doc;
        doc["battery_voltage"] = battery_voltage;
        doc["motor_current"] = motor_current;
        doc["battery_current"] = battery_current;
        doc["mppt_current"] = mppt_current;
        
        // Send json using char array
        char output[doc_size];
        serializeJson(doc, output);
        request->send(200, "application/json", output);
    });

    // temperatures
    server.on("/temperature-system", HTTP_GET, [](AsyncWebServerRequest *request) {
        
        // Send temperature data from singleton class
        float temperature_motor = systemData.temperatureSystem.temperature_motor;
        float temperature_battery = systemData.temperatureSystem.temperature_battery;
        float temperature_mppt = systemData.temperatureSystem.temperature_mppt;
        
        constexpr uint16_t doc_size = 128;
        StaticJsonDocument<doc_size> doc;
        doc["temperature_motor"] = temperature_motor;
        doc["temperature_battery"] = temperature_battery;
        doc["temperature_mppt"] = temperature_mppt;
        
        // Send json using char array
        char output[doc_size];
        serializeJson(doc, output);
        request->send(200, "application/json", output);
    });
    
    server.on("/gps-system", HTTP_GET, [](AsyncWebServerRequest *request) {
        
        // Send GPS data from singleton class
        float latitude = systemData.gpsSystem.latitude;
        float longitude = systemData.gpsSystem.longitude;
        float speed = systemData.gpsSystem.speed;
        float course = systemData.gpsSystem.course;
        uint8_t satellites = systemData.gpsSystem.satellites_visible;

        constexpr uint16_t doc_size = 200;
        StaticJsonDocument<doc_size> doc;
        doc["latitude"] = latitude;
        doc["longitude"] = longitude;
        doc["speed"] = speed;
        doc["course"] = course;
        doc["satellites"] = satellites;

        // Send json using char array
        char output[doc_size];
        serializeJson(doc, output);
        request->send(200, "application/json", output);

    });

    server.on("/auxiliary-system", HTTP_GET, [](AsyncWebServerRequest *request) {
        // Send control system data from singleton class
        uint8_t pumps = systemData.auxiliarySystem.pumps;
        float aux_current = systemData.auxiliarySystem.current;
        float aux_voltage = systemData.auxiliarySystem.voltage;
        
        constexpr uint16_t doc_size = 128;
        StaticJsonDocument<doc_size> doc;
        doc["pumps"] = pumps;
        doc["aux_current"] = aux_current;
        doc["aux_voltage"] = aux_voltage;

        // Send json using char array
        char output[doc_size];
        serializeJson(doc, output);
        request->send(200, "application/json", output);
    });

    //Wait for notification from WiFi connection task before starting the server.
    ulTaskNotifyTake(pdTRUE, portMAX_DELAY);

    // Allow the server to be accessed by hostname instead of IP address.
    if(!MDNS.begin("boat-companion")) {
        Serial.println("[MDNS]Error starting mDNS!");
    }
    
    // Attach the update handler to the server and initialize the server.
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

        case 'C' : {
            // Try to parse float to send current calibration value to auxiliary reader task
            float calibration_value = 0.0f;
            if (sscanf((const char*)&buffer[1], "%f", &calibration_value)) {
                Serial.printf("\n[SERIAL-CALIBRATION] Value: %f\n", calibration_value);
                xTaskNotify(auxiliaryReaderTaskHandle, (uint32_t)calibration_value, eSetValueWithOverwrite);
            }
            break;
        }

        case 'Q' : {
            // Send notification to auxiliary reader to start calibration
            xTaskNotify(auxiliaryReaderTaskHandle, 1, eSetValueWithOverwrite);
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
void TemperatureReaderTask(void* parameter) {

    constexpr uint8_t power_pin = 2; // GPIO used to power the temperature probes
    constexpr uint8_t temperature_bus_pin = 15; // GPIO used for OneWire communication
    
    pinMode(power_pin, OUTPUT); digitalWrite(power_pin, HIGH); // Set power pin to HIGH to power the temperature probes
    OneWire one_wire(temperature_bus_pin); // Setup a one_wire instance to communicate with any devices that use the OneWire protocol
    DallasTemperature sensors(&one_wire); // Pass our one_wire reference to Dallas Temperature sensor, which uses the OneWire protocol.
    
    // Each probe has a unique 8-byte address. Use the scanIndex method to initially find the addresses of the probes. 
    // Then hardcode the addresses into the program. This is done to avoid the overhead of scanning for the addresses every time the function is called.
    // You should then physically label the probes with tags or stripes as to differentiate them.
    DeviceAddress thermal_probe_zero = {0x28, 0x86, 0x1C, 0x07, 0xD6, 0x01, 0x3C, 0x8C};
    DeviceAddress thermal_probe_one = { 0 }; 
    DeviceAddress thermal_probe_two = { 0 };

    while (true) {
        sensors.requestTemperatures(); // Send the command to update temperature readings
        float temperature_motor = sensors.getTempC(thermal_probe_zero);
        float temperature_battery = -64.0f; // TODO: Implement battery temperature reading
        float temperature_mppt = sensors.getTempC(thermal_probe_one);

        #ifdef DEBUG_PRINTF
        if (systemData.debug_print & SystemData::debug_print_flags::Temperature) {
            if (temperature_motor == DEVICE_DISCONNECTED_C) {
                DEBUG_PRINTF("\n[Temperature][%x]Motor: Device disconnected\n", thermal_probe_zero[0]);
            } else {
                DEBUG_PRINTF("\n[Temperature][%x]Motor: %f\n", thermal_probe_zero[0], temperature_motor); // [Temperature][First byte of probe address] = value is the format
            }

            if (temperature_battery == DEVICE_DISCONNECTED_C) {
                DEBUG_PRINTF("\n[Temperature][%x]Battery: Device disconnected\n", thermal_probe_one[0]);
            } else {
                DEBUG_PRINTF("\n[Temperature][%x]Battery: %f\n", thermal_probe_one[0], temperature_battery);
            }

            if (temperature_mppt == DEVICE_DISCONNECTED_C) {
                DEBUG_PRINTF("\n[Temperature][%x]MPPT: Device disconnected\n", thermal_probe_two[0]);
            } else {
                DEBUG_PRINTF("\n[Temperature][%x]MPPT: %f\n", thermal_probe_one[0], temperature_mppt);
            }
        }
        #endif

        // Prepare and send a mavlink message
        mavlink_message_t message;
        mavlink_temperatures_t temperatures = {
            .temperature_motor = temperature_motor,
            .temperature_battery = temperature_battery,
            .temperature_mppt = temperature_mppt
        };
        mavlink_msg_temperatures_encode_chan(1, MAV_COMP_ID_ONBOARD_COMPUTER, MAVLINK_COMM_0, &message, &temperatures);
        uint8_t buffer[MAVLINK_MAX_PACKET_LEN];
        uint16_t len = mavlink_msg_to_send_buffer(buffer, &message);
        Serial.write(buffer, len);

        if (ulTaskNotifyTake(pdTRUE, pdMS_TO_TICKS(10000))) { // Wait for notification from serial reader task to scan for new probes
            DallasDeviceScanIndex(sensors); 
        }
    }
}

/// @brief Auxiliary function to print the 8-byte address of a Dallas Thermal Probe to the serial port
/// @param device_address 
void PrintProbeAddress(DeviceAddress device_address) {

    uint8_t device_address_length = 8; // The length of the device address is 8 bytes
    for (uint8_t i = 0; i < device_address_length; i++) { // Loop through each byte in the eight-byte address
        if (device_address[i] < 15) Serial.print("0"); // If byte is less than 0x10, add a leading zero to maintain 2-digit format
        Serial.print(device_address[i], HEX);
    }
    Serial.printf("\n");
}

/// @brief Scans for Dallas Thermal Probes and prints their addresses to the serial port
/// After adding a new probe, run this function to find the address of the probe. Then hardcode the address into the program
/// for faster performance.
/// @param sensors 
void DallasDeviceScanIndex(DallasTemperature &sensors) {
    sensors.begin(); // Scan for devices on the OneWire bus.
    Serial.printf("\nFound %d devices\n", sensors.getDeviceCount());
    for (uint8_t i = 0; i < sensors.getDeviceCount(); i++) {
        DeviceAddress device_address;
        if (!sensors.getAddress(device_address, i)) {
            Serial.printf("Unable to find address for Device %d\n", i);
        } else {
            Serial.printf("Device %d Address: \n", i);
            PrintProbeAddress(device_address);
        }
    }
}

void GpsReaderTask(void* parameter) {

    // Example of latitude: 40.741895 (north is positive)
    // Example of longitude: -73.989308 (west is negative)
    // The fifth decimal place is worth up to 1.1 m. The sixth decimal place is worth up to 11cm. And so forth.
    systemData.gpsSystem.latitude = -22.9085185092264; // Initialize with a default value 
    systemData.gpsSystem.longitude = -43.1723022460938; // Initialize with a default value
    
    // Three hardware serial ports are available on the ESP32 with configurable GPIOs.
    // Serial0 is used for debugging and is connected to the USB-to-serial converter. Therefore, Serial1 and Serial2 are available.
    
    TinyGPSPlus gps; // Object that parses NMEA sentences from the NEO-6M GPS module
    constexpr uint8_t gps_rx_pin = 16;  
    constexpr uint8_t gps_tx_pin = 17; 
    constexpr int32_t baud_rate = 9600; // Fixed baud rate used by NEO-6M GPS module
    static uint32_t mavlink_timer = 0; // Timer used to send mavlink messages at a fixed rate
    Serial2.begin(baud_rate, SERIAL_8N1, gps_rx_pin, gps_tx_pin); // Initialize Serial2 with the chosen baud rate and pins

    while (true) {
        while (Serial2.available()) {
            // Reads the serial stream from the NEO-6M GPS module and parses it into TinyGPSPlus object if a valid NMEA sentence is received
            if (gps.encode(Serial2.read())) { 

                if (gps.location.isValid()) {
                    systemData.gpsSystem.latitude = gps.location.lat();
                    systemData.gpsSystem.longitude = gps.location.lng();
                    //DEBUG_PRINTF("[GPS]Latitude: %f, Longitude: %f\n", latitude, longitude);
                }
                if (gps.speed.isValid()) {
                    systemData.gpsSystem.speed = gps.speed.kmph();
                    //DEBUG_PRINTF("[GPS]Speed: %f\n", speed);
                }
                if (gps.course.isValid()) {
                    systemData.gpsSystem.course = gps.course.deg();
                    //DEBUG_PRINTF("[GPS]Course: %f\n", course);
                }

                if (gps.satellites.isValid()) {
                    systemData.gpsSystem.satellites_visible = gps.satellites.value();
                    //DEBUG_PRINTF("[GPS]Satellites: %d\n", satellites);
                }

                break;
            }
        }

        if (millis() - mavlink_timer > 7000) {
            mavlink_timer = millis();
            // Prepare and send mavlink message by encoding the payload into a struct, then encoding the struct into a mavlink message below.
            mavlink_message_t message;
            mavlink_msg_gps_info_encode_chan(1, MAV_COMP_ID_ONBOARD_COMPUTER, MAVLINK_COMM_0, &message, &systemData.gpsSystem);
            uint8_t buffer[MAVLINK_MAX_PACKET_LEN];
            uint16_t length = mavlink_msg_to_send_buffer(buffer, &message);
            Serial.write(buffer, length);
            xTaskNotify(ledBlinkerTaskHandle, BlinkRate::Pulse, eSetValueWithOverwrite); // Notify the LED blinker task that a message is being sent
        }           
        vTaskDelay(pdMS_TO_TICKS(100));
    }
}


float CalculateVoltagePrimaryResistor(const float pin_voltage, const float sensor_output_ratio, const int32_t primary_resistance, const int32_t burden_resistance);
float CalculateInputVoltage(const float voltage_primary_resistor_drop, const float primary_voltage_divider_ratio);
float LinearCorrection(const float input_value, const float slope, const float intercept);
float CalculateCurrentLA55(const float pin_voltage, const float sensor_output_ratio, const int32_t burden_resistance);
float CalculateCurrentT201(const float pin_voltage, const float selected_full_scale_range, const int32_t burden_resistance);
void InstrumentationReaderTask(void* parameter) {

     // The ADS1115 is a Delta-sigma (ΔΣ) ADC, which is based on the principle of oversampling. The input
    // signal of a ΔΣ ADC is sampled at a high frequency (modulator frequency) and subsequently filtered and
    // decimated in the digital domain to yield a conversion result at the respective output data rate.
    // The ratio between modulator frequency and output data rate is called oversampling ratio (OSR). By increasing the OSR, and thus
    // reducing the output data rate, the noise performance of the ADC can be optimized. In other words, the input-
    // referred noise drops when reducing the output data rate because more samples of the internal modulator are
    // averaged to yield one conversion result. Increasing the gain, therefore reducing the input voltage range, also reduces the input-referred noise, which is
    // particularly useful when measuring low-level signals

    // The use of an external ADC, the ADS1115, was chosen to obtain higher resolution and linearity, as well as programmable gain to avoid the need for instrumentation amplifiers.
    // The ADS1115 is a 16-bit ADC with 4 channels. It is used to read the voltage of the battery and the current of motor, the MPPT output and the battery current or auxiliary system current.
    // The ADS1115 has 4 addresses, which are determined by the state of the ADDR pin. Our board has a solder bridge that allows selection between 0x48 and 0x49.
    // The ADS1115 is connected to the ESP32 via I2C. The ESP32 is the master and the ADS1115 is the slave. It uses the default Wire instance at pins 21(SDA) and 22(SCL) for communication.


    // Make sure that the ADS1115 is connected to the ESP32 via I2C and that the solder bridge is set to the correct address.
    // A common ground is also required between the ESP32 and the ADS1115, which is given when the ESP32 is powered by the same battery as the ADS1115,
    // but not when the ESP32 is powered by the USB port during tests on the laboratory workbench. In this case, the ground of the ESP32 and the ADS1115
    // must be explictly connected together for the I2C communication to work. If the ADS1115 is not detected, check continuity of the wires with multimeter.
    
    Adafruit_ADS1115 adc; 
    constexpr uint8_t adc_addresses[] = {0x48, 0x49}; // Address is determined by a solder bridge on the instrumentation board.
    adc.setGain(GAIN_FOUR); // Configuring the PGA( Programmable Gain Amplifier) to amplify the signal by 4 times, so that the maximum input voltage is +/- 1.024V
    adc.setDataRate(RATE_ADS1115_16SPS); // Setting a low data rate to increase the oversampling ratio of the ADC and thus reduce the noise.
    
    bool is_adc_initialized = false;
    
    while (!is_adc_initialized) {
        xTaskNotify(ledBlinkerTaskHandle, BlinkRate::Fast, eSetValueWithOverwrite); // Blinks the LED to indicate that the ADC is not initialized yet.
        for (auto address : adc_addresses) {
            Serial.printf("\n[ADS]Trying to initialize ADS1115 at address 0x%x\n", address);
            if (adc.begin(address)) {
                Serial.printf("\n[ADS]ADS1115 successfully initialized at address 0x%x\n", address);
                is_adc_initialized = true;
                xTaskNotify(ledBlinkerTaskHandle, BlinkRate::Slow, eSetValueWithOverwrite); // Return LED to default blink rate.
                break;
            }
            vTaskDelay(pdMS_TO_TICKS(1000));
        }
    }
    
    while (true) {
        // Check and confirm which values of resistors are being used on the board.
        // Values associated with the voltage sensor.
        constexpr float voltage_conversion_ratio = 2.59081f; // Datasheet gives a reference value of 2.50, but here it is being used an iterative process to find a value that satisfies the conversion measurements.
        constexpr int32_t voltage_primary_resistance = 4700; // Resistor connected to primary side of LV-20P voltage sensor.
        constexpr int32_t voltage_primary_coil_resistance = 250; // Resistance of the primary coil of the LV-20P voltage sensor.
        constexpr float primary_voltage_divider_ratio  = (float)voltage_primary_coil_resistance / voltage_primary_resistance;
        constexpr int32_t voltage_burden_resistance = 33; // Burden resistor connected to secondary side of LV-20P voltage sensor.

        // Values associated with current sensors.
        constexpr int32_t selected_full_scale_range = 100; // Selected full scale range of the T201 current sensor.
        constexpr float current_conversion_ratio = 0.001f; // Output Conversion ratio of the LA55-P current sensor.
        constexpr int32_t motor_burden_resistance = 22;
        constexpr int32_t battery_burden_resistance = 22;
        constexpr int32_t mppt_burden_resistance = 10; 

        // In the ADS1115 single ended measurements have 15 bits of resolution. Only differential measurements have 16 bits of resolution.
        // As we are using the 4 analog inputs for each of the 4 sensors, single ended measurements are being used in order to access all 4 sensors.
        // When using single ended mode, the maximum output code is 0x7FFF(32767), which corresponds to the full-scale input voltage.

        float battery_voltage_pin_voltage = adc.computeVolts(adc.readADC_SingleEnded(0));
        float motor_current_pin_voltage = adc.computeVolts(adc.readADC_SingleEnded(1));
        float current_battery_pin_voltage = adc.computeVolts(adc.readADC_SingleEnded(2));
        float current_mppt_pin_voltage = adc.computeVolts(adc.readADC_SingleEnded(3));
        //DEBUG_PRINTF("\n[Instrumentation-PIN-VOLTAGE]Battery voltage: %f, Motor voltage: %f, Battery voltage: %f, MPPT voltage: %f\n", battery_voltage_pin_voltage, motor_current_pin_voltage, current_battery_pin_voltage, current_mppt_pin_voltage);

        // Calibrate the voltage by comparing the value of voltage_primary_resistor_drop variable against the actual voltage drop on the primary resistor using a multimeter. 
        // Take multiple readings across different voltages and do a linear regression to find the slope and intercept.
        float voltage_primary_resistor_drop = CalculateVoltagePrimaryResistor(battery_voltage_pin_voltage, voltage_conversion_ratio, voltage_primary_resistance, voltage_burden_resistance);
        float battery_voltage = CalculateInputVoltage(voltage_primary_resistor_drop, primary_voltage_divider_ratio);
        float calibrated_battery_voltage = LinearCorrection(battery_voltage, 1.0025059f, 0.0f);
        
        float motor_current = CalculateCurrentT201(motor_current_pin_voltage, selected_full_scale_range, motor_burden_resistance);
        float battery_current = CalculateCurrentT201(current_battery_pin_voltage, selected_full_scale_range, battery_burden_resistance);
        float current_mppt = CalculateCurrentLA55(current_mppt_pin_voltage, current_conversion_ratio, mppt_burden_resistance);
        if (systemData.debug_print & SystemData::debug_print_flags::Instrumentation) {
            DEBUG_PRINTF( "\n[Instrumentation]Primary resistor voltage drop: %fV\n"
                            "[Instrumentation]Battery: %fV\n"
                            "[Instrumentation]Calibrated battery: %fA\n"
                            "[Instrumentation]Motor current: %fA\n"
                            "[Instrumentation]Battery current: %fA\n"
                            "[Instrumentation]MPPT current: %fA\n",
            voltage_primary_resistor_drop, battery_voltage, calibrated_battery_voltage, motor_current, battery_current, current_mppt);
        }

        systemData.instrumentationSystem.battery_voltage = calibrated_battery_voltage;
        systemData.instrumentationSystem.motor_current = motor_current;
        systemData.instrumentationSystem.battery_current = battery_current;
        systemData.instrumentationSystem.mppt_current = current_mppt;

        // Prepare and send Mavlink message
        mavlink_message_t message;
        mavlink_instrumentation_t instrumentation = systemData.instrumentationSystem;
        
        mavlink_msg_instrumentation_encode_chan(1, MAV_COMP_ID_ONBOARD_COMPUTER, MAVLINK_COMM_0, &message, &instrumentation);
        uint8_t buffer[MAVLINK_MAX_PACKET_LEN];
        uint16_t len = mavlink_msg_to_send_buffer(buffer, &message);
        Serial.write(buffer, len);

        xTaskNotify(ledBlinkerTaskHandle, BlinkRate::Pulse, eSetValueWithOverwrite); // Blink LED to indicate that a message has been sent.
        vTaskDelay(pdMS_TO_TICKS(8000));
    }
}

/// @brief Calculates the voltage drop across the primary resistor for a LV-20P voltage sensor, from which the input voltage can be later calculated.
/// @param pin_voltage Voltage at corresponding pin of ADS1115.
/// @param primary_resistance Value of resistor connected to primary side of LV-20P voltage sensor. It should be rated for 10mA for nominal RMS voltage being measured, therefore 14mA is the allowed peak current.
/// @param burden_resistance Value of resistor connected to secondary side of LV-20P voltage sensor. Current through this resistor creates a voltage drop that is measured by the ADS1115. Low gain is preferred to reduce noise.
/// @param sensor_output_ratio 2.50 is a nominal ratio given by the datasheet. Both resistors and the sensor have tolerances, but, by assuming the resistors are accurate,
/// the sensor output ratio can be varied to calibrate the voltage measurement.
/// @return Voltage drop across primary resistor of LV-20P voltage sensor.
float CalculateVoltagePrimaryResistor(const float pin_voltage, const float sensor_output_ratio, const int32_t primary_resistance, const int32_t burden_resistance) {
    
    return pin_voltage * primary_resistance / (burden_resistance * sensor_output_ratio);
}

/// @brief Calculates the input voltage for a LV-20P voltage sensor given the voltage drop across the primary resistor.
/// @param voltage_primary_resistor_drop Voltage drop across primary resistor of LV-20P voltage sensor.
/// @param primary_voltage_divider_ratio Ratio between the primary coil resistance and the resistance connected to the primary side of LV-20P voltage sensor.
/// @return Input voltage at primary side of LV-20P voltage sensor.
float CalculateInputVoltage(const float voltage_primary_resistor_drop, const float primary_voltage_divider_ratio) {
    
    return voltage_primary_resistor_drop + voltage_primary_resistor_drop * primary_voltage_divider_ratio;
}

/// @brief Calculates the input current for a LA-55P current sensor.
/// @param pin_voltage Voltage at corresponding pin of ADS1115.
/// @param burden_resistance Value of resistor connected to secondary side of LV-55P current sensor. Current through this resistor creates a voltage drop that is measured by the ADS1115. Low gain is preferred to reduce noise.
/// @param sensor_output_ratio Current ratio between secondary and primary side of LV-20P voltage sensor. Given by datasheet.
/// @return Input current at primary side of LA-55P current sensor.
float CalculateCurrentLA55(const float pin_voltage, const float sensor_output_ratio, const int32_t burden_resistance) {
    
    return pin_voltage / (burden_resistance * sensor_output_ratio);
}

/// @brief Calculates the input current for a Seneca T201DC 4-20mA loop current sensor by using a linear equation.
/// The 4-20mA loop works by outputting 4mA for zero input and 20mA for full scale input, which get multiplied by the burden resistor to create a voltage drop that is measured by the ADS1115.
/// It has 4 switches. One to set bipolar mode (AC current or reverse current), two bit switches to set the measurement scale and one switch to set damping on or off.
/// By default monopolar mode and no damping are being used for the boat.
/// @param pin_voltage Voltage at corresponding pin of ADS1115.
/// @param burden_resistance Value of resistor connected to secondary side of LV-20P voltage sensor. Current through this resistor creates a voltage drop that is measured by the ADS1115. Low gain is preferred to reduce noise.
/// @param sensor_output_ratio Current ratio between secondary and primary side of LV-20P voltage sensor. Given by datasheet.
/// @return Input current at primary side of LA-55P current sensor.
float CalculateCurrentT201(const float pin_voltage, const float selected_full_scale_range, const int32_t burden_resistance) {
    
    // Calculates the slope and intercept of the linear equation that relates input current to output voltage.
    const float zero_input_voltage = 4.0f * burden_resistance * 0.001f; // 4mA * burden resistor
    const float full_input_voltage = 20.0f * burden_resistance * 0.001f; // 20mA * burden resistor
    const float zero_input_current = 0.0f;
    const float full_input_current = selected_full_scale_range;
    const float slope = (full_input_current - zero_input_current) / (full_input_voltage - zero_input_voltage);
    const float intercept = zero_input_current - slope * zero_input_voltage;
    return slope * pin_voltage + intercept;
}

/// @brief Calibrates a reading by using a linear equation obtained by comparing the readings with a multimeter.
/// @param input 
/// @param slope 
/// @param intercept 
/// @return Calibrated reading
float LinearCorrection(const float input_value, const float slope, const float intercept) {
    return slope * input_value + intercept;
}

void EncoderControlTask(void* parameter) {
    
    constexpr uint8_t dac_pin = 25;
    constexpr uint8_t power_pin = 27;
    constexpr uint8_t dataPin = 14;
    constexpr uint8_t clockPin = 12;

    Encoder encoder(clockPin, dataPin);
    pinMode(power_pin, OUTPUT); digitalWrite(power_pin, HIGH);
 
    static int32_t previousPosition = 0;
    constexpr uint8_t dac_resolution = 255; // 8-bit DAC
    constexpr uint8_t max_number_steps = 50;
    constexpr int16_t max_dac_output_voltage = 3300; // mV
    constexpr int16_t max_dac_amplified_output_voltage = 5000; // mV
    
    static uint32_t print_timer = 0;
    static uint32_t can_print_timer = 0;
    uint32_t mavlink_timer = 0;
    uint32_t mavlink_timer_interval = 6000;
    static bool can_print = false;
    encoder.readAndReset(); // Reset encoder position to zero.
  
    while (true) {
        int32_t currentPosition = encoder.read();
        currentPosition = constrain(currentPosition, 0, max_number_steps);
        if (currentPosition != previousPosition) {
            previousPosition = currentPosition;
            can_print_timer = millis();
            can_print = true;
            uint8_t discrete_output = currentPosition * dac_resolution / max_number_steps;
            dacWrite(dac_pin, discrete_output);

            int32_t output_voltage = currentPosition * max_dac_output_voltage / max_number_steps;
            systemData.controlSystem.dac_output = (float)output_voltage;
        }

        if ((millis() - can_print_timer > 2000) && can_print) {
            // If the encoder has not been moved for 2 seconds, stop printing to the serial port.
            can_print_timer = millis();
            can_print = false;
        }

        if ((millis() - print_timer > 500) && can_print) {
            // Print the encoder position to the serial port every 500ms if the encoder has been moved until after 2 seconds of inactivity.
            print_timer = millis();
            DEBUG_PRINTF("\n[DAC]Encoder position: %d%%\n", currentPosition * 100 / max_number_steps); // Print encoder position as a percentage.
            DEBUG_PRINTF("[DAC]Output: %d mV\n", currentPosition * max_dac_output_voltage / max_number_steps); // Print the output voltage of the DAC.
            DEBUG_PRINTF("[DAC]Amplified output: %d mV\n", currentPosition * max_dac_amplified_output_voltage / max_number_steps); // Print the amplified output voltage of the DAC.
        }

        if (millis() - mavlink_timer > mavlink_timer_interval) {
            // Send mavlink message every interval
            mavlink_timer = millis();
            DEBUG_PRINTF("\n[DAC]Amplified output: %f mV\n", systemData.controlSystem.dac_output);

            mavlink_message_t message;
            mavlink_control_system_t control_system = systemData.controlSystem;

            mavlink_msg_control_system_encode_chan(1, MAV_COMP_ID_ONBOARD_COMPUTER, MAVLINK_COMM_0, &message, &control_system);
            uint8_t buffer[MAVLINK_MAX_PACKET_LEN];
            uint16_t len = mavlink_msg_to_send_buffer(buffer, &message);
            Serial.write(buffer, len);
        }

        vTaskDelay(pdMS_TO_TICKS(50));
    }
}

/// @brief Auxiliary task that reads the battery voltage and state of pumps.
/// @param parameter 
void AuxiliaryReaderTask(void* parameter) {
    
    // Read lead-acid battery and pumps voltage through 4k7-1k voltage divider.
    constexpr uint8_t port_pump_pin = 36;
    constexpr uint8_t starboard_pump_pin = 39;
    constexpr uint8_t battery_voltage_pin = 34;
    constexpr uint8_t battery_current_pin = 35;
    constexpr float battery_voltage_divider_ratio = 1.0f / (4.7f + 1.0f); // Voltage divider ratio used to measure battery voltage.
    constexpr float adc_reference_voltage = 3.3f;
    constexpr uint16_t adc_resolution = 4095; // 12-bit ADC
    constexpr float battery_max_voltage = 13.8f;
    constexpr float battery_min_voltage = 11.8f;
    constexpr float battery_max_voltage_divided = battery_max_voltage * battery_voltage_divider_ratio; 
    constexpr float battery_min_voltage_divided = battery_min_voltage * battery_voltage_divider_ratio; 
    constexpr uint16_t number_samples_filter = 4; // Number of samples to use in the moving average filter.
    constexpr float pump_threshold_voltage = 10.0f; // Voltage at which the pump is considered to be on.

    pinMode(battery_voltage_pin, INPUT);
    pinMode(port_pump_pin, INPUT);
    pinMode(starboard_pump_pin, INPUT);
    pinMode(battery_current_pin, INPUT);

    float aux_battery_voltage = 0.0f;
    float aux_battery_current = 0.0f;
    bool port_pump_voltage = 0.0f;
    bool starboard_pump_voltage = 0.0f;

    /// @brief Read current using ACS712 current sensor.
    /// @param power_voltage Voltage at the power pin of the ACS712 current sensor.
    /// @param pin Pin connected to the output pin of the ACS712 current sensor.
    /// @param sensitivity Sensitivity of the ACS712 current sensor, which is the rise in output voltage per ampere of input current.
    auto ReadBatteryCurrent = [](uint8_t pin, float calibrated_offset_adc, float calibrated_sensitivity) {
       
        float measured_adc = analogRead(pin);
        float measured_current = (measured_adc - calibrated_offset_adc) * calibrated_sensitivity;
        return measured_current;
    };

    auto CalibrateCurrentSensor = [](uint8_t pin, float& offset_adc_reference, float& sensitivity_adc, bool& asked_to_calibrate) {
        // By using non volatile memory, first obtain the calibration factor from the memory. If it is not set, then calibrate the sensor and save the calibration factor to the memory.;
        // If the calibration factor is not set, then 50 readings are taken and averaged to obtain the average offset voltage when no current is flowing through the sensor.
        // Then the user is asked to input the current flowing through the sensor for a new set of 50 readings to obtain the average sensitivity of the sensor.

        Preferences preferences;
        preferences.begin("aux", false);
        constexpr float error_value = -1.0f;
        offset_adc_reference = preferences.getFloat("offset", error_value); 
        sensitivity_adc = preferences.getFloat("sensitivity", error_value); 

        if ((offset_adc_reference < 0.0f || sensitivity_adc < 0.0f) || asked_to_calibrate) {

            auto previous_print_state = systemData.debug_print;
            systemData.debug_print = systemData.debug_print_flags::Auxiliary;
            Serial.printf("\n[AUX]Calibrating current sensor\n"
                            "[AUX]Make sure that no current is flowing through the sensor during initialization\n"
                            "[AUX]Press 'C' to continue\n");
            xTaskNotify(ledBlinkerTaskHandle, BlinkRate::Fast, eSetValueWithOverwrite);
            ulTaskNotifyTake(pdTRUE, portMAX_DELAY);
            
            asked_to_calibrate = false;
            float offset_adc_sum = 0.0f;
            float measured_adc = 0.0f;
            constexpr uint32_t number_samples = 50;
            constexpr uint32_t sample_interval = 100;
            for (uint32_t i = 0; i < number_samples; i++) {
                offset_adc_sum += analogRead(pin);
                vTaskDelay(pdMS_TO_TICKS(sample_interval));
            }
            offset_adc_reference = offset_adc_sum / number_samples;
            Serial.printf("\n[AUX]Offset adc: %.2f\n", offset_adc_reference);
            Serial.printf("\n[AUX]Turn on the current source and input it starting with a 'C'");
            
            uint32_t notification_value;
            while (!xTaskNotifyWait(0, ULONG_MAX, &notification_value, 8000)) {
                Serial.printf("\n[AUX]Please input the current flowing through the sensor starting with a 'C'\n");

            }

            float current = (float)notification_value;
            DEBUG_PRINTF("[AUX]CAL-Current: %.3f\n", current);

            for (uint32_t i = 0; i < number_samples; i++) {
                measured_adc += analogRead(pin);
                vTaskDelay(pdMS_TO_TICKS(sample_interval));
            }
            measured_adc = measured_adc / number_samples;
            sensitivity_adc = current / (measured_adc - offset_adc_reference);
            Serial.printf("\n[AUX]Offset adc: %.2f\n", offset_adc_reference);
            Serial.printf("[AUX]Measured adc: %.2f\n", measured_adc);
            Serial.printf("[AUX]Sensitivity adc: %.2f\n", sensitivity_adc);
            preferences.putFloat("offset", offset_adc_reference);
            preferences.putFloat("sensitivity", sensitivity_adc);
            preferences.end(); 
            systemData.debug_print = previous_print_state;
            xTaskNotify(ledBlinkerTaskHandle, BlinkRate::Slow, eSetValueWithOverwrite);
        }
    };

    static bool asked_to_calibrate = false;
    constexpr float error_value = -1.0f;
    float offset_adc_reference = error_value;
    float sensitivity_adc = error_value;
    CalibrateCurrentSensor(battery_current_pin, offset_adc_reference, sensitivity_adc, asked_to_calibrate);

    while (true) {
        float battery_voltage_reading = (analogRead(battery_voltage_pin) * adc_reference_voltage) / (adc_resolution * battery_voltage_divider_ratio);
        aux_battery_voltage = (battery_voltage_reading + aux_battery_voltage * number_samples_filter) / (number_samples_filter + 1);

        float battery_current_reading = ReadBatteryCurrent(battery_current_pin, offset_adc_reference, sensitivity_adc);
        aux_battery_current = (battery_current_reading + aux_battery_current * number_samples_filter) / (number_samples_filter + 1);

        float port_pump_voltage_reading = (analogRead(port_pump_pin) * adc_reference_voltage) / (adc_resolution * battery_voltage_divider_ratio);
        port_pump_voltage = (port_pump_voltage_reading + port_pump_voltage * number_samples_filter) / (number_samples_filter + 1);

        float starboard_pump_voltage_reading = (analogRead(starboard_pump_pin) * adc_reference_voltage) / (adc_resolution * battery_voltage_divider_ratio);
        starboard_pump_voltage = (starboard_pump_voltage_reading + starboard_pump_voltage * number_samples_filter) / (number_samples_filter + 1);

        bool is_port_pump_on = port_pump_voltage_reading > pump_threshold_voltage;
        bool is_starboard_pump_on = starboard_pump_voltage_reading > pump_threshold_voltage;

        systemData.auxiliarySystem.voltage = aux_battery_voltage;
        systemData.auxiliarySystem.current = aux_battery_current;
        systemData.auxiliarySystem.pumps = (is_port_pump_on << 1) | is_starboard_pump_on;

        static uint32_t print_timer = 0;
        if (millis() - print_timer > 8000) {
            print_timer = millis();
            if (systemData.debug_print & SystemData::debug_print_flags::Auxiliary) {
                DEBUG_PRINTF("\n[AUX]Battery voltage: %.2fV\n", aux_battery_voltage);
                DEBUG_PRINTF("[AUX]Battery current: %.2fA\n", aux_battery_current);
                DEBUG_PRINTF("[AUX]Port pump: %s\n", is_port_pump_on ? "ON" : "OFF");
                DEBUG_PRINTF("[AUX]Starboard pump: %s\n", is_starboard_pump_on ? "ON" : "OFF");
            }

            // Prepare and send mavlink message
            mavlink_message_t message;
            mavlink_aux_system_t aux_system = systemData.auxiliarySystem;

            mavlink_msg_aux_system_encode_chan(1, MAV_COMP_ID_ONBOARD_COMPUTER, MAVLINK_COMM_0, &message, &aux_system);
            uint8_t buffer[MAVLINK_MAX_PACKET_LEN];
            uint16_t len = mavlink_msg_to_send_buffer(buffer, &message);
            xTaskNotify(ledBlinkerTaskHandle, BlinkRate::Pulse, eSetValueWithOverwrite); // Blink LED to indicate that a message has been sent.
            Serial.write(buffer, len);
        }

        if (ulTaskNotifyTake(pdTRUE, pdMS_TO_TICKS(500))) {
            asked_to_calibrate = true;
            CalibrateCurrentSensor(battery_current_pin, offset_adc_reference, sensitivity_adc, asked_to_calibrate);
        }
    }
}

/// @brief Auxiliary task to measure free stack memory of each task and free heap of the system.
/// Useful to detect possible stack overflows on a task and allocate more stack memory for it if necessary.
/// @param parameter Unused. Just here to comply with the task function signature.
void StackHighWaterMeasurerTask(void* parameter) {
    while (true) {
        if (systemData.debug_print & SystemData::debug_print_flags::Temperature) {
            Serial.printf("\n");
            for (int i = 0; i < taskHandlesSize; i++) {
                Serial.printf("[Task]%s has %d bytes of free stack\n", pcTaskGetTaskName(*taskHandles[i]), uxTaskGetStackHighWaterMark(*taskHandles[i]));
            }
            Serial.printf("[Task]System free heap: %d\n", esp_get_free_heap_size());            
        }
        vTaskDelay(pdMS_TO_TICKS(25000));
    }
}

void setup() {\

    Serial.begin(4800);
    Wire.begin(); // Master mode
    xTaskCreate(LedBlinkerTask, "ledBlinker", 2048, NULL, 1, &ledBlinkerTaskHandle);
    xTaskCreate(WifiConnectionTask, "wifiConnection", 4096, NULL, 1, &wifiConnectionTaskHandle);
    xTaskCreate(VPNConnectionTask, "vpnConnection", 4096, NULL, 1, &vpnConnectionTaskHandle);
    xTaskCreate(ServerTask, "server", 4096, NULL, 1, &serverTaskHandle);
    xTaskCreate(SerialReaderTask, "serialReader", 4096, NULL, 1, &serialReaderTaskHandle);
    xTaskCreate(TemperatureReaderTask, "temperatureReader", 4096, NULL, 1, &temperatureReaderTaskHandle);
    xTaskCreate(GpsReaderTask, "gpsReader", 4096, NULL, 2, &gpsReaderTaskHandle);
    xTaskCreate(InstrumentationReaderTask, "instrumentationReader", 4096, NULL, 2, &instrumentationReaderTaskHandle);
    xTaskCreate(AuxiliaryReaderTask, "auxiliaryReader", 4096, NULL, 1, &auxiliaryReaderTaskHandle);
    xTaskCreate(EncoderControlTask, "encoderControl", 4096, NULL, 1, &encoderControlTaskHandle);
    //xTaskCreate(StackHighWaterMeasurerTask, "measurer", 2048, NULL, 1, NULL);  
}

void loop() {
    
}


