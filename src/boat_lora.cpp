#include <Arduino.h> // Main Arduino library, required for projects that use the Arduino framework.
#include <WiFi.h> // Main library for WiFi connectivity, also used by AsyncWebServer.
#include <unordered_map> // Hashtable for storing WiFi credentials.
#include <AsyncTCP.h> // Asynchronous TCP library for the ESP32.
#include <ESPAsyncWebServer.h> // Asynchronous web server for the ESP32.
#include <AsyncElegantOTA.h> // Over the air updates for the ESP32.
#include "arariboat\mavlink.h" // Custom mavlink dialect for the boat generated using Mavgen tool.
#include "arariboat\SystemData.hpp" // Singleton class to hold system wide data
#include <Wire.h> // I2C library for communicating with LoRa32 board.
#include <LoRa.h> // SandeepMistry physical layer library
#include "BoardDefinitions.h" // SX1276, SDCard and OLED display pin definitions
#include <ESPmDNS.h> // Allows to resolve hostnames to IP addresses within a local network.
#include <StreamString.h> // Allows to convert a Stream object to a String object.
#include <Preferences.h>
Preferences flashMemory;

//#define DEBUG // Uncomment to enable debug messages.
#ifdef DEBUG
#define DEBUG_PRINTF(message, ...) Serial.printf(message, __VA_ARGS__)
#else
#define DEBUG_PRINTF(message, ...)
#endif

// Declare a handle for each task to allow manipulation of the task from other tasks, such as sending notifications, resuming or suspending.
// The handle is initialized to nullptr to avoid the task being created before the setup() function.
// Each handle is then assigned to the task created in the setup() function.

TaskHandle_t ledBlinkerHandle = nullptr;
TaskHandle_t wifiConnectionHandle = nullptr;
TaskHandle_t serverTaskHandle = nullptr;
TaskHandle_t serialReaderHandle = nullptr;
TaskHandle_t highWaterMeasurerHandle = nullptr;

// Array of pointers to the task handles. This allows to iterate over the array and perform operations on all tasks, such as resuming, suspending or reading free stack memory.
TaskHandle_t* taskHandles[] = { &ledBlinkerHandle, &wifiConnectionHandle, &serverTaskHandle, &serialReaderHandle, &highWaterMeasurerHandle };
constexpr auto taskHandlesSize = sizeof(taskHandles) / sizeof(taskHandles[0]); // Get the number of elements in the array.

enum BlinkRate : uint32_t {
    Slow = 1000,
    Medium = 500,
    Fast = 100,
    Pulse = 1000 // Pulse is a special value that will make the LED blink fast and then return to the previous blink rate.
};

// Tasks can send notifications here to change the blink rate of the LED in order to communicate the status of the boat.
void FastBlinkPulse(int pin);
void LedBlinkerTask(void* parameter) {

    constexpr int ledPin = 25; // Pin 25 is the onboard LED on the TTGO LoRa V2.1.6 board.
    pinMode(ledPin, OUTPUT);

    uint32_t blinkRate = BlinkRate::Slow;
    uint32_t previousBlinkRate = blinkRate;

    // Lambda function to blink the LED fast 4 times and then return to the previous blink rate.
    auto FastBlinkPulse = [](int pin) {
        for (int i = 0; i < 4; i++) {
            digitalWrite(pin, HIGH); vTaskDelay(pdMS_TO_TICKS(50));
            digitalWrite(pin, LOW);  vTaskDelay(pdMS_TO_TICKS(50));
        }
    };
    
    while (true) {
        digitalWrite(ledPin, HIGH); vTaskDelay(pdMS_TO_TICKS(blinkRate));
        digitalWrite(ledPin, LOW);  vTaskDelay(pdMS_TO_TICKS(blinkRate));
        
        // Set blink rate to the value received from the notification
        if (xTaskNotifyWait(0, 0, (uint32_t*)&blinkRate, 0) == pdTRUE) {
            if (blinkRate == BlinkRate::Pulse) {
                FastBlinkPulse(ledPin);
                blinkRate = previousBlinkRate;
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
            xTaskNotify(ledBlinkerHandle, BlinkRate::Fast, eSetValueWithOverwrite);
            for (auto& wifi : wifiCredentials) {
                WiFi.begin(wifi.first, wifi.second);
                Serial.printf("\nTrying to connect to %s\n", wifi.first);
                int i = 0;
                while (WiFi.status() != WL_CONNECTED) {
                    vTaskDelay(pdMS_TO_TICKS(500));
                    Serial.print(".");
                    i++;
                    if (i > 5) {
                        Serial.printf("\nFailed to connect to %s\n", wifi.first);
                        break;
                    }
                }
                if (WiFi.status() == WL_CONNECTED) {
                    Serial.printf("\nConnected to %s\nIP: %s\n", wifi.first, WiFi.localIP().toString().c_str());
                    xTaskNotify(ledBlinkerHandle, BlinkRate::Slow, eSetValueWithOverwrite);
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
        request->send(200, "text/html", "<h1>Boat-Lora</h1><p>WiFi connected: " + WiFi.SSID() + "</p><p>IP address: " + WiFi.localIP().toString() + "</p>");
    });

    server.on("/reset", HTTP_GET, [](AsyncWebServerRequest *request) {
        // log reset message
        request->send(200, "text/html", "<h1>Boat-Lora</h1><p>Resetting...</p>");
        vTaskDelay(pdMS_TO_TICKS(1000));
        ESP.restart();
    });

    server.on("/lora-dump", HTTP_GET, [](AsyncWebServerRequest *request) {
        uint8_t buffer[256];
        StreamString stream;
        LoRa.dumpRegisters(stream);
        request->send(200, "text/html", stream.c_str());
    });

    server.on("/lora-config", HTTP_GET, [](AsyncWebServerRequest *request) {

        flashMemory.begin("lora", false); // Open flash memory partition named "lora" in read-write mode
        String response_message = "<h1>Boat Lora Configuration</h1>";
        bool data_updated = false;

        if (request->hasParam("codingRate4")) {
            int codingRate4 = request->getParam("codingRate4")->value().toInt();
            if (codingRate4 < 5 || codingRate4 > 8) {
                response_message += "<p>Invalid coding rate 4 value. Must be between 5 and 8.</p>";
                request->send(400, "text/html", response_message);
                return;
            }
            LoRa.setCodingRate4(codingRate4);
            flashMemory.putInt("codingRate4", codingRate4);
            response_message += "<p>Coding rate 4 set to " + String(codingRate4) + "</p>";
            data_updated = true;
        }

        if (request->hasParam("bandwidth")) {
            int bandwidth = request->getParam("bandwidth")->value().toInt();
            if (bandwidth < 7E3 || bandwidth > 500E3) {
                response_message += "<p>Invalid bandwidth value. Must be between 7E3 and 500E3.</p>";
                request->send(400, "text/html", response_message);
                return;
            }
            LoRa.setSignalBandwidth(bandwidth);
            flashMemory.putInt("bandwidth", bandwidth);
            response_message += "<p>Bandwidth set to " + String(bandwidth) + "</p>";
            data_updated = true;
        }

        if (request->hasParam("spreadingFactor")) {
            int spreadingFactor = request->getParam("spreadingFactor")->value().toInt();
            if (spreadingFactor < 6 || spreadingFactor > 12) {
                response_message += "<p>Invalid spreading factor value. Must be between 6 and 12.</p>";
                request->send(400, "text/html", response_message);
                return;
            }
            LoRa.setSpreadingFactor(spreadingFactor);
            flashMemory.putInt("spreadingFactor", spreadingFactor);
            response_message += "<p>Spreading factor set to " + String(spreadingFactor) + "</p>";
            data_updated = true;
        }

        if (request->hasParam("crc")) {
            bool crc = request->getParam("crc")->value().equalsIgnoreCase("true");
            flashMemory.putBool("crc", crc);
            if (crc) {
                LoRa.enableCrc();
                response_message += "<p>CRC enabled</p>";
            } else {
                LoRa.disableCrc();
                response_message += "<p>CRC disabled</p>";
            }
            data_updated = true;
        }
        flashMemory.end();

        if (!data_updated) {
            response_message += "<p>No data updated</p>";
            // Retrieve stored values from flash memory
            flashMemory.begin("lora", true); // Open flash memory partition named "lora" in read-only mode
            int stored_spreading_factor = flashMemory.getInt("spreadingFactor", 7);
            int stored_bandwidth = flashMemory.getInt("bandwidth", 125000);
            int stored_coding_rate4 = flashMemory.getInt("codingRate4", 5);
            bool stored_crc = flashMemory.getBool("crc", true);
            flashMemory.end();
            
            response_message += "<p>Spreading factor: " + String(stored_spreading_factor) + "</p>";
            response_message += "<p>Bandwidth: " + String(stored_bandwidth) + "</p>";
            response_message += "<p>Coding rate 4: " + String(stored_coding_rate4) + "</p>";
            response_message += "<p>CRC: " + String(stored_crc) + "</p>";

        }
        request->send(200, "text/html", response_message);
    });

    // Wait for notification from WifiConnection task that WiFi is connected in order to begin the server
    ulTaskNotifyTake(pdTRUE, portMAX_DELAY);
    
    // Allow the server to be accessed by hostname instead of IP address.
    if(!MDNS.begin("boat-lora")) {
        Serial.println("[MDNS]Error starting mDNS!");
    }
    
    // Attach OTA update handler to the server and initialize the server.
    AsyncElegantOTA.begin(&server); // Available at http://[esp32ip]/update or http://[esp32hostname]/update
    server.begin();

    while (true) {
        vTaskDelay(pdMS_TO_TICKS(500));
    }
}

template <std::size_t N>
void ProcessSerialMessage(const std::array<uint8_t, N> &buffer);
void SerialReaderTask(void* parameter) {
    
    std::array<uint8_t, 128> buffer = { 0 };
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
                    xTaskNotify(ledBlinkerHandle, (uint32_t)it->second, eSetValueWithOverwrite);
                    Serial.printf("Blink rate set to %c\n", buffer[1]);
                    break;
                }
                else {
                    Serial.printf("Invalid blink rate: %c\n", buffer[1]);
                    break;
                }
            break;
        }

        case '\r':
        case '\n':
            break;

        default:
            break;
    }
}

bool ProcessStreamChannel(Stream& byte_stream, mavlink_channel_t channel) {

    mavlink_message_t message;
    mavlink_status_t* status = mavlink_get_channel_status(channel);
    
    constexpr uint8_t arariboat_max_msg_len = 17;
    if (status->packet_idx > arariboat_max_msg_len) {
        DEBUG_PRINTF("Channel %d overrun. Resetting status...", channel);
        mavlink_reset_channel_status(channel);
    }

    while (byte_stream.available()) {
        uint8_t received_byte = byte_stream.read();
        if (mavlink_parse_char(channel, received_byte, &message, status)) {
            Serial.print('\n');
            switch (message.msgid) {
                case MAVLINK_MSG_ID_HEARTBEAT: {    
                    DEBUG_PRINTF("[RX]Received heartbeat from channel %d\n", channel);
                    break;
                }
                case MAVLINK_MSG_ID_CONTROL_SYSTEM: {
                    DEBUG_PRINTF("[RX]Received control system from channel %d\n", channel);
                    mavlink_control_system_t control_system;
                    mavlink_msg_control_system_decode(&message, &control_system);
                    systemData.controlSystem = control_system;

                    DEBUG_PRINTF("[CONTROL]DAC output: %.2f\n", control_system.dac_output);
                    DEBUG_PRINTF("[CONTROL]Potentiometer signal: %.2f\n", control_system.potentiometer_signal);
                    break;
                }

                case MAVLINK_MSG_ID_INSTRUMENTATION: {
                    DEBUG_PRINTF("[RX]Received instrumentation from channel %d\n", channel);
                    mavlink_instrumentation_t instrumentation;
                    mavlink_msg_instrumentation_decode(&message, &instrumentation);
                    systemData.instrumentationSystem = instrumentation;

                    DEBUG_PRINTF("[INSTRUMENTATION]Battery voltage: %.2f\n", instrumentation.battery_voltage);
                    DEBUG_PRINTF("[INSTRUMENTATION]Motor current: %.2f\n", instrumentation.motor_current);
                    DEBUG_PRINTF("[INSTRUMENTATION]Battery current: %.2f\n", instrumentation.battery_current);
                    DEBUG_PRINTF("[INSTRUMENTATION]MPPT current: %.2f\n", instrumentation.mppt_current);
                    break;
                }
                case MAVLINK_MSG_ID_TEMPERATURES: {
                    DEBUG_PRINTF("[RX]Received temperatures from channel %d\n", channel);
                    mavlink_temperatures_t temperatures;
                    mavlink_msg_temperatures_decode(&message, &temperatures);
                    systemData.temperatureSystem = temperatures;

                    #ifdef DEBUG_PRINTF
                   #define DEVICE_DISCONNECTED_C -127.0f
                    if (systemData.temperatureSystem.temperature_motor == DEVICE_DISCONNECTED_C) {
                        DEBUG_PRINTF("[Temperature]Motor: Probe disconnected\n", NULL);
                    } else {
                        DEBUG_PRINTF("[Temperature]Motor: %.2f\n", temperatures.temperature_motor); // [Temperature][last byte of probe address] = value is the format
                    }
                    if (systemData.temperatureSystem.temperature_battery == DEVICE_DISCONNECTED_C) {
                        DEBUG_PRINTF("\n[Temperature]Battery: Probe disconnected\n", NULL);
                    } else {
                        DEBUG_PRINTF("[Temperature]Battery: %.2f\n", temperatures.temperature_battery); // [Temperature][last byte of probe address] = value is the format
                    }                  
                    if (systemData.temperatureSystem.temperature_mppt == DEVICE_DISCONNECTED_C) {
                        DEBUG_PRINTF("[Temperature]MPPT: Probe disconnected\n", NULL);
                    } else {
                        DEBUG_PRINTF("[Temperature]MPPT: %.2f\n", temperatures.temperature_mppt);
                    }
                    #endif

                    break;
                }
                case MAVLINK_MSG_ID_GPS_INFO: {
                    DEBUG_PRINTF("[RX]Received GPS info from channel %d\n", channel);
                    mavlink_gps_info_t gps_info;
                    mavlink_msg_gps_info_decode(&message, &gps_info);
                    systemData.gpsSystem = gps_info;

                    DEBUG_PRINTF("[GPS]Latitude: %.2f\n", gps_info.latitude);
                    DEBUG_PRINTF("[GPS]Longitude: %.2f\n", gps_info.longitude);
                    break;
                }
                case MAVLINK_MSG_ID_AUX_SYSTEM: {
                    DEBUG_PRINTF("[RX]Received aux system from channel %d\n", channel);
                    mavlink_aux_system_t aux_system;
                    mavlink_msg_aux_system_decode(&message, &aux_system);
                    systemData.auxiliarySystem = aux_system;

                    DEBUG_PRINTF("[AUX]Aux system voltage: %.2f\n", aux_system.voltage);
                    DEBUG_PRINTF("[AUX]Aux system current: %.2f\n", aux_system.current);
                    DEBUG_PRINTF("[AUX]Aux system pumps: %d\n", aux_system.pumps);
                    break;
                }
                default: {
                    DEBUG_PRINTF("[RX]Received message with ID #%d from channel %d\n", message.msgid, channel);
                    break;
                }
            }
  
            uint8_t buffer[MAVLINK_MAX_PACKET_LEN];
            uint16_t len = mavlink_msg_to_send_buffer(buffer, &message);
            if (!LoRa.beginPacket()) {
                DEBUG_PRINTF("[TX]Failed to begin packet\n", NULL);
                return false;
            }
            LoRa.write(buffer, len);
            LoRa.endPacket();
            return true;
        }
        vTaskDelay(pdMS_TO_TICKS(5));
    }
    return false;
}

void SerialChannelReaderTask(void* parameter) {
 
    while (true) {
        static uint32_t last_reception_time = 0;
        if (millis() - last_reception_time >= 6000) {
            last_reception_time = millis();
            Serial.printf("\n[BOAT-LORA]Waiting for Mavlink on channel %d\n", MAVLINK_COMM_0);
        }

        if (ProcessStreamChannel(Serial, MAVLINK_COMM_0)) {
            xTaskNotify(ledBlinkerHandle, BlinkRate::Pulse, eSetValueWithOverwrite);
            last_reception_time = millis();
        }

        vTaskDelay(25);
    }
}

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

        auto ssid = WiFi.SSID();
        String ip = WiFi.localIP().toString();
        int8_t rssi = WiFi.RSSI();
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
        // Display LoRa status
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

/// @brief Receives Mavlink messages from the routing queue and transmits them via LoRa.
/// @param parameter 
void LoraTransmissionTask(void* parameter) {

    while (true) {
        mavlink_message_t heartbeat_msg;
        mavlink_msg_heartbeat_pack(0, 0, &heartbeat_msg, MAV_TYPE_GCS, MAV_AUTOPILOT_INVALID, MAV_MODE_FLAG_DECODE_POSITION_MANUAL, 0, MAV_STATE_ACTIVE);
        uint8_t buffer[MAVLINK_MAX_PACKET_LEN];
        uint16_t len = mavlink_msg_to_send_buffer(buffer, &heartbeat_msg);
        if (!LoRa.beginPacket()) {
            DEBUG_PRINTF("[TX]Failed to begin packet\n", NULL);
            return;
        }
        LoRa.write(buffer, len);
        LoRa.endPacket();
        vTaskDelay(pdMS_TO_TICKS(1000));
    }
}

/// @brief Auxiliary task to measure free stack memory of each task and free heap of the system.
/// Useful to detect possible stack overflows on a task and allocate more stack memory for it if necessary.
/// @param parameter Unused. Just here to comply with the task function signature.
void StackHighWaterMeasurerTask(void* parameter) {

    while (true) {
        Serial.printf("\n");
        for (int i = 0; i < taskHandlesSize; i++) {
            Serial.printf("[Task] %s has %d bytes of free stack\n", pcTaskGetTaskName(*taskHandles[i]), uxTaskGetStackHighWaterMark(*taskHandles[i]));
        }
        Serial.printf("[Task]System free heap: %d\n", esp_get_free_heap_size());
        vTaskDelay(pdMS_TO_TICKS(25000));
    }
}

void StartLora() {
    LoRa.setPins(CONFIG_NSS, CONFIG_RST, CONFIG_DIO0); // Use ESP32 pins instead of default Arduino pins set by LoRa constructor
    while (!LoRa.begin(BAND)) { // Attention: initializes default SPI bus at pins 5, 18, 19, 27
        Serial.println("Starting LoRa failed!");
        vTaskDelay(pdMS_TO_TICKS(1000));
    }
    Serial.println("Starting LoRa succeeded!");

    flashMemory.begin("lora", true); // Open flash memory partition named "lora" in read-only mode
    int stored_coding_rate4 = flashMemory.getInt("codingRate4", 5);
    int stored_bandwidth = flashMemory.getInt("bandwidth", 125000);
    int stored_spreading_factor = flashMemory.getInt("spreadingFactor", 7);
    bool stored_crc = flashMemory.getBool("crc", true);

    Serial.printf("Stored coding rate 4: %d\n", stored_coding_rate4);
    Serial.printf("Stored bandwidth: %d\n", stored_bandwidth);
    Serial.printf("Stored spreading factor: %d\n", stored_spreading_factor);
    Serial.printf("Stored CRC: %d\n", stored_crc);
    
    // Set LoRa parameters by writing to registers after SPI bus is initialized
    LoRa.setSyncWord(SYNC_WORD);
    LoRa.setCodingRate4(stored_coding_rate4);
    LoRa.setSignalBandwidth(stored_bandwidth);
    LoRa.setSpreadingFactor(stored_spreading_factor);
    if (stored_crc) {
        LoRa.enableCrc();
    } else {
        LoRa.disableCrc();
    }

    flashMemory.end();
}

void setup() {

    Serial.begin(4800);
    xTaskCreate(LedBlinkerTask, "ledBlinker", 2048, NULL, 1, &ledBlinkerHandle);
    xTaskCreate(DisplayScreenTask, "displayScreen", 4096, NULL, 1, NULL);
    StartLora();
    xTaskCreate(WifiConnectionTask, "wifiConnection", 4096, NULL, 1, &wifiConnectionHandle);
    xTaskCreate(ServerTask, "server", 4096, NULL, 1, &serverTaskHandle);
    xTaskCreate(SerialReaderTask, "serialReader", 4096, NULL, 1, &serialReaderHandle);
    xTaskCreate(SerialChannelReaderTask, "serialReader", 4096, NULL, 3, NULL);
    xTaskCreate(LoraTransmissionTask, "loraTransmission", 4096, NULL, 1, NULL);
    xTaskCreate(StackHighWaterMeasurerTask, "measurer", 2048, NULL, 1, NULL);  
}
void loop() {

}


