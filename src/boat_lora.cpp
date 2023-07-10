#include <Arduino.h> // Main Arduino library, required for projects that use the Arduino framework.
#include <WiFi.h> // Main library for WiFi connectivity, also used by AsyncWebServer.
#include <unordered_map> // Hashtable for storing WiFi credentials.
#include <AsyncTCP.h> // Asynchronous TCP library for the ESP32.
#include <ESPAsyncWebServer.h> // Asynchronous web server for the ESP32.
#include <AsyncElegantOTA.h> // Over the air updates for the ESP32.
#include <TFT_eSPI.h>     // Hardware-specific library
#include <TFT_eWidget.h>  // Widget library
#include "arariboat\mavlink.h" // Custom mavlink dialect for the boat generated using Mavgen tool.
#include <Wire.h> // I2C library for communicating with LoRa32 board.
#include <LoRa.h> // SandeepMistry physical layer library
#include "BoardDefinitions.h" // SX1276, SDCard and OLED display pin definitions
#include <ESPmDNS.h> // Allows to resolve hostnames to IP addresses within a local network.

#define DEBUG // Uncomment to enable debug messages.
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
TaskHandle_t cockpitDisplayHandle = nullptr;
TaskHandle_t highWaterMeasurerHandle = nullptr;

// Array of pointers to the task handles. This allows to iterate over the array and perform operations on all tasks, such as resuming, suspending or reading free stack memory.
TaskHandle_t* taskHandles[] = { &ledBlinkerHandle, &wifiConnectionHandle, &serverTaskHandle, &serialReaderHandle, &cockpitDisplayHandle, &highWaterMeasurerHandle };
constexpr auto taskHandlesSize = sizeof(taskHandles) / sizeof(taskHandles[0]); // Get the number of elements in the array.

// Singleton class for storing system-data that needs to be accessed by multiple tasks.
class SystemData {

public:
    static SystemData& getInstance() {
        static SystemData instance;
        return instance;
    }

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
    }
    SystemData(SystemData const&) = delete; // Delete copy constructor.
    SystemData& operator=(SystemData const&) = delete; // Delete assignment operator.
    SystemData(SystemData&&) = delete; // Delete move constructor.
};

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

// For some reason, the display gets a bug if I declare the display objects inside the task, so I declare them here
// with static storage duration instead of using the default thread storage duration of the task.
TFT_eSPI tft_display = TFT_eSPI(); // Object to control the TFT display
MeterWidget widget_battery_volts    = MeterWidget(&tft_display);
MeterWidget widget_battery_current  = MeterWidget(&tft_display);
MeterWidget widget_motor_current    = MeterWidget(&tft_display);
MeterWidget widget_mppt_current     = MeterWidget(&tft_display);

void CockpitDisplayTask(void* parameter) {

    //Needs Font 2 (also Font 4 if using large scale label)
    //Make sure all the display driver and pin connections are correct by
    //editing the User_Setup.h file in the TFT_eSPI library folder.

    constexpr float battery_volts_full_scale = 54.0;
    constexpr float battery_volts_zero_scale = 48.0;
    constexpr float battery_amps_full_scale = 60.0;
    constexpr float battery_amps_zero_scale = 0.0;
    constexpr float motor_amps_full_scale = 60.0;
    constexpr float motor_amps_zero_scale = 0.0;
    constexpr float mppt_amps_full_scale = 40.0;
    constexpr float mppt_amps_zero_scale = 0.0;
    constexpr float widget_length = 239.0f;
    constexpr float widget_height = 126.0f;

    tft_display.init();
    tft_display.setRotation(3);
    tft_display.fillScreen(TFT_BLACK);
    tft_display.drawString("Corrente-Bateria", 240 + widget_length / 7, 2, 4);
    tft_display.drawString("Tensao-Bateria", widget_length / 7, 2, 4);
    tft_display.drawString("Corrente-Motor", widget_length / 7, 160, 4);
    tft_display.drawString("Corrente-MPPT", 240 + widget_length / 7, 160, 4);

    // Colour zones are set as a start and end percentage of full scale (0-100)
    // If start and end of a colour zone are the same then that colour is not used
    //                              -Red-   -Org-  -Yell-  -Grn-
    widget_battery_volts.setZones(0, 100, 15, 25, 0, 0, 25, 100);
    widget_battery_volts.analogMeter(0, 30, battery_volts_zero_scale, battery_volts_full_scale, "V", "48.0", "49.5", "51.0", "52.5", "54.0"); 

    //                              --Red--  -Org-   -Yell-  -Grn-
    widget_battery_current.setZones(75, 100, 50, 75, 25, 50, 0, 25); // Example here red starts at 75% and ends at 100% of full scale
    widget_battery_current.analogMeter(240, 30, battery_amps_zero_scale, battery_amps_full_scale, "A", "0", "15", "30", "45", "60"); 

    //                              -Red-   -Org-  -Yell-  -Grn-
    widget_motor_current.setZones(75, 100, 50, 75, 25, 50, 0, 25); // Example here red starts at 75% and ends at 100% of full scale
    widget_motor_current.analogMeter(0, 180, motor_amps_zero_scale, motor_amps_full_scale, "A", "0", "15", "30", "45", "60"); 

    //                           -Red-   -Org-  -Yell-  -Grn-
    widget_mppt_current.setZones(75, 100, 50, 75, 25, 50, 0, 25); // Example here red starts at 75% and ends at 100% of full scale
    widget_mppt_current.analogMeter(240, 180, mppt_amps_zero_scale, mppt_amps_full_scale, "A", "0", "10", "20", "30", "40"); 
  
    while (true) {
        constexpr int loop_period = 35; // Display updates every 35 ms
        static uint32_t update_time = 0;  

        auto test_sine_wave = [&]() {
            static uint32_t test_update_time = 0;
            static float angle = 0.0f;
            constexpr float radians_to_degrees = 3.14159265358979323846 / 180.0;

            if (millis() - test_update_time > loop_period) {
                test_update_time = millis();
                angle += 4; if (angle > 360) angle = 0;

                // Create a Sine wave for testing, value is in range 0 - 100
                float value = 50.0 + 50.0 * sin(angle * radians_to_degrees);

                auto mapValue = [](float ip, float ipmin, float ipmax, float tomin, float tomax) {
                    return (ip - ipmin) * (tomax - tomin) / (ipmax - ipmin) + tomin;
                };

                float battery_current;
                battery_current = mapValue(value, (float)0.0, (float)100.0, battery_amps_zero_scale, battery_amps_full_scale);
                widget_battery_current.updateNeedle(battery_current, 0);

                float battery_voltage;
                battery_voltage = mapValue(value, (float)0.0, (float)100.0, battery_volts_zero_scale, battery_volts_full_scale);
                widget_battery_volts.updateNeedle(battery_voltage, 0);

                float motor_current;
                motor_current = mapValue(value, (float)0.0, (float)100.0, motor_amps_zero_scale, motor_amps_full_scale);
                widget_motor_current.updateNeedle(motor_current, 0);

                float mppt_current;
                mppt_current = mapValue(value, (float)0.0, (float)100.0, mppt_amps_zero_scale, mppt_amps_full_scale);
                widget_mppt_current.updateNeedle(mppt_current, 0);
            }
        };

        // Use data from SystemData static class to update the display

        auto update_display = [&]() {
            if (millis() - update_time > loop_period) {
                update_time = millis();

                widget_battery_volts.updateNeedle(SystemData::getInstance().instrumentation.voltage_battery, 0);
                widget_battery_current.updateNeedle(SystemData::getInstance().instrumentation.current_zero, 0);
                widget_motor_current.updateNeedle(SystemData::getInstance().instrumentation.current_one , 0);
                widget_mppt_current.updateNeedle(SystemData::getInstance().instrumentation.current_two, 0);
            }
        };

        update_display();
        vTaskDelay(pdMS_TO_TICKS(10));
    }
}

bool ProcessStreamChannel(Stream& byte_stream, mavlink_channel_t channel) {

    mavlink_message_t message;
    mavlink_status_t status; 

    while (byte_stream.available()) {
        uint8_t received_byte = byte_stream.read();
        if (mavlink_parse_char(channel, received_byte, &message, &status)) {
            Serial.print('\n');
            switch (message.msgid) {
                case MAVLINK_MSG_ID_HEARTBEAT: {    
                    DEBUG_PRINTF("[RX]Received heartbeat from channel %d\n", channel);
                    break;
                }
                case MAVLINK_MSG_ID_INSTRUMENTATION: {
                    Serial.printf("[RX]Received instrumentation from channel %d\n", channel);
                    mavlink_instrumentation_t instrumentation;
                    mavlink_msg_instrumentation_decode(&message, &instrumentation);

                    SystemData::getInstance().instrumentation.voltage_battery = instrumentation.voltage_battery;
                    SystemData::getInstance().instrumentation.current_zero    = instrumentation.current_zero;
                    SystemData::getInstance().instrumentation.current_one     = instrumentation.current_one;
                    SystemData::getInstance().instrumentation.current_two     = instrumentation.current_two;

                    DEBUG_PRINTF("[RX]Battery voltage: %f\n", instrumentation.voltage_battery);
                    DEBUG_PRINTF("[RX]Motor current: %f\n", instrumentation.current_zero);
                    DEBUG_PRINTF("[RX]Battery current: %f\n", instrumentation.current_one);
                    DEBUG_PRINTF("[RX]MPPT current: %f\n", instrumentation.current_two);
                    break;
                }
                case MAVLINK_MSG_ID_TEMPERATURES: {
                    Serial.printf("[RX]Received temperatures from channel %d\n", channel);
                    mavlink_temperatures_t temperatures;
                    mavlink_msg_temperatures_decode(&message, &temperatures);

                    SystemData::getInstance().temperature.temperature_motor = temperatures.temperature_motor;
                    SystemData::getInstance().temperature.temperature_mppt = temperatures.temperature_mppt;

                   #ifdef DEBUG_PRINTF
                   #define DEVICE_DISCONNECTED_C -127.0f
                    if (temperatures.temperature_motor == DEVICE_DISCONNECTED_C) {
                        DEBUG_PRINTF("\n[Temperature]Motor: Probe disconnected\n", NULL);
                    } else {
                        DEBUG_PRINTF("[Temperature]Motor: %f\n", temperatures.temperature_motor); // [Temperature][last byte of probe address] = value is the format
                    }

                    if (temperatures.temperature_mppt == DEVICE_DISCONNECTED_C) {
                        DEBUG_PRINTF("[Temperature]MPPT: Probe disconnected\n", NULL);
                    } else {
                        DEBUG_PRINTF("[Temperature]MPPT: %f\n", temperatures.temperature_mppt);
                    }
                    #endif

                    break;
                }
                case MAVLINK_MSG_ID_GPS_INFO: {
                    Serial.printf("[RX]Received GPS info from channel %d\n", channel);
                    mavlink_gps_info_t gps_info;
                    mavlink_msg_gps_info_decode(&message, &gps_info);

                    SystemData::getInstance().gps.latitude = gps_info.latitude;
                    SystemData::getInstance().gps.longitude = gps_info.longitude;

                    DEBUG_PRINTF("[RX]GPS latitude: %f\n", gps_info.latitude);
                    DEBUG_PRINTF("[RX]GPS longitude: %f\n", gps_info.longitude);
                    break;
                }
                default: {
                    Serial.printf("[RX]Received message with ID #%d from channel %d\n", message.msgid, channel);
                    break;
                }
            }

            // Route to radio queue
            //xQueueSend(routing_queue, &message, 0);    
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
        if (millis() - last_reception_time >= 10000) {
            last_reception_time = millis();
            Serial.printf("\n[CHANNEL]Waiting for Mavlink on channel %d\n", MAVLINK_COMM_0);
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

    LoRa.setPins(CONFIG_NSS, CONFIG_RST, CONFIG_DIO0); // Use ESP32 pins instead of default Arduino pins set by LoRa constructor
    LoRa.setSyncWord(SYNC_WORD);
    while (!LoRa.begin(BAND)) { // Attention: initializes default SPI bus at pins 5, 18, 19, 27
        Serial.println("Starting LoRa failed!");
        vTaskDelay(pdMS_TO_TICKS(1000));
    }
    Serial.println("Starting LoRa succeeded!");

    // QueueHandle_t is already a pointer, so there is no need to use the & operator when passing to the task
    // nor casting it to a pointer again when receiving it.
    QueueHandle_t routing_queue = (QueueHandle_t)parameter;

    while (true) {
        mavlink_message_t message;
        if (xQueueReceive(routing_queue, &message, pdMS_TO_TICKS(1000))) {
            uint8_t buffer[MAVLINK_MAX_PACKET_LEN];
            uint16_t len = mavlink_msg_to_send_buffer(buffer, &message);
            LoRa.beginPacket();
            LoRa.write(buffer, len);
            LoRa.endPacket();
            xTaskNotify(ledBlinkerHandle, BlinkRate::Pulse, eSetValueWithOverwrite); // Notify LED blinker task to blink LED
            DEBUG_PRINTF("Sent message with ID %d\n", message.msgid);
        }
        vTaskDelay(pdMS_TO_TICKS(100));
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
    LoRa.setSyncWord(SYNC_WORD);
    LoRa.setCodingRate4(5);
    LoRa.setSignalBandwidth(500E3);
    LoRa.setSpreadingFactor(7);
    while (!LoRa.begin(BAND)) { // Attention: initializes default SPI bus at pins 5, 18, 19, 27
        Serial.println("Starting LoRa failed!");
        vTaskDelay(pdMS_TO_TICKS(1000));
    }
    Serial.println("Starting LoRa succeeded!");
}

void setup() {

    Serial.begin(4800);
    xTaskCreate(LedBlinkerTask, "ledBlinker", 2048, NULL, 1, &ledBlinkerHandle);
    xTaskCreate(DisplayScreenTask, "displayScreen", 4096, NULL, 1, NULL);
    StartLora();
    xTaskCreate(WifiConnectionTask, "wifiConnection", 4096, NULL, 3, &wifiConnectionHandle);
    xTaskCreate(ServerTask, "server", 4096, NULL, 1, &serverTaskHandle);
    //xTaskCreate(SerialReaderTask, "serialReader", 4096, NULL, 1, &serialReaderHandle);
    //xTaskCreate(CockpitDisplayTask, "cockpitDisplay", 4096, NULL, 3, &cockpitDisplayHandle);
    xTaskCreate(SerialChannelReaderTask, "serialReader", 4096, NULL, 1, NULL);
    //xTaskCreate(LoraTransmissionTask, "loraTransmission", 4096, NULL, 1, NULL);
    xTaskCreate(StackHighWaterMeasurerTask, "measurer", 2048, NULL, 1, NULL);  
}
void loop() {

}


