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

class SystemData {
public:
    float voltage_battery;
    float current_motor;
    float current_battery;
    float current_mppt;
    float aux_voltage_battery;
    float latitude;
    float longitude;
    float temperature_motor;
    float temperature_mppt;
    bool is_pump_port_on;
    bool is_pump_starboard_on;
};

SystemData systemData = {
    .voltage_battery = 48.0f,
    .current_motor = 0.0f,
    .current_battery = 0.0f,
    .current_mppt = 0.0f,
    .aux_voltage_battery = 0.0f,
    .latitude = -22.909378f,
    .longitude = -43.117346f,
    .temperature_motor = 25.0f,
    .temperature_mppt = 25.0f,
    .is_pump_port_on = false,
    .is_pump_starboard_on = false
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
        request->send(200, "text/html", "<h1>Lora32</h1><p>WiFi connected: " + WiFi.SSID() + "</p><p>IP address: " + WiFi.localIP().toString() + "</p>");
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

                widget_battery_volts.updateNeedle(systemData.voltage_battery, 0);
                widget_battery_current.updateNeedle(systemData.current_motor, 0);
                widget_motor_current.updateNeedle(systemData.current_battery , 0);
                widget_mppt_current.updateNeedle(systemData.current_mppt, 0);
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

                    systemData.voltage_battery = instrumentation.voltage_battery;
                    systemData.current_motor   = instrumentation.current_zero;
                    systemData.current_battery = instrumentation.current_one;
                    systemData.current_mppt    = instrumentation.current_two;

                    DEBUG_PRINTF("[RX]Battery voltage: %f\n", systemData.voltage_battery);
                    DEBUG_PRINTF("[RX]Motor current: %f\n", systemData.current_motor);
                    DEBUG_PRINTF("[RX]Battery current: %f\n", systemData.current_battery);
                    DEBUG_PRINTF("[RX]MPPT current: %f\n", systemData.current_mppt);
                    break;
                }
                case MAVLINK_MSG_ID_TEMPERATURES: {
                    Serial.printf("[RX]Received temperatures from channel %d\n", channel);
                    mavlink_temperatures_t temperatures;
                    mavlink_msg_temperatures_decode(&message, &temperatures);

                    systemData.temperature_motor = temperatures.temperature_motor;
                    systemData.temperature_mppt = temperatures.temperature_mppt;

                   #ifdef DEBUG_PRINTF
                   #define DEVICE_DISCONNECTED_C -127.0f
                    if (systemData.temperature_motor == DEVICE_DISCONNECTED_C) {
                        DEBUG_PRINTF("\n[Temperature]Motor: Probe disconnected\n", NULL);
                    } else {
                        DEBUG_PRINTF("[Temperature]Motor: %f\n", systemData.temperature_motor); // [Temperature][last byte of probe address] = value is the format
                    }

                    if (systemData.temperature_mppt == DEVICE_DISCONNECTED_C) {
                        DEBUG_PRINTF("[Temperature]MPPT: Probe disconnected\n", NULL);
                    } else {
                        DEBUG_PRINTF("[Temperature]MPPT: %f\n", systemData.temperature_mppt);
                    }
                    #endif

                    break;
                }
                case MAVLINK_MSG_ID_GPS_INFO: {
                    Serial.printf("[RX]Received GPS info from channel %d\n", channel);
                    mavlink_gps_info_t gps_info;
                    mavlink_msg_gps_info_decode(&message, &gps_info);

                    systemData.latitude = gps_info.latitude;
                    systemData.longitude = gps_info.longitude;

                    DEBUG_PRINTF("[RX]GPS latitude: %f\n", systemData.latitude);
                    DEBUG_PRINTF("[RX]GPS longitude: %f\n", systemData.longitude);
                    break;
                }
                default: {
                    Serial.printf("[RX]Received message with ID #%d from channel %d\n", message.msgid, channel);
                    break;
                }
            }
            
            // Route received message to serial port
            uint8_t buffer[MAVLINK_MAX_PACKET_LEN];
            uint16_t len = mavlink_msg_to_send_buffer(buffer, &message);
            Serial.write(buffer, len);
            LoRa.beginPacket();
            LoRa.write(buffer, len);
            LoRa.endPacket();
            return true;
        }
    }
    return false;
}

void ChannelReaderTask(void* parameter) {

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

void ReceiveI2CMessage(int number_bytes) {
    Serial.printf("Received %d bytes from [I2C]\n", number_bytes);
    while (Wire.available()) {
        mavlink_message_t message;
        mavlink_status_t status;
        uint8_t received_byte = Wire.read();
        Serial.printf("Received byte: %d\n", received_byte);
        if (mavlink_parse_char(MAVLINK_COMM_1, received_byte, &message, &status)) {
            Serial.print('\n');
            switch (message.msgid) {
                case MAVLINK_MSG_ID_HEARTBEAT: {    
                    DEBUG_PRINTF("Received heartbeat from channel %d\n", MAVLINK_COMM_1);
                    break;
                }
                case MAVLINK_MSG_ID_INSTRUMENTATION: {
                    DEBUG_PRINTF("Received instrumentation from channel %d\n", MAVLINK_COMM_1);
                    mavlink_instrumentation_t instrumentation;
                    mavlink_msg_instrumentation_decode(&message, &instrumentation);

                    systemData.voltage_battery = instrumentation.voltage_battery;
                    systemData.current_motor   = instrumentation.current_zero;
                    systemData.current_battery = instrumentation.current_one;
                    systemData.current_mppt    = instrumentation.current_two;

                    DEBUG_PRINTF("Battery voltage: %f\n", systemData.voltage_battery);
                    DEBUG_PRINTF("Motor current: %f\n", systemData.current_motor);
                    DEBUG_PRINTF("Battery current: %f\n", systemData.current_battery);
                    DEBUG_PRINTF("MPPT current: %f\n", systemData.current_mppt);
                    break;
                }
                default: {
                    Serial.printf("Received message with ID #%d from channel 1\n", message.msgid);
                    break;
                }
            }
            // Route received message to serial port
            uint8_t buffer[MAVLINK_MAX_PACKET_LEN];
            uint16_t len = mavlink_msg_to_send_buffer(buffer, &message);
            Serial.write(buffer, len);
        }
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
        vTaskDelay(pdMS_TO_TICKS(10000));
    }
}

void StartLora() {
    LoRa.setPins(CONFIG_NSS, CONFIG_RST, CONFIG_DIO0); // Use ESP32 pins instead of default Arduino pins set by LoRa constructor
    LoRa.setSyncWord(0xFD);
    while (!LoRa.begin(BAND)) { // Attention: initializes default SPI bus at pins 5, 18, 19, 27
        Serial.println("Starting LoRa failed!");
        vTaskDelay(pdMS_TO_TICKS(1000));
    }
    Serial.println("Starting LoRa succeeded!");
}

void setup() {

    Serial.begin(115200);
    xTaskCreate(LedBlinkerTask, "ledBlinker", 2048, NULL, 1, &ledBlinkerHandle);
    StartLora();
    xTaskCreate(WifiConnectionTask, "wifiConnection", 4096, NULL, 3, &wifiConnectionHandle);
    xTaskCreate(ServerTask, "server", 4096, NULL, 1, &serverTaskHandle);
    //xTaskCreate(SerialReaderTask, "serialReader", 4096, NULL, 1, &serialReaderHandle);
    xTaskCreate(CockpitDisplayTask, "cockpitDisplay", 4096, NULL, 3, &cockpitDisplayHandle);
    xTaskCreate(ChannelReaderTask, "companionReader", 4096, NULL, 1, NULL);
    xTaskCreate(StackHighWaterMeasurerTask, "measurer", 2048, NULL, 1, NULL);  
    Wire.begin(0x04);
    Wire.setBufferSize(MAVLINK_MAX_PACKET_LEN);
    Wire.onReceive([](int number_bytes) { 
                        if (ProcessStreamChannel(Wire, MAVLINK_COMM_1)) {
                            xTaskNotify(ledBlinkerHandle, BlinkRate::Pulse, eSetValueWithOverwrite);
                        }
                    });
}
void loop() {

}


