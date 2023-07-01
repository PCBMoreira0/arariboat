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

// Declare a handle for each task to allow manipulation of the task from other tasks, such as sending notifications, resuming or suspending.
// The handle is initialized to nullptr to avoid the task being created before the setup() function.
// Each handle is then assigned to the task created in the setup() function.

TaskHandle_t ledBlinkerHandle = nullptr;
TaskHandle_t displayScreenHandle = nullptr;
TaskHandle_t wifiConnectionHandle = nullptr;
TaskHandle_t serverTaskHandle = nullptr;
TaskHandle_t serialReaderHandle = nullptr;
TaskHandle_t loraReceiverHandle = nullptr;
TaskHandle_t highWaterMeasurerHandle = nullptr;

QueueHandle_t mavlinkQueue = nullptr;

// Array of pointers to the task handles. This allows to iterate over the array and perform operations on all tasks, such as resuming, suspending or reading free stack memory.
TaskHandle_t* taskHandles[] = { &ledBlinkerHandle, &displayScreenHandle, &wifiConnectionHandle, &serverTaskHandle, &serialReaderHandle, &loraReceiverHandle, &highWaterMeasurerHandle };
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
     bool is_pump_port_on;
     bool is_pump_starboard_on;
     uint8_t mavlink_channel_flags;
};

SystemData systemData = { 48.0f, 0.0f, 0.0f, 0.0f, 0.0f, -22.909378f, -43.117346f, false, false, 0x00};

enum BlinkRate : uint32_t {
    Slow = 1000,
    Medium = 500,
    Fast = 100,
    Pulse = 100
};

enum LoraStatus : uint32_t {
    Failed,
    Idle,
    Sending,
    Receiving
};

#define DEBUG // Uncomment to enable debug messages.
#ifdef DEBUG
#define DEBUG_PRINTF(message, ...) Serial.printf(message, __VA_ARGS__)
#else
#define DEBUG_PRINTF(message, ...)
#endif

// Tasks can send notifications here to change the blink rate of the LED in order to communicate the status of the boat.
void FastBlinkPulse(int pin);
void LedBlinkerTask(void* parameter) {

    constexpr int ledPin = 25;
    pinMode(ledPin, OUTPUT);

    uint32_t blinkRate = BlinkRate::Slow;
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
            xTaskNotify(ledBlinkerHandle, BlinkRate::Fast, eSetValueWithOverwrite);
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
                    Serial.printf("Connected to %s\nIP: %s\n", wifi.first, WiFi.localIP().toString().c_str());
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

bool ProcessStreamChannel(Stream& byte_stream, mavlink_channel_t channel) {

    mavlink_message_t message;
    mavlink_status_t status;

    while (byte_stream.available()) {
        uint8_t received_byte = byte_stream.read();
        if (mavlink_parse_char(channel, received_byte, &message, &status)) {
            Serial.print('\n');
            switch (message.msgid) {
                case MAVLINK_MSG_ID_HEARTBEAT: {    
                    DEBUG_PRINTF("Received heartbeat from channel %d\n", channel);
                    break;
                }
                case MAVLINK_MSG_ID_INSTRUMENTATION: {
                    Serial.printf("Received instrumentation from channel %d\n", channel);
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
                    Serial.printf("Received message with ID #%d from channel %d\n", message.msgid, channel);
                    break;
                }
            }
            // Route received message to serial port
            
            uint8_t buffer[MAVLINK_MAX_PACKET_LEN];
            uint16_t len = mavlink_msg_to_send_buffer(buffer, &message);
            Serial.write(buffer, len);
            return true;
        }
    }
    return false;
}

struct SerialChannelReaderParams {
    HardwareSerial& serial_stream;
    mavlink_channel_t channel;
};

/// @brief Task that reads MAVlink messages from serial port
/// @param parameter 
void SerialChannelReaderTask(void* parameter) {

    // MAVlink supports up to 4 logical channels, each of which can be bound to a different serial port,
    // with the default channel being 0.
    mavlink_channel_t channel = MAVLINK_COMM_0;
    assert(channel >= 0 && channel <= 3); // Ensure channel is valid
    assert(!(systemData.mavlink_channel_flags & (1 << channel))); // Ensure channel is not already in use
    
    uint32_t timeout_update_time = 0; //Used to determine if MAVlink messages are being received
    uint32_t task_delay_time = 25; //Delay time between task executions in milliseconds

    while (true) {
        if (ProcessStreamChannel(Serial, channel)) {
            timeout_update_time = millis();
        }
        
        if (millis() - timeout_update_time >= 10000) {
            timeout_update_time = millis();
            task_delay_time = 250; //Decrease task priority as no messages are being received
            Serial.printf("Waiting for MAVlink messages on channel %d\n", channel);
        }
        vTaskDelay(pdMS_TO_TICKS(task_delay_time));
    }
}

void LoraReceiverTask(void* parameter) {

    xTaskNotify(displayScreenHandle, (uint32_t)LoraStatus::Idle, eSetValueWithOverwrite);
    Serial.println("Starting LoRa succeeded!");

    mavlink_channel_t channel = MAVLINK_COMM_2;
    while (true) {
        int packet_size = LoRa.parsePacket();
        if (packet_size) {
            xTaskNotify(displayScreenHandle, (uint32_t)LoraStatus::Receiving, eSetValueWithOverwrite);
            while (LoRa.available()) {
                ProcessStreamChannel(LoRa, channel);
            }
        }
        else {
            xTaskNotify(displayScreenHandle, (uint32_t)LoraStatus::Idle, eSetValueWithOverwrite);
        }
        vTaskDelay(pdMS_TO_TICKS(50));
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

    screen.init();
    screen.flipScreenVertically(); // Flip screen for debugging
    screen.clear();
    screen.setFont(ArialMT_Plain_10);   
    screen.setTextAlignment(TEXT_ALIGN_CENTER);
    screen.drawString(screen.getWidth() / 2, screen.getHeight() / 2, "Lora Receiver");
    screen.display();
    
    while (true) {
        vTaskDelay(pdMS_TO_TICKS(1000));
    }

}

/// @brief Auxiliary task to measure free stack memory of each task and free heap of the system.
/// Useful to detect possible stack overflows on a task and allocate more stack memory for it if necessary.
/// @param parameter Unused. Just here to comply with the task function signature.
void HighWaterMeasurerTask(void* parameter) {

    while (true) {
        Serial.printf("\n");
        for (int i = 0; i < taskHandlesSize; i++) {
            Serial.printf("Task %s has %d bytes of free stack\n", pcTaskGetTaskName(*taskHandles[i]), uxTaskGetStackHighWaterMark(*taskHandles[i]));
        }
        Serial.printf("Free heap: %d\n", esp_get_free_heap_size());
        Serial.printf("\n");
        vTaskDelay(pdMS_TO_TICKS(10000));
    }
}

void setup() {

    Serial.begin(115200);
    mavlinkQueue = xQueueCreate(10, sizeof(mavlink_message_t));
    xTaskCreate(LedBlinkerTask, "ledBlinker", 2048, NULL, 1, &ledBlinkerHandle);
    xTaskCreate(DisplayScreenTask, "displayScreen", 4096, NULL, 1, &displayScreenHandle);
    xTaskCreate(WifiConnectionTask, "wifiConnection", 4096, NULL, 3, &wifiConnectionHandle);
    xTaskCreate(ServerTask, "server", 4096, NULL, 1, &serverTaskHandle);
    xTaskCreate(SerialChannelReaderTask, "companionReader", 4096, NULL, 1, NULL);
    xTaskCreate(LoraReceiverTask, "loraReceiver", 4096, NULL, 1, &loraReceiverHandle);
    xTaskCreate(HighWaterMeasurerTask, "measurer", 2048, NULL, 1, NULL);  
}

void loop() {

}


