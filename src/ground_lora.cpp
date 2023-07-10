#include <Arduino.h> // Main Arduino library, required for projects that use the Arduino framework.
#include <WiFi.h> // Main library for WiFi connectivity, also used by AsyncWebServer.
#include <unordered_map> // Hashtable for storing WiFi credentials.
#include <AsyncTCP.h> // Asynchronous TCP library for the ESP32.
#include <ESPAsyncWebServer.h> // Asynchronous web server for the ESP32.
#include <AsyncElegantOTA.h> // Over the air updates for the ESP32.
#include "arariboat\mavlink.h" // Custom mavlink dialect for the boat generated using Mavgen tool.
#include <Wire.h> // I2C library for communicating with LoRa32 board.
#include <LoRa.h> // SandeepMistry physical layer library
#include "BoardDefinitions.h" // SX1276, SDCard and OLED display pin definitions
#include <WebSerial.h>
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
    Pulse = 1000 // Pulse is a special value that will cause the LED to blink fast and then return to the previous blink rate.
};

enum LoraStatus : uint32_t {
    Failed,
    Idle,
    Sending,
    Receiving
};

// Tasks can send notifications here to change the blink rate of the LED in order to communicate the status of the boat.
void FastBlinkPulse(int pin);
void LedBlinkerTask(void* parameter) {

    constexpr int ledPin = 25; // Pin 25 is the onboard LED on the TGGO LoRa V2.1.6 board.
    constexpr uint8_t buzzer_pin = 4;
    pinMode(ledPin, OUTPUT);
    pinMode(buzzer_pin, OUTPUT);

    uint32_t blinkRate = BlinkRate::Slow;
    uint32_t previousBlinkRate = blinkRate;

    // Lambda function to blink the LED fast 4 times and then return to the previous blink rate.
    auto FastBlinkPulse = [](int pin, int pulses = 4) {
        for (int i = 0; i < 4; i++) {
            digitalWrite(pin, HIGH); vTaskDelay(pdMS_TO_TICKS(50));
            digitalWrite(pin, LOW);  vTaskDelay(pdMS_TO_TICKS(50));
        }
    };

    // Lambda function to beep the buzzer using PWM
    auto Beep = [](int pin, int channel, int pulses, int frequency = 1000, int duration = 100) {
        // Set up the LEDC module with channel 0, 1000 Hz frequency, and 8-bit resolution
        ledcSetup(channel, 1000, 8);
        // Attach the LEDC module to the specified pin using channel 0
        ledcAttachPin(pin, channel);
        for (int i = 0; i < pulses; i++) {
            // Set the tone of the beep using the specified frequency and channel 0
            ledcWriteTone(channel, frequency);
            // Delay for the specified duration using the vTaskDelay function
            vTaskDelay(pdMS_TO_TICKS(duration));
            // Turn off the beep by setting the tone to 0 using channel 0
            ledcWriteTone(channel, 0);
            vTaskDelay(pdMS_TO_TICKS(duration));
        }
    };
    
    while (true) {
        digitalWrite(ledPin, HIGH); vTaskDelay(pdMS_TO_TICKS(blinkRate));
        digitalWrite(ledPin, LOW);  vTaskDelay(pdMS_TO_TICKS(blinkRate));
        
        // Set blink rate to the value received from the notification
        if (xTaskNotifyWait(0, 0, (uint32_t*)&blinkRate, 0)) {
            if (blinkRate == BlinkRate::Pulse) {
                blinkRate = previousBlinkRate;
                FastBlinkPulse(ledPin, 4);
                Beep(buzzer_pin, 0, 2, 1000, 100);
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
                    Serial.printf("\n[WIFI]Connected to %s\nIP: %s\n", wifi.first, WiFi.localIP().toString().c_str());
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
        request->send(200, "text/html", "<h1>Ground-Lora</h1><p>WiFi connected: " + WiFi.SSID() + "</p><p>IP address: " + WiFi.localIP().toString() + "</p>");
    });

    server.on("/reset", HTTP_GET, [](AsyncWebServerRequest *request) {
        // log reset message
        request->send(200, "text/html", "<h1>Boat32</h1><p>Resetting...</p>");
        vTaskDelay(pdMS_TO_TICKS(1000));
        ESP.restart();
    });

    // Wait for notification from WifiConnection task that WiFi is connected in order to begin the server
    ulTaskNotifyTake(pdTRUE, portMAX_DELAY);

    auto recvMsg = [&](uint8_t *data, size_t len){
        WebSerial.println("Received Data...");
        String d = "";
        for(int i=0; i < len; i++){
            d += char(data[i]);
        }
        WebSerial.println(d);
    };

    WebSerial.begin(&server);
    /* Attach Message Callback */
    WebSerial.msgCallback(recvMsg);  
    // Attach OTA update handler to the server and initialize the server.

    // Allow the server to be accessed by hostname instead of IP address.
    if(!MDNS.begin("ground-lora")) {
        Serial.println("[MDNS]Error starting mDNS!");
    }

    AsyncElegantOTA.begin(&server); // Available at http://[esp32ip]/update or http://[esp32hostname]/update
    server.begin();

    while (true) {
        WebSerial.printf("Millis=%lu\n", millis());
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
                    DEBUG_PRINTF("[RX]Received heartbeat from channel %d\n", channel);
                    break;
                }
                case MAVLINK_MSG_ID_INSTRUMENTATION: {
                    DEBUG_PRINTF("[RX]Received instrumentation from channel %d\n", channel);
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
                    DEBUG_PRINTF("[RX]Received message with ID #%d from channel %d\n", message.msgid, channel);
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
    
    uint32_t timeout_update_time = 0; //Used to determine if MAVlink messages are being received
    uint32_t task_delay_time = 25; //Delay time between task executions in milliseconds

    while (true) {
        if (ProcessStreamChannel(Serial, channel)) {
            timeout_update_time = millis();
        }
        
        if (millis() - timeout_update_time >= 10000) {
            timeout_update_time = millis();
            task_delay_time = 250; //Decrease task priority as no messages are being received
            DEBUG_PRINTF("\n[CHANNEL]Waiting for MAVlink messages on channel %d\n", channel);
        }
        vTaskDelay(pdMS_TO_TICKS(task_delay_time));
    }
}

void StartLora() {
    LoRa.setPins(CONFIG_NSS, CONFIG_RST, CONFIG_DIO0); // Use ESP32 pins instead of default Arduino pins set by LoRa constructor
    LoRa.setSyncWord(SYNC_WORD);
    LoRa.setCodingRate4(5);
    LoRa.setSignalBandwidth(500E3);
    LoRa.setSpreadingFactor(7); // Receiver and transmitter must have the same spreading factor. 
    while (!LoRa.begin(BAND)) { // Attention: initializes default SPI bus at pins 5, 18, 19, 27
        Serial.println("Starting LoRa failed!");
        vTaskDelay(pdMS_TO_TICKS(3500));
    }

    //xTaskNotify(displayScreenHandle, (uint32_t)LoraStatus::Idle, eSetValueWithOverwrite);    
    Serial.println("Starting LoRa succeeded!");
}

void LoraReceiverTask(void* parameter) {

    LoRa.setPins(CONFIG_NSS, CONFIG_RST, CONFIG_DIO0); // Use ESP32 pins instead of default Arduino pins set by LoRa constructor
    LoRa.setSyncWord(SYNC_WORD);
    LoRa.setCodingRate4(5);
    LoRa.setSignalBandwidth(500E3);
    LoRa.setSpreadingFactor(7); // Receiver and transmitter must have the same spreading factor. 
    while (!LoRa.begin(BAND)) { // Attention: initializes default SPI bus at pins 5, 18, 19, 27
        Serial.println("Starting LoRa failed!");
        vTaskDelay(pdMS_TO_TICKS(3500));
    }

    //xTaskNotify(displayScreenHandle, (uint32_t)LoraStatus::Idle, eSetValueWithOverwrite);    
    Serial.println("Starting LoRa succeeded!");

    constexpr mavlink_channel_t channel = MAVLINK_COMM_2;
    while (true) {

        static uint32_t last_reception_time = 0;
        if (millis() - last_reception_time >= 5000) {
            last_reception_time = millis();
            DEBUG_PRINTF("\n[CHANNEL]Waiting for LoRa messages on channel %d\n", channel);
        }
   
        int packet_size = LoRa.parsePacket();
        if (packet_size) {
            xTaskNotify(displayScreenHandle, (uint32_t)LoraStatus::Receiving, eSetValueWithOverwrite);
            last_reception_time = millis();
            while (LoRa.available()) {
                if (ProcessStreamChannel(LoRa, channel)) {
                    xTaskNotify(ledBlinkerHandle, BlinkRate::Pulse, eSetValueWithOverwrite);
                }
                vTaskDelay(pdMS_TO_TICKS(5));
            }
        }
        else {
            xTaskNotify(displayScreenHandle, (uint32_t)LoraStatus::Idle, eSetValueWithOverwrite);
        }
        vTaskDelay(pdMS_TO_TICKS(25));   
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
        screen.drawString(screen.getWidth() / 2, (screen.getHeight() - vertical_offset) / 2, "Lora Receiver");
        screen.display();
    };

    auto ShowWifiScreen = [&]() {

        auto ssid = WiFi.SSID();
        String ip = WiFi.localIP().toString();
        //RSSI Value Range	WiFi Signal Strength
        //RSSI > -30 dBm	 Amazing
        //RSSI < – 55 dBm	 Very good signal
        //RSSI < – 67 dBm	 Fairly Good
        //RSSI < – 70 dBm	 Okay
        //RSSI < – 80 dBm	 Not good
        //RSSI < – 90 dBm	 Extremely weak signal (unusable)
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

    auto ShowLoraScreen = [&]() {
        screen.clear();
        screen.setFont(ArialMT_Plain_10);
        screen.setTextAlignment(TEXT_ALIGN_LEFT);
        // Display LoRa status
        screen.drawString((screen.getWidth() - 40) / 2, 0, "[LoRa]");
        screen.drawString(0, 15, "Message ID: " + String(mavlink_get_channel_status(MAVLINK_COMM_2)->packet_idx));
        screen.drawString(0, 25, "RSSI: " + String(LoRa.packetRssi()));
        screen.drawString(0, 35, "Sequence: " + String(mavlink_get_channel_status(MAVLINK_COMM_2)->current_rx_seq));
        screen.display();
    };
    
    screen.init();
    screen.flipScreenVertically(); // Rotate screen to get correct orientation
    constexpr uint16_t update_rate = 500;
    static uint32_t last_update_time = 0;
    int16_t interval;
    while (true) {
        for (int i = 0; i < NumberPages; i++) {
            switch (i) {
                case Home:
                    interval = 2000;
                    break;
                case Wifi:
                    interval = 2000;
                    break;
                case Lora:
                    interval = 45000;
                    break;
            }
            
            while (millis() - last_update_time < interval) {
                switch (i) {
                    case Home:
                        ShowHomeScreen();
                        break;
                    case Wifi:
                        ShowWifiScreen();
                        break;
                    case Lora:
                        ShowLoraScreen();
                        break;
                }
                vTaskDelay(pdMS_TO_TICKS(update_rate));
            }
            last_update_time = millis();
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
            Serial.printf("[Task]%s has %d bytes of free stack\n", pcTaskGetTaskName(*taskHandles[i]), uxTaskGetStackHighWaterMark(*taskHandles[i]));
        }
        Serial.printf("[Task]System free heap: %d\n", esp_get_free_heap_size());
        vTaskDelay(pdMS_TO_TICKS(25000));
    }
}

void setup() {

    Serial.begin(4800);
    xTaskCreate(LedBlinkerTask, "ledBlinker", 2048, NULL, 1, &ledBlinkerHandle);
    xTaskCreate(DisplayScreenTask, "displayScreen", 4096, NULL, 1, &displayScreenHandle);
    xTaskCreate(WifiConnectionTask, "wifiConnection", 4096, NULL, 1, &wifiConnectionHandle);
    xTaskCreate(ServerTask, "server", 4096, NULL, 1, &serverTaskHandle);
    xTaskCreate(SerialChannelReaderTask, "companionReader", 4096, NULL, 1, NULL);
    xTaskCreate(LoraReceiverTask, "loraReceiver", 4096, NULL, 3, &loraReceiverHandle);
    xTaskCreate(StackHighWaterMeasurerTask, "measurer", 2048, NULL, 1, NULL);  
}

void loop() {

}

