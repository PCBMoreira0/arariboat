#include <Arduino.h> // Main Arduino library, required for projects that use the Arduino framework.
#include <WiFi.h> // Main library for WiFi connectivity, also used by AsyncWebServer.
#include <unordered_map> // Hashtable for storing WiFi credentials.
#include "HTTPClient.h" // HTTP client for sending requests to a listening server.
#include "HttpClientFunctions.hpp" // Auxiliary functions for sending HTTP requests.
#include "Husarnet.h" // IPV6 for ESP32 to enable peer-to-peer communication between devices inside a Husarnet network.
#include "ESPAsyncWebServer.h" // Make sure to include Husarnet before this.
#include "AsyncElegantOTA.h" // Over the air updates for the ESP32.
#include "DallasTemperature.h" // For the DS18B20 temperature probes.
#include "TinyGPSPlus.h" // GPS NMEA sentence parser.
#include "arariboat/mavlink.h" // Custom mavlink dialect for the boat generated using Mavgen tool.
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
        Auxiliary = 0b0010000000,
        Encoder = 0b0100000000,
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
    }
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
TaskHandle_t auxiliaryReaderTaskHandle = nullptr;
TaskHandle_t highWaterMeasurerTaskHandle = nullptr;

// Array of pointers to the task handles. This allows to iterate over the array and perform operations on all tasks, such as resuming, suspending or reading free stack memory.
TaskHandle_t* taskHandles[] = { &ledBlinkerTaskHandle, &wifiConnectionTaskHandle, &serverTaskHandle, &vpnConnectionTaskHandle, &serialReaderTaskHandle, 
                                &temperatureReaderTaskHandle, &gpsReaderTaskHandle, &instrumentationReaderTaskHandle, 
                                &auxiliaryReaderTaskHandle, &highWaterMeasurerTaskHandle};

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
        float current_motor = SystemData::getInstance().instrumentation.current_zero;
        float current_battery = SystemData::getInstance().instrumentation.current_one;
        float current_mppt = SystemData::getInstance().instrumentation.current_two;
        float voltage_battery = SystemData::getInstance().instrumentation.voltage_battery;
        request->send(200, "text/html", "<h1>Boat32</h1><p>Current motor: " + String(current_motor) + "</p><p>Current battery: " + String(current_battery) + "</p><p>Current MPPT: " + String(current_mppt) + "</p><p>Voltage battery: " + String(voltage_battery) + "</p>");
    });
    
    server.on("/gps", HTTP_GET, [](AsyncWebServerRequest *request) {
        // Send GPS data from singleton class
        float latitude = SystemData::getInstance().gps.latitude;
        float longitude = SystemData::getInstance().gps.longitude;
        request->send(200, "text/html", "<h1>Boat32</h1><p>Latitude: " + String(latitude) + "</p><p>Longitude: " + String(longitude) + "</p>");
    });

    server.on("/control-system", HTTP_GET, [](AsyncWebServerRequest *request) {
        // Send control system data from singleton class
        uint8_t pump_mask = SystemData::getInstance().controlSystem.pump_mask;
        float dac_output = SystemData::getInstance().controlSystem.dac_output;
        request->send(200, "text/html", "<h1>Boat32</h1><p>Pump mask: " + String(pump_mask) + "</p><p>DAC output: " + String(dac_output) + "</p>");
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

        case 'C' : {
            // Try to parse float to send current calibration value to auxiliary reader task
            constexpr int16_t scale_factor = 1000; // Scale factor to convert float to fixed point representation
            float calibration_value = 0.0f;
            if (sscanf((const char*)&buffer[1], "%f", &calibration_value)) {
                Serial.printf("\n[SERIAL-CALIBRATION] Value: %f\n", calibration_value);
                calibration_value *= scale_factor;
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
    constexpr uint16_t number_samples_filter = 9; // Number of samples to use in the moving average filter.
    constexpr float pump_threshold_voltage = 10.0f; // Voltage at which the pump is considered to be on.

    pinMode(battery_voltage_pin, INPUT);
    pinMode(port_pump_pin, INPUT);
    pinMode(starboard_pump_pin, INPUT);
    pinMode(battery_current_pin, INPUT);

    float battery_voltage = 0.0f;
    float battery_current = 0.0f;
    bool port_pump_voltage = 0.0f;
    bool starboard_pump_voltage = 0.0f;

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

            asked_to_calibrate = false;
            auto previous_print_state = SystemData::getInstance().debug_print;
            SystemData::getInstance().debug_print = SystemData::getInstance().debug_print_flags::Auxiliary;
            Serial.printf("\n[AUX]Calibrating current sensor\n"
                            "[AUX]Make sure that no current is flowing through the sensor during initialization\n"
                            "[AUX]Press 'C' to continue\n");

            // Suspend other tasks to avoid interference with the current sensor readings.
            vTaskSuspend(instrumentationReaderTaskHandle);
            vTaskSuspend(temperatureReaderTaskHandle);
            vTaskSuspend(gpsReaderTaskHandle);
            xTaskNotify(ledBlinkerTaskHandle, BlinkRate::Fast, eSetValueWithOverwrite);       
            ulTaskNotifyTake(pdTRUE, portMAX_DELAY);
            
            constexpr uint32_t number_samples = 400;
            constexpr uint32_t sample_interval_ms = 10;

            float offset_adc_sum = 0.0f;
            for (uint32_t i = 0; i < number_samples; i++) {
                offset_adc_sum += analogRead(pin);
                vTaskDelay(pdMS_TO_TICKS(sample_interval_ms));
            }
            offset_adc_reference = offset_adc_sum / number_samples;
            Serial.printf("\n[AUX]Offset adc: %.2f\n", offset_adc_reference);
            Serial.printf(  "[AUX]Now turn on the current source and input the current passing through the sensor\n"
                            "[AUX]Write it starting with a 'C' as in 'C2.00'\n ");
            
            uint32_t notification_value;
            while (!xTaskNotifyWait(0, ULONG_MAX, &notification_value, 8000)) {
                Serial.printf("\n[AUX]Please input the current flowing through the sensor starting with a 'C'\n");

            }

            constexpr int16_t scale_factor = 1000;
            float current_sensor = (float)notification_value / scale_factor;
            DEBUG_PRINTF("[AUX]CAL-Current: %.3f\n", current_sensor);

            float measured_adc = 0.0f;
            for (uint32_t i = 0; i < number_samples; i++) {
                measured_adc += analogRead(pin);
                vTaskDelay(pdMS_TO_TICKS(sample_interval_ms));
            }
            
            measured_adc = measured_adc / number_samples;
            sensitivity_adc = current_sensor / (measured_adc - offset_adc_reference);
            Serial.printf("\n[AUX]Offset adc: %.3f\n", offset_adc_reference);
            Serial.printf("[AUX]Measured adc: %.3f\n", measured_adc);
            Serial.printf("[AUX]Sensitivity adc: %.3f\n", sensitivity_adc);
            preferences.putFloat("offset", offset_adc_reference);
            preferences.putFloat("sensitivity", sensitivity_adc);
            preferences.end(); 
            SystemData::getInstance().debug_print = previous_print_state;
            vTaskResume(instrumentationReaderTaskHandle);
            vTaskResume(temperatureReaderTaskHandle);
            vTaskResume(gpsReaderTaskHandle);
            xTaskNotify(ledBlinkerTaskHandle, BlinkRate::Slow, eSetValueWithOverwrite);
        }
    };

    /// @brief Read current using ACS712 current sensor.
    /// @param power_voltage Voltage at the power pin of the ACS712 current sensor.
    /// @param pin Pin connected to the output pin of the ACS712 current sensor.
    /// @param sensitivity Sensitivity of the ACS712 current sensor, which is the rise in current per unit rise in adc value.
    auto ReadBatteryCurrent = [](uint8_t pin, float calibrated_offset_adc, float calibrated_sensitivity) {
       
        // A higher number of samples is used here in order to stabilize the ADC readings before getting the value of the current.
        // As the output current is very sensitive to the ADC readings, it is necessary to stabilize the ADC readings before getting the current
        // instead of filtering the final output directly.

        constexpr uint32_t number_samples_filter = 14;
        static float measured_adc = analogRead(pin);
        measured_adc = (measured_adc * number_samples_filter  + analogRead(pin)) / (number_samples_filter + 1); // Moving average filter.
        float measured_current = (measured_adc - calibrated_offset_adc) * calibrated_sensitivity;
        return measured_current;
    };


    static bool asked_to_calibrate = false;
    constexpr float error_value = -1.0f;
    float offset_adc_reference = error_value;
    float sensitivity_adc = error_value;
    CalibrateCurrentSensor(battery_current_pin, offset_adc_reference, sensitivity_adc, asked_to_calibrate);

    while (true) {
        float battery_voltage_reading = (analogRead(battery_voltage_pin) * adc_reference_voltage) / (adc_resolution * battery_voltage_divider_ratio);
        battery_voltage = (battery_voltage_reading + battery_voltage * number_samples_filter) / (number_samples_filter + 1);

        float battery_current_reading = ReadBatteryCurrent(battery_current_pin, offset_adc_reference, sensitivity_adc);
        //battery_current = (battery_current_reading + battery_current * number_samples_filter) / (number_samples_filter + 1);
        battery_current = battery_current_reading;

        float port_pump_voltage_reading = (analogRead(port_pump_pin) * adc_reference_voltage) / (adc_resolution * battery_voltage_divider_ratio);
        port_pump_voltage = (port_pump_voltage_reading + port_pump_voltage * number_samples_filter) / (number_samples_filter + 1);

        float starboard_pump_voltage_reading = (analogRead(starboard_pump_pin) * adc_reference_voltage) / (adc_resolution * battery_voltage_divider_ratio);
        starboard_pump_voltage = (starboard_pump_voltage_reading + starboard_pump_voltage * number_samples_filter) / (number_samples_filter + 1);

        bool is_port_pump_on = port_pump_voltage_reading > pump_threshold_voltage;
        bool is_starboard_pump_on = starboard_pump_voltage_reading > pump_threshold_voltage;

        SystemData::getInstance().controlSystem.pump_mask = (is_port_pump_on << 1) | is_starboard_pump_on;

        static uint32_t print_timer = 0;
        if (millis() - print_timer > 3000) {
            print_timer = millis();
            if (SystemData::getInstance().debug_print & SystemData::debug_print_flags::Auxiliary) {
                DEBUG_PRINTF("\n[AUX]Battery voltage: %.2fV\n", battery_voltage);
                DEBUG_PRINTF("[AUX]Battery current: %.2fA\n", battery_current);
                DEBUG_PRINTF("[AUX]Port pump: %s\n", is_port_pump_on ? "ON" : "OFF");
                DEBUG_PRINTF("[AUX]Starboard pump: %s\n", is_starboard_pump_on ? "ON" : "OFF");
            }
        }

        if (ulTaskNotifyTake(pdTRUE, pdMS_TO_TICKS(75))) {
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
    //xTaskCreate(InstrumentationReaderTask, "instrumentationReader", 4096, NULL, 3, &instrumentationReaderTaskHandle);
    //xTaskCreate(AuxiliaryReaderTask, "auxiliaryReader", 4096, NULL, 1, &auxiliaryReaderTaskHandle);
    //xTaskCreate(StackHighWaterMeasurerTask, "measurer", 2048, NULL, 1, NULL);  
}

void loop() {
    
}


