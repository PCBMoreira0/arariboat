#include <Arduino.h> // Main Arduino library, required for projects that use the Arduino framework.
#include "DallasTemperature.h" // For the DS18B20 temperature probes.

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
/// @param probes 
void ScanProbeAddresses(DallasTemperature &probes) {
    probes.begin(); // Scan for devices on the OneWire bus.

    if (probes.getDeviceCount() == 0) {
        Serial.printf("\nNo probes found\n");
        return;
    }

    Serial.printf("\nFound %d probes\n", probes.getDeviceCount());
    for (uint8_t i = 0; i < probes.getDeviceCount(); i++) {
        DeviceAddress device_address;
        if (!probes.getAddress(device_address, i)) {
            Serial.printf("Unable to find address for Device %d\n", i);
        } else {
            Serial.printf("Device %d Address: ", i);
            PrintProbeAddress(device_address);
        }
    }
}

void TemperatureReaderTask(void* parameter) {

    constexpr uint8_t pin_temperature_bus = 15; // GPIO used for OneWire communication
    
    OneWire one_wire_device(pin_temperature_bus); // Setup a one_wire_device instance to communicate with any devices that use the OneWire protocol
    DallasTemperature probes(&one_wire_device); // Pass our one_wire_device reference to Dallas Temperature sensor, which uses the OneWire protocol.
    
    // Each probe has a unique 8-byte address. Use the scanIndex method to initially find the addresses of the probes. 
    // Then hardcode the addresses into the program. This is done to avoid the overhead of scanning for the addresses every time the function is called.
    // You should then physically label the probes with tags or stripes as to differentiate them.
    DeviceAddress thermal_probe_zero = {0x28, 0xFF, 0x7C, 0x1C, 0x72, 0x16, 0x05, 0xF7};
    DeviceAddress thermal_probe_one = {0x28, 0x86, 0x1C, 0x07, 0xD6, 0x01, 0x3C, 0x8C};
    DeviceAddress thermal_probe_two = {0x28, 0xFF, 0xA5, 0x12, 0xA0, 0x16, 0x03, 0xC4};

    while (true) {
        ScanProbeAddresses(probes); 
        probes.requestTemperatures();
        float temperature_battery_front = probes.getTempC(thermal_probe_zero);
        if (temperature_battery_front == DEVICE_DISCONNECTED_C) {
            Serial.printf("\n[Temperature][%x%x]Battery: Device disconnected\n", thermal_probe_zero[7]);
        } else {
            Serial.printf("\n[Temperature][%x%x]Battery: %f\n", thermal_probe_zero[6], thermal_probe_zero[7], temperature_battery_front);
        }

        float temperature_battery_rear = probes.getTempC(thermal_probe_one);
        if (temperature_battery_rear == DEVICE_DISCONNECTED_C) {
            Serial.printf("\n[Temperature][%x%x]Battery: Device disconnected\n", thermal_probe_one[7]);
        } else {
            Serial.printf("\n[Temperature][%x%x]Battery: %f\n", thermal_probe_one[6], thermal_probe_one[7], temperature_battery_rear);
        }

        float temperature_mppt = probes.getTempC(thermal_probe_two);
        if (temperature_mppt == DEVICE_DISCONNECTED_C) {
            Serial.printf("\n[Temperature][%x%x]MPPT: Device disconnected\n", thermal_probe_two[7]);
        } else {
            Serial.printf("\n[Temperature][%x%x]MPPT: %f\n", thermal_probe_two[6], thermal_probe_two[7], temperature_mppt);
        }

        #ifdef DEBUG_PRINTF
        if (SystemData::getInstance().debug_print & SystemData::debug_print_flags::Temperature) {
            if (temperature_motor == DEVICE_DISCONNECTED_C) {
                DEBUG_PRINTF("\n[Temperature][%x]Motor: Device disconnected\n", thermal_probe_zero[0]);
            } else {
                DEBUG_PRINTF("\n[Temperature][%x]Motor: %f\n", thermal_probe_zero[0], temperature_motor); // [Temperature][First byte of probe address] = value is the format
            }

            if (temperature_mppt == DEVICE_DISCONNECTED_C) {
                DEBUG_PRINTF("\n[Temperature][%x]MPPT: Device disconnected\n", thermal_probe_one[0]);
            } else {
                DEBUG_PRINTF("\n[Temperature][%x]MPPT: %f\n", thermal_probe_one[0], temperature_mppt);
            }
        }
        #endif

        vTaskDelay(pdMS_TO_TICKS(4000));
    }
}

