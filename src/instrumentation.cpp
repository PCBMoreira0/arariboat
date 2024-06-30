#include <Arduino.h> // Main Arduino library, required for projects that use the Arduino framework.
#include "Adafruit_ADS1X15.h" // 16-bit high-linearity with programmable gain amplifier Analog-Digital Converter for measuring current and voltage.
#include <SPI.h> // Required for the ADS1115 ADC.
#include <Wire.h> // Required for the ADS1115 ADC and communication with the LoRa board.
#include "Utilities.hpp" // Custom utility macros and functions.
#include "arariboat/mavlink.h" // Custom mavlink dialect for the boat generated using Mavgen tool.

typedef Adafruit_ADS1115 ADS1115; // Alias for the ADS1115 class.

/// @brief Calibrates a reading by using a linear equation obtained by comparing the readings with a multimeter.
/// @param input 
/// @param slope 
/// @param intercept 
/// @return Calibrated reading
float LinearCorrection(const float input_value, const float slope, const float intercept) {
    return slope * input_value + intercept;
}

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
    
    ADS1115 adc; 
    constexpr uint8_t adc_addresses[] = {0x48, 0x49}; // Address is determined by a solder bridge on the instrumentation board.
    adc.setGain(GAIN_FOUR); // Configuring the PGA( Programmable Gain Amplifier) to amplify the signal by 4 times, so that the maximum input voltage is +/- 1.024V
    adc.setDataRate(RATE_ADS1115_16SPS); // Setting a low data rate to increase the oversampling ratio of the ADC and thus reduce the noise.
    
    bool is_adc_initialized = false;
    
    while (!is_adc_initialized) {
        for (auto address : adc_addresses) {
            Serial.printf("\n[ADS]Trying to initialize ADS1115 at address 0x%x\n", address);
            if (adc.begin(address)) {
                Serial.printf("\n[ADS]ADS1115 successfully initialized at address 0x%x\n", address);
                is_adc_initialized = true;
                break;
            }
            vTaskDelay(pdMS_TO_TICKS(1000));
        }
    }
    
    while (true) {
        // In the ADS1115 single ended measurements have 15 bits of resolution. Only differential measurements have 16 bits of resolution.
        // As we are using the 4 analog inputs for each of the 4 sensors, single ended measurements are being used in order to access all 4 sensors.
        // When using single ended mode, the maximum output code is 0x7FFF(32767), which corresponds to the full-scale input voltage.
        
        
        adc.setGain(GAIN_FOUR);
        float voltage_battery = LinearCorrection(adc.readADC_SingleEnded(0), 0.002472f, 0.801442);

        adc.setGain(GAIN_EIGHT);
        float current_port = LinearCorrection(adc.readADC_SingleEnded(1), 0.005653f, -56.366843f);

        adc.setGain(GAIN_EIGHT);
        float current_starboard = LinearCorrection(adc.readADC_SingleEnded(2), 0.005627f, -56.204637f);

        adc.setGain(GAIN_EIGHT);
        float current_mppt = LinearCorrection(adc.readADC_SingleEnded(3), 0.001602f, 0.015848f);

        Serial.printf("\n[Instrumentation]Battery voltage: %.2fV\n"
                      "[Instrumentation]Port current: %.2fA\n"
                      "[Instrumentation]Starboard current: %.2fA\n"
                      "[Instrumentation]MPPT current: %.2fA\n",
                      voltage_battery, current_port, current_starboard, current_mppt);
        
        /*
        if (SystemData::getInstance().debug_print & SystemData::debug_print_flags::Instrumentation) {
            DEBUG_PRINTF( "\n[Instrumentation]Primary resistor voltage drop: %fV\n"
                            "[Instrumentation]Battery: %fV\n"
                            "[Instrumentation]Calibrated battery: %fV\n"
                            "[Instrumentation]Motor current: %fV\n"
                            "[Instrumentation]Battery current: %fV\n"
                            "[Instrumentation]MPPT current: %fV\n",
            voltage_primary_resistor_drop, voltage_battery, calibrated_voltage_battery, current_motor, current_battery, current_mppt);
        }
        

        SystemData::getInstance().instrumentation.voltage_battery = calibrated_voltage_battery;
        SystemData::getInstance().instrumentation.current_zero = current_motor;
        SystemData::getInstance().instrumentation.current_one = current_battery;
        SystemData::getInstance().instrumentation.current_two = current_mppt;

        
        */

        // Prepare and send Mavlink message
        mavlink_message_t message;
        mavlink_instrumentation_t instrumentation = {
            .battery_voltage = voltage_battery,
            .motor_current = current_starboard,
            .battery_current = current_port,
            .mppt_current = current_mppt,
        };
        
        mavlink_msg_instrumentation_encode_chan(1, MAV_COMP_ID_ONBOARD_COMPUTER, MAVLINK_COMM_0, &message, &instrumentation);
        uint8_t buffer[MAVLINK_MAX_PACKET_LEN];
        uint16_t len = mavlink_msg_to_send_buffer(buffer, &message);
        Serial.write(buffer, len);
        vTaskDelay(pdMS_TO_TICKS(4000));
    }
}


