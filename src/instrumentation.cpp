#include <Arduino.h>
#include "Adafruit_ADS1X15.h"
#include <Wire.h>
#include "Utilities.hpp"       
#include "data.hpp"
#include "INA226.h"
#include "time_manager.h"
#include "queues.hpp"
#include "propulsion_defs.h"
#include "propulsion.h"
#include "esp_console.h"
#include "argtable3/argtable3.h"

typedef Adafruit_ADS1115 ADS1115; //Shorter alias for the ADS1115 class from Adafruit library

// ADC I2C Addresses
constexpr uint8_t currents_adc_address = 0x48;
constexpr uint8_t solar_panel_currents_adc_address = 0x49;
constexpr uint8_t voltages_adc_address = 0x4A;
constexpr uint8_t propulsion_adc_address = 0x4B;

// INA226 I2C Address for Auxiliary Battery
constexpr uint8_t aux_battery_ina226_address = 0x40;

// EEPROM I2C Addresses (MUST BE UNIQUE AND MATCH HARDWARE)
constexpr uint8_t currents_board_eeprom_address = 0x50;
constexpr uint8_t solar_panel_currents_board_eeprom_address = 0x51;
constexpr uint8_t voltages_board_eeprom_address = 0x52;
constexpr uint8_t propulsion_board_eeprom_address = 0x53;

//Time to post data to the system queue
#ifndef PROPULSION_BOARD
int time_to_post_data_ms = 250; //for instrumentation data posting
#endif
#ifdef PROPULSION_BOARD
int time_to_post_data_ms = 50; //for propulsion data posting
#endif

int instrumentation_debug_print_interval_ms = 500; //Interval between serial prints

// --- Generic Calibration Data Management ---
struct __attribute__((packed)) LinearCalibration {
    float slope;
    float intercept;
};

const uint8_t MAX_CALIBRATION_PAIRS_PER_BOARD = 4;
const uint8_t CALIBRATION_DATA_VERSION = 1;
const uint8_t CALIBRATION_VALID_MARKER = 0xAA;
const uint16_t EEPROM_STRUCT_BASE_ADDR = 0; // Structs stored at the beginning of their respective EEPROMs

struct __attribute__((packed)) BoardCalibrationData {
    uint8_t version;
    uint8_t isValidFlag;
    LinearCalibration calibrations[MAX_CALIBRATION_PAIRS_PER_BOARD];
};

// Global instances for loaded calibrations for each board
BoardCalibrationData currents_board_cal_data;
BoardCalibrationData solar_panel_currents_board_cal_data;
BoardCalibrationData voltages_board_cal_data; 
BoardCalibrationData propulsion_board_cal_data; 

// --- Default Calibration Values ---
// LSB for ADS1115 at GAIN_ONE (±4.096V range / 32767 counts)
const float ADC_LSB_VOLTS_GAIN_ONE = 4.096f / 32767.0f; // Approx. 0.00012500381f V/count

// Default for "Currents" Board (using all 4 pairs)
const BoardCalibrationData DEFAULT_CURRENTS_BOARD_CAL = {
    CALIBRATION_DATA_VERSION, 
    CALIBRATION_VALID_MARKER,
    {
        {0.013063f, -227.935685f},  // Channel 0: Battery Current
        {0.004162f, -54.649315f},   // Channel 1: Motor Left Current
        {0.004176f, -54.880628f},   // Channel 2: Motor Right Current
        {0.004146f, -54.474844f}    // Channel 3: MPPT Current
    }
};

// Default for "Voltages" Board
// Irradiance: (RawADC * LSB_V * 1000 / Sensitivity_V_1000W)
// Sensitivity_V_1000W = 22.4mV / 1000 = 0.0224V
// Slope = (ADC_LSB_VOLTS_GAIN_ONE * 1000.0f) / 0.0224f
const float DEFAULT_IRRADIANCE_SLOPE = (ADC_LSB_VOLTS_GAIN_ONE * 1000.0f) / 0.0224f; // Approx 5.580527f

//Main battery voltage
const float DEFAULT_MAIN_BATTERY_VOLTAGE_SLOPE = 1.0f; // 1:1 slope for main battery voltage (no correction)
const float DEFAULT_MAIN_BATTERY_VOLTAGE_INTERCEPT = 0.0f; // No offset for main battery voltage

// Pump currents (left and right) are assumed to be 1:1 for simplicity
const float DEFAULT_PUMP_LEFT_SLOPE = 1.0f; // 1:1 slope for left pump current
const float DEFAULT_PUMP_LEFT_INTERCEPT = 0.0f; // No offset for left pump current
const float DEFAULT_PUMP_RIGHT_SLOPE = 1.0f; // 1:1 slope for right pump current
const float DEFAULT_PUMP_RIGHT_INTERCEPT = 0.0f; // No offset for right pump current

const BoardCalibrationData DEFAULT_VOLTAGES_BOARD_CAL = {
    CALIBRATION_DATA_VERSION, 
    CALIBRATION_VALID_MARKER,
    {
        {DEFAULT_IRRADIANCE_SLOPE, 0.0f},                                             // Channel 0
        {DEFAULT_PUMP_LEFT_SLOPE, DEFAULT_PUMP_LEFT_INTERCEPT},                       // Channel 1
        {DEFAULT_PUMP_RIGHT_SLOPE, DEFAULT_PUMP_RIGHT_INTERCEPT},                     // Channel 2
        {DEFAULT_MAIN_BATTERY_VOLTAGE_SLOPE, DEFAULT_MAIN_BATTERY_VOLTAGE_INTERCEPT}  // Channel 3
    }
};

constexpr float DEFAULT_NEUTRAL_SLOPE = 1.0f; // 1:1 slope for propulsion board (no correction)
constexpr float DEFAULT_NEUTRAL_INTERCEPT = 0.0f; // No offset for propulsion board

const BoardCalibrationData DEFAULT_PROPULSION_BOARD_CAL = {
    CALIBRATION_DATA_VERSION, 
    CALIBRATION_VALID_MARKER,
    {
        {DEFAULT_NEUTRAL_SLOPE, DEFAULT_NEUTRAL_INTERCEPT}, // Channel 0
        {DEFAULT_NEUTRAL_SLOPE, DEFAULT_NEUTRAL_INTERCEPT}, // Channel 1
        {DEFAULT_NEUTRAL_SLOPE, DEFAULT_NEUTRAL_INTERCEPT}, // Channel 2
        {DEFAULT_NEUTRAL_SLOPE, DEFAULT_NEUTRAL_INTERCEPT}  // Channel 3
    }
};

// Generic EEPROM byte read/write and struct load/save functions (remain unchanged from previous version)
uint8_t read_byte_from_eeprom(uint8_t eeprom_i2c_address, uint16_t byte_offset_in_struct) {
    Wire.beginTransmission(eeprom_i2c_address);
    Wire.write((uint8_t)(EEPROM_STRUCT_BASE_ADDR + byte_offset_in_struct)); 
    Wire.endTransmission(false); // Send restart to keep connection active for read
    Wire.requestFrom(eeprom_i2c_address, (uint8_t)1);
    if (Wire.available()) {
        return Wire.read();
    }
    DEBUG_PRINTF("[ERROR] Failed to read from EEPROM 0x%X at offset %u\n", eeprom_i2c_address, byte_offset_in_struct);
    return 0; // Return 0 on failure
}

bool write_byte_to_eeprom(uint8_t eeprom_i2c_address, uint16_t byte_offset_in_struct, uint8_t value) {
    Wire.beginTransmission(eeprom_i2c_address);
    Wire.write((uint8_t)(EEPROM_STRUCT_BASE_ADDR + byte_offset_in_struct)); // EEPROM memory address
    Wire.write(value); // Data byte
    if (Wire.endTransmission() != 0) { // Returns 0 on success
        DEBUG_PRINTF("[ERROR] Failed to write to EEPROM 0x%X at offset %u value %u\n", eeprom_i2c_address, byte_offset_in_struct, value);
        return false;
    }
    return true;
}

template<typename T>
void load_struct_from_eeprom(T& data_struct, uint8_t eeprom_i2c_address) {
    uint8_t* ptr = reinterpret_cast<uint8_t*>(&data_struct);
    DEBUG_PRINTF("[Calibration] Loading %zu bytes from EEPROM 0x%X...\n", sizeof(T), eeprom_i2c_address);
    for (size_t i = 0; i < sizeof(T); ++i) {
        ptr[i] = read_byte_from_eeprom(eeprom_i2c_address, i);
    }
}

template<typename T>
bool save_struct_to_eeprom(const T& data_struct, uint8_t eeprom_i2c_address) {
    const uint8_t* ptr = reinterpret_cast<const uint8_t*>(&data_struct);
    bool all_success = true;
    DEBUG_PRINTF("[Calibration] Saving %zu bytes to EEPROM 0x%X...\n", sizeof(T), eeprom_i2c_address);
    for (size_t i = 0; i < sizeof(T); ++i) {
        if (!write_byte_to_eeprom(eeprom_i2c_address, i, ptr[i])) {
            all_success = false;
        }
        vTaskDelay(pdMS_TO_TICKS(10)); // Delay for EEPROM write cycle
    }
    if (all_success) {
        DEBUG_PRINTF("[Calibration] Saved struct to EEPROM 0x%X successfully.\n", eeprom_i2c_address);
        return true; // Return true if all bytes were written successfully
    } else {
        DEBUG_PRINTF("[ERROR] Failed to save entire struct to EEPROM 0x%X.\n", eeprom_i2c_address);
        return false; // Return false if any byte write failed
    }
}

// Specific default initializers for the generic BoardCalibrationData struct
void initialize_default_calibrations_for_board(BoardCalibrationData& data_struct, bool is_currents_board) {
    if (is_currents_board) {
        DEBUG_PRINTF("[Calibration] Initializing Currents Board with default generic calibrations.\n");
        data_struct = DEFAULT_CURRENTS_BOARD_CAL;
    } else { // Assuming it's the voltages board
        DEBUG_PRINTF("[Calibration] Initializing Voltages Board with default generic calibrations.\n");
        data_struct = DEFAULT_VOLTAGES_BOARD_CAL;
    }
}
// --- End Calibration Data Management ---


bool i2c_scan() { // ... remains the same ...
    bool device_found = false;
    Serial.println("Scanning I2C bus...");
    for (uint8_t i = 1; i < 127; i++) {
        Wire.beginTransmission(i);
        if (Wire.endTransmission() == 0) {
            Serial.printf("Found device at address: 0x%02X (%d)\n", i, i);
            device_found = true;
        }
    }
    Serial.println("I2C scan complete.");
    return device_found;
}

class LowPassIIR {
public:
    LowPassIIR(float alpha) : _alpha(alpha), _last_output(0.0f) {
        if (alpha < 0.0f || alpha > 1.0f) {
            DEBUG_PRINTF("[ERROR] LowPassIIR: Alpha must be between 0.0 and 1.0, got %.2f\n", alpha);
            _alpha = 0.5f; // Default to a safe value
        }
    }

    //Process a single input sample through the IIR filter and return the filtered output
    float filter(float input) {
        _last_output = _alpha * input + (1.0f - _alpha) * _last_output;
        return _last_output;
    }

    // Get the last output value (useful for debugging or further processing)
    float value() const {
        return _last_output;
    }

    // Reset the filter state
    void reset() {
        _last_output = 0.0f; // Reset last output to zero
    }

private:
    float _alpha;          // Smoothing factor (0.0 to 1.0)
    float _last_output;    // Last output value

};

// Command argument structure
static struct {
    struct arg_str *board; // Board type (currents, voltages, propulsion)
    struct arg_int *channel; // Channel number (0-3)
    struct arg_str *param; // Parameter to set (slope, intercept)
    struct arg_dbl *value; // Value to set for the parameter
    struct arg_end *end;
} set_cal_args;


//Handler function that will be executed when "set_cal" is called
static int handle_set_cal_command(int argc, char **argv) {
    int nerrors = arg_parse(argc, argv, (void**)&set_cal_args);
    if (nerrors != 0) {
        arg_print_errors(stderr, set_cal_args.end, argv[0]);
        return 1; // Return error code
    }

    const char* board_name = set_cal_args.board->sval[0];
    int channel = set_cal_args.channel->ival[0];
    const char* param_name = set_cal_args.param->sval[0];
    float new_value = set_cal_args.value->dval[0];

    printf("Setting calibration for %s board, channel %d, parameter '%s' to %.6f\n",
           board_name, channel, param_name, new_value);

    BoardCalibrationData* target_cal_data = nullptr;
    uint8_t target_eeprom_addr = 0;

    if (strcmp(board_name, "currents") == 0) {
        target_cal_data = &currents_board_cal_data;
        target_eeprom_addr = currents_board_eeprom_address;
    } else if (strcmp(board_name, "voltages") == 0) {
        target_cal_data = &voltages_board_cal_data;
        target_eeprom_addr = voltages_board_eeprom_address;
    } else if (strcmp(board_name, "propulsion") == 0) {
        target_cal_data = &propulsion_board_cal_data;
        target_eeprom_addr = propulsion_board_eeprom_address;
    } else {
        printf("Unknown board type: %s\n", board_name);
        return 1; // Return error code
    }

    if (channel < 0 || channel >= MAX_CALIBRATION_PAIRS_PER_BOARD) {
        printf("Invalid channel number: %d. Must be between 0 and %d.\n", channel, MAX_CALIBRATION_PAIRS_PER_BOARD - 1);
        return 1; // Return error code
    }

    if (strcmp(param_name, "slope") == 0) {
        target_cal_data->calibrations[channel].slope = new_value;
    } else if (strcmp(param_name, "intercept") == 0) {
        target_cal_data->calibrations[channel].intercept = new_value;
    } else {
        printf("Unknown parameter: %s. Use 'slope' or 'intercept'.\n", param_name);
        return 1; // Return error code
    }

    // Save updated calibration data to EEPROM
    if (!save_struct_to_eeprom(*target_cal_data, target_eeprom_addr)) {
        printf("Failed to save calibration data to EEPROM 0x%X\n", target_eeprom_addr);
        return 1; // Return error code
    }
    printf("Calibration updated successfully for %s board, channel %d.\n", board_name, channel);
    printf("New values: Slope=%.6f, Intercept=%.4f\n",
           target_cal_data->calibrations[channel].slope,
           target_cal_data->calibrations[channel].intercept);

    return 0; // Return success code
}

static int handle_get_cal_command(int argc, char **argv) {
    char buffer[512]; // Buffer to hold the entire output string
    int len = 0;      // Current length of the string in the buffer

    len += snprintf(buffer + len, sizeof(buffer) - len, "--- Current Calibration Values (from RAM) ---\n");

    // Print Currents Board Calibrations
    len += snprintf(buffer + len, sizeof(buffer) - len, "Currents Board (EEPROM 0x%X):\n", currents_board_eeprom_address);
    for (int i = 0; i < MAX_CALIBRATION_PAIRS_PER_BOARD; ++i) {
        len += snprintf(buffer + len, sizeof(buffer) - len, "  Ch %d: Slope=%.6f, Intercept=%.4f\n", i,
                        currents_board_cal_data.calibrations[i].slope,
                        currents_board_cal_data.calibrations[i].intercept);
    }

    // Print Voltages Board Calibrations
    len += snprintf(buffer + len, sizeof(buffer) - len, "\nVoltages Board (EEPROM 0x%X):\n", voltages_board_eeprom_address);
    for (int i = 0; i < MAX_CALIBRATION_PAIRS_PER_BOARD; ++i) {
        len += snprintf(buffer + len, sizeof(buffer) - len, "  Ch %d: Slope=%.6f, Intercept=%.4f\n", i,
                        voltages_board_cal_data.calibrations[i].slope,
                        voltages_board_cal_data.calibrations[i].intercept);
    }

    printf("%s", buffer); // Print the entire buffer at once
    return 0;
}

void register_instrumentation_commands() {

    set_cal_args.board = arg_str1(NULL, NULL, "<board>", "Board type (currents, voltages, propulsion)");
    set_cal_args.channel = arg_int1(NULL, NULL, "<channel>", "Channel number (0-3)");
    set_cal_args.param = arg_str1(NULL, NULL, "<param>", "Parameter to set (slope, intercept)");
    set_cal_args.value = arg_dbl1(NULL, NULL, "<value>", "Value to set for the parameter");
    set_cal_args.end = arg_end(2); // Allow for 2 additional arguments (board, channel, param, value) 

    const esp_console_cmd_t set_cal_cmd = {
        .command = "set_cal",
        .help = "Set calibration parameter for a specific board and channel",
        .hint = NULL,
        .func = handle_set_cal_command,
        .argtable = &set_cal_args
    };
    ESP_ERROR_CHECK(esp_console_cmd_register(&set_cal_cmd));

    const esp_console_cmd_t get_cal_cmd = {
        .command = "get_cal",
        .help = "Get calibration status for all boards",
        .hint = NULL,
        .func = handle_get_cal_command,
        .argtable = NULL
    };
    ESP_ERROR_CHECK(esp_console_cmd_register(&get_cal_cmd));
}

void instrumentation_task(void* parameter) {
    
    // Initialize I2C bus
    constexpr gpio_num_t i2c_scl = GPIO_NUM_19;
    constexpr gpio_num_t i2c_sda = GPIO_NUM_21;
    Wire.begin(i2c_sda, i2c_scl);

    // Initialize I2C devices
    ADS1115 currentsAdc;
    ADS1115 solar_panel_currents_adc;
    ADS1115 voltagesAdc;
    ADS1115 propulsionAdc;
    INA226 aux_battery_monitor(aux_battery_ina226_address); // INA226 for auxiliary battery
    
    // Flags for initialization 
    bool is_currents_adc_initialized = false;
    bool is_solar_panel_currents_adc_initialized = false;
    bool is_voltages_adc_initialized = false;
    bool is_propulsion_adc_initialized = false; 

    // Flags for calibration data loading
    bool is_currents_cal_loaded = false;
    bool is_solar_panel_currents_cal_loaded = false;
    bool is_voltages_cal_loaded = false;
    bool is_propulsion_cal_loaded = false;
    bool is_aux_battery_monitor_initialized = false; // Flag for INA226


    float filter_alpha = 0.7f; // Smoothing factor. Percentage of previous data to use for smoothing (0.0 to 1.0)

    // Current board measurements
    LowPassIIR battery_current     (filter_alpha); 
    LowPassIIR current_motor_left  (filter_alpha); 
    LowPassIIR current_motor_right (filter_alpha); 
    LowPassIIR current_mppt        (filter_alpha); 

    // Solar panel currents measurements
    LowPassIIR solar_panel_current_one (filter_alpha); 
    LowPassIIR solar_panel_current_two  (filter_alpha); 
    LowPassIIR solar_panel_current_three  (filter_alpha); 
    LowPassIIR solar_panel_current_four(filter_alpha); 

    // Voltage board measurements
    LowPassIIR irradiance           (filter_alpha);
    LowPassIIR pump_left_voltage    (filter_alpha);
    LowPassIIR pump_right_voltage   (filter_alpha);
    LowPassIIR main_battery_voltage (filter_alpha);

    // Propulsion board measurements
    LowPassIIR backup_potentiometer             (filter_alpha);
    LowPassIIR helm_potentiometer               (filter_alpha);
    LowPassIIR throttle_left_potentiometer      (filter_alpha);
    LowPassIIR throttle_right_potentiometer     (filter_alpha);

    DEBUG_PRINTF("[Instrumentation] Starting main measurement loop.\n");

    while (true) {
        
        vTaskDelay(pdMS_TO_TICKS(50)); // Short delay to allow other tasks to run

        char instrumentation_debug_buffer[768]; // Increased buffer size for more data
        memset(instrumentation_debug_buffer, 0, sizeof(instrumentation_debug_buffer));
        size_t buffer_current_len = 0;

        static unsigned long last_init_check_time = 0;
        constexpr unsigned long init_check_interval = 1000; // Quick fix to prevent excessive heap allocations due to failed begin() calls when board is not present

        #ifndef PROPULSION_BOARD
        // --- CURRENTS ADC & CALIBRATION ---
        // if (!is_currents_adc_initialized && (millis() - last_init_check_time > init_check_interval)) {
        //     if (currentsAdc.begin(currents_adc_address)) {
        //         DEBUG_PRINTF("\n[ADS]Currents ADC (0x%X) successfully initialized.\n", currents_adc_address);
        //         currentsAdc.setDataRate(RATE_ADS1115_16SPS);
        //         currentsAdc.setGain(GAIN_ONE); 
        //         is_currents_adc_initialized = true;
        //     } else { DEBUG_PRINTF("\n[ADS]Currents ADC (0x%X) init failed.\n", currents_adc_address); /* Log failure, non-blocking */ }
        // }

        // if (is_currents_adc_initialized && !is_currents_cal_loaded) {
        //     DEBUG_PRINTF("[Calibration] Loading for Currents Board (EEPROM 0x%X).\n", currents_board_eeprom_address);
        //     load_struct_from_eeprom(currents_board_cal_data, currents_board_eeprom_address);
        //     if (currents_board_cal_data.isValidFlag != CALIBRATION_VALID_MARKER || currents_board_cal_data.version != CALIBRATION_DATA_VERSION) {
        //         DEBUG_PRINTF("[Calibration] Invalid/outdated for Currents Board. Loading defaults & saving to EEPROM 0x%X.\n", currents_board_eeprom_address);
        //         initialize_default_calibrations_for_board(currents_board_cal_data, true); // True for currents board
        //         save_struct_to_eeprom(currents_board_cal_data, currents_board_eeprom_address);
        //     } else { DEBUG_PRINTF("[Calibration] Loaded for Currents Board from EEPROM 0x%X.\n", currents_board_eeprom_address); }
        //     is_currents_cal_loaded = true;
        // }

        // // --- SOLAR PANEL CURRENTS ADC & CALIBRATION ---
        // if (!is_solar_panel_currents_adc_initialized && (millis() - last_init_check_time > init_check_interval)) {
        //     if (solar_panel_currents_adc.begin(solar_panel_currents_adc_address)) {
        //         DEBUG_PRINTF("\n[ADS]Solar Panel Currents ADC (0x%X) successfully initialized.\n", solar_panel_currents_adc_address);
        //         solar_panel_currents_adc.setDataRate(RATE_ADS1115_16SPS);
        //         solar_panel_currents_adc.setGain(GAIN_ONE); // GAIN_ONE for LSB consistency
        //         is_solar_panel_currents_adc_initialized = true;
        //     } else { DEBUG_PRINTF("\n[ADS]Solar Panel Currents ADC (0x%X) init failed.\n", solar_panel_currents_adc_address); /* Log failure, non-blocking */ }
        // }

        // if (is_solar_panel_currents_adc_initialized && !is_solar_panel_currents_cal_loaded) {
        //     DEBUG_PRINTF("[Calibration] Loading for Solar Panel Currents Board (EEPROM 0x%X).\n", solar_panel_currents_board_eeprom_address);
        //     load_struct_from_eeprom(solar_panel_currents_board_cal_data, solar_panel_currents_board_eeprom_address);
        //     if (solar_panel_currents_board_cal_data.isValidFlag != CALIBRATION_VALID_MARKER || solar_panel_currents_board_cal_data.version != CALIBRATION_DATA_VERSION) {
        //         DEBUG_PRINTF("[Calibration] Invalid/outdated for Solar Panel Currents Board. Loading defaults & saving to EEPROM 0x%X.\n", solar_panel_currents_board_eeprom_address);
        //         initialize_default_calibrations_for_board(solar_panel_currents_board_cal_data, true); // True for currents board
        //         save_struct_to_eeprom(solar_panel_currents_board_cal_data, solar_panel_currents_board_eeprom_address);
        //     } else { DEBUG_PRINTF("[Calibration] Loaded for Solar Panel Currents Board from EEPROM 0x%X.\n", solar_panel_currents_board_eeprom_address); }
        //     is_solar_panel_currents_cal_loaded = true;
        // }

        // --- VOLTAGES ADC & CALIBRATION ---
        if (!is_voltages_adc_initialized && (millis() - last_init_check_time > init_check_interval)) {
            if (voltagesAdc.begin(voltages_adc_address)) {
                DEBUG_PRINTF("\n[ADS]Voltages ADC (0x%X) successfully initialized.\n", voltages_adc_address);
                voltagesAdc.setDataRate(RATE_ADS1115_16SPS);
                voltagesAdc.setGain(GAIN_ONE); // GAIN_ONE for LSB consistency with irradiance cal
                is_voltages_adc_initialized = true;
            } else { DEBUG_PRINTF("\n[ADS]Voltages ADC (0x%X) init failed.\n", voltages_adc_address);/* Log failure, non-blocking */ }
        }
        
        if (is_voltages_adc_initialized && !is_voltages_cal_loaded) {
            DEBUG_PRINTF("[Calibration] Loading for Voltages Board (EEPROM 0x%X).\n", voltages_board_eeprom_address);
            load_struct_from_eeprom(voltages_board_cal_data, voltages_board_eeprom_address);
            if (voltages_board_cal_data.isValidFlag != CALIBRATION_VALID_MARKER || voltages_board_cal_data.version != CALIBRATION_DATA_VERSION) {
                DEBUG_PRINTF("[Calibration] Invalid/outdated for Voltages Board. Loading defaults & saving to EEPROM 0x%X.\n", voltages_board_eeprom_address);
                initialize_default_calibrations_for_board(voltages_board_cal_data, false); // False for voltages board
                save_struct_to_eeprom(voltages_board_cal_data, voltages_board_eeprom_address);
            } else { DEBUG_PRINTF("[Calibration] Loaded for Voltages Board from EEPROM 0x%X.\n", voltages_board_eeprom_address); }
            is_voltages_cal_loaded = true;
        }

        // // --- AUXILIARY BATTERY MONITOR (INA226) ---
        // if (!is_aux_battery_monitor_initialized && (millis() - last_init_check_time > init_check_interval)) {
        //     if (aux_battery_monitor.begin()) { // Address was set in constructor
        //         DEBUG_PRINTF("\n[INA226] Aux Battery Monitor (0x%X) successfully initialized.\n", aux_battery_ina226_address);
        //         // Configure INA226 (max current, shunt resistance, normalize LSB)
        //         aux_battery_monitor.setMaxCurrentShunt(13.0f, 0.005f, true); 
        //         is_aux_battery_monitor_initialized = true;
        //     } else {
        //         DEBUG_PRINTF("\n[INA226] Aux Battery Monitor (0x%X) init failed.\n", aux_battery_ina226_address);
        //         /* Log failure, non-blocking, will retry next loop */
        //     }
        // }
        #endif

        #ifdef PROPULSION_BOARD
        // --- PROPULSION ADC ---
        if (!is_propulsion_adc_initialized && (millis() - last_init_check_time > init_check_interval)) {
            if (propulsionAdc.begin(propulsion_adc_address)) {
                DEBUG_PRINTF("\n[ADS]Propulsion ADC (0x%X) successfully initialized.\n", propulsion_adc_address);
                propulsionAdc.setDataRate(RATE_ADS1115_860SPS);
                propulsionAdc.setGain(GAIN_ONE); // GAIN_ONE for LSB consistency
                is_propulsion_adc_initialized = true;
            } else { DEBUG_PRINTF("\n[ADS]Propulsion ADC (0x%X) init failed.\n", propulsion_adc_address); /* Log failure, non-blocking */ }
        }
        #endif

        //Update last initialization check time
        // last_init_check_time = millis();

        #ifndef PROPULSION_BOARD
        // --- Perform measurements and build output string ---
        if (is_currents_adc_initialized && is_currents_cal_loaded) {

            int16_t raw_adc_battery_current = currentsAdc.readADC_SingleEnded(0);
            int16_t raw_adc_motor_left_current = currentsAdc.readADC_SingleEnded(1);
            int16_t raw_adc_motor_right_current = currentsAdc.readADC_SingleEnded(2);
            int16_t raw_adc_mppt_current = currentsAdc.readADC_SingleEnded(3);

            float battery_current_sample     = LinearCorrection(raw_adc_battery_current, currents_board_cal_data.calibrations[0].slope, currents_board_cal_data.calibrations[0].intercept);
            float current_motor_left_sample  = LinearCorrection(raw_adc_motor_left_current, currents_board_cal_data.calibrations[1].slope, currents_board_cal_data.calibrations[1].intercept);
            float current_motor_right_sample = LinearCorrection(raw_adc_motor_right_current, currents_board_cal_data.calibrations[2].slope, currents_board_cal_data.calibrations[2].intercept);
            float current_mppt_sample        = LinearCorrection(raw_adc_mppt_current, currents_board_cal_data.calibrations[3].slope, currents_board_cal_data.calibrations[3].intercept);

            // Apply low-pass IIR filtering to smooth the readings
            battery_current.filter(battery_current_sample);
            current_motor_left.filter(current_motor_left_sample);
            current_motor_right.filter(current_motor_right_sample);
            current_mppt.filter(current_mppt_sample);

            buffer_current_len += snprintf(
                instrumentation_debug_buffer + buffer_current_len,
                sizeof(instrumentation_debug_buffer) - buffer_current_len,
                "%s[Currents ADC 0x%X | EEPROM 0x%X]\n"
                "  Battery Current:     %.2f A (RawADC: %d)\n"
                "  Motor Left Current:  %.2f A (RawADC: %d)\n"
                "  Motor Right Current: %.2f A (RawADC: %d)\n"
                "  MPPT Current:        %.2f A (RawADC: %d)\n",
                (buffer_current_len == 0) ? "" : "\n",
                currents_adc_address, currents_board_eeprom_address,
                battery_current.value(),     raw_adc_battery_current,
                current_motor_left.value(),  raw_adc_motor_left_current,
                current_motor_right.value(), raw_adc_motor_right_current,
                current_mppt.value(),        raw_adc_mppt_current
            );

        } else {
            // buffer_current_len += snprintf(instrumentation_debug_buffer + buffer_current_len,
            //                                sizeof(instrumentation_debug_buffer) - buffer_current_len,
            //                                "%s[Currents Board (ADC 0x%X / EEPROM 0x%X) not ready.]\n",
            //                                (buffer_current_len == 0) ? "" : "\n",
            //                                currents_adc_address, currents_board_eeprom_address);
        }

        // --- Readings from Solar Panel Currents ADC ---
        // One current for each string of the solar panel made up of 4 panels (string 0 up to 4)
        if (is_solar_panel_currents_adc_initialized && is_solar_panel_currents_cal_loaded) {

            int16_t raw_adc_solar_panel_current_one = solar_panel_currents_adc.readADC_SingleEnded(0);
            int16_t raw_adc_solar_panel_current_two = solar_panel_currents_adc.readADC_SingleEnded(1);
            int16_t raw_adc_solar_panel_current_three = solar_panel_currents_adc.readADC_SingleEnded(2);
            int16_t raw_adc_solar_panel_current_four = solar_panel_currents_adc.readADC_SingleEnded(3);

            float solar_panel_current_one_sample = LinearCorrection(raw_adc_solar_panel_current_one, solar_panel_currents_board_cal_data.calibrations[0].slope, solar_panel_currents_board_cal_data.calibrations[0].intercept);
            float solar_panel_current_two_sample = LinearCorrection(raw_adc_solar_panel_current_two, solar_panel_currents_board_cal_data.calibrations[1].slope, solar_panel_currents_board_cal_data.calibrations[1].intercept);
            float solar_panel_current_three_sample = LinearCorrection(raw_adc_solar_panel_current_three, solar_panel_currents_board_cal_data.calibrations[2].slope, solar_panel_currents_board_cal_data.calibrations[2].intercept);
            float solar_panel_current_four_sample = LinearCorrection(raw_adc_solar_panel_current_four, solar_panel_currents_board_cal_data.calibrations[3].slope, solar_panel_currents_board_cal_data.calibrations[3].intercept);

            // Apply low-pass IIR filtering to smooth the readings
            solar_panel_current_one.filter(solar_panel_current_one_sample);
            solar_panel_current_two.filter(solar_panel_current_two_sample);
            solar_panel_current_three.filter(solar_panel_current_three_sample);
            solar_panel_current_four.filter(solar_panel_current_four_sample);

            buffer_current_len += snprintf(
                instrumentation_debug_buffer + buffer_current_len,
                sizeof(instrumentation_debug_buffer) - buffer_current_len,
                "%s[Solar Panel Currents ADC 0x%X | EEPROM 0x%X]\n"
                "  String 1 Current: %.2f A (RawADC: %d)\n"
                "  String 2 Current: %.2f A (RawADC: %d)\n"
                "  String 3 Current: %.2f A (RawADC: %d)\n"
                "  String 4 Current: %.2f A (RawADC: %d)\n",
                (buffer_current_len == 0) ? "" : "\n",
                solar_panel_currents_adc_address, solar_panel_currents_board_eeprom_address,
                solar_panel_current_one.value(), raw_adc_solar_panel_current_one,
                solar_panel_current_two.value(), raw_adc_solar_panel_current_two,
                solar_panel_current_three.value(), raw_adc_solar_panel_current_three,
                solar_panel_current_four.value(), raw_adc_solar_panel_current_four
            );

        } else {
            // buffer_current_len += snprintf(instrumentation_debug_buffer + buffer_current_len,
            //                                sizeof(instrumentation_debug_buffer) - buffer_current_len,
            //                                "%s[Solar Panel Currents Board (ADC 0x%X / EEPROM 0x%X) not ready.]\n",
            //                                (buffer_current_len == 0) ? "" : "\n",
            //                                solar_panel_currents_adc_address, solar_panel_currents_board_eeprom_address);
        }


        if (is_voltages_adc_initialized && is_voltages_cal_loaded) {

            int16_t raw_adc_irradiance = voltagesAdc.readADC_SingleEnded(0);
            int16_t raw_adc_pump_left_voltage = voltagesAdc.readADC_SingleEnded(1);
            int16_t raw_adc_pump_right_voltage = voltagesAdc.readADC_SingleEnded(2);
            int16_t raw_adc_main_battery_voltage = voltagesAdc.readADC_SingleEnded(3);

            // Convert raw ADC readings to calibrated values using the loaded calibration data
            float irradiance_sample = LinearCorrection(raw_adc_irradiance, voltages_board_cal_data.calibrations[0].slope, voltages_board_cal_data.calibrations[0].intercept);
            float pump_left_voltage_sample = LinearCorrection(raw_adc_pump_left_voltage, voltages_board_cal_data.calibrations[1].slope, voltages_board_cal_data.calibrations[1].intercept);              
            float pump_right_voltage_sample = LinearCorrection(raw_adc_pump_right_voltage, voltages_board_cal_data.calibrations[2].slope, voltages_board_cal_data.calibrations[2].intercept);                                                                                              
            float main_battery_voltage_sample = LinearCorrection(raw_adc_main_battery_voltage, voltages_board_cal_data.calibrations[3].slope, voltages_board_cal_data.calibrations[3].intercept);
                                                                                  
            // Apply low-pass IIR filtering to smooth the irradiance reading
            irradiance.filter(irradiance_sample);
            pump_left_voltage.filter(pump_left_voltage_sample);
            pump_right_voltage.filter(pump_right_voltage_sample);
            main_battery_voltage.filter(main_battery_voltage_sample);
            
            buffer_current_len += snprintf(
                instrumentation_debug_buffer + buffer_current_len,
                sizeof(instrumentation_debug_buffer) - buffer_current_len,
                "%s[Voltages ADC 0x%X | EEPROM 0x%X]\n"
                "  Irradiance: %.0f W/m^2 (RawADC: %d)\n"
                "  Pump Left Voltage: %.2f V (RawADC: %d)\n"
                "  Pump Right Voltage: %.2f V (RawADC: %d)\n"
                "  Main Battery Voltage: %.2f V (RawADC: %d)\n",
                (buffer_current_len == 0) ? "" : "\n",
                voltages_adc_address, voltages_board_eeprom_address,
                irradiance.value(), raw_adc_irradiance,
                pump_left_voltage.value(), raw_adc_pump_left_voltage,
                pump_right_voltage.value(), raw_adc_pump_right_voltage,
                main_battery_voltage.value(), raw_adc_main_battery_voltage
            );


        } else {
            // buffer_current_len += snprintf(instrumentation_debug_buffer + buffer_current_len,
            //                                sizeof(instrumentation_debug_buffer) - buffer_current_len,
            //                                "%s[Voltages Board (ADC 0x%X / EEPROM 0x%X) not ready.]\n",
            //                                (buffer_current_len == 0) ? "" : "\n",
            //                                voltages_adc_address, voltages_board_eeprom_address);
        }
        
        // --- Readings from Auxiliary Battery Monitor (INA226) ---
        if (is_aux_battery_monitor_initialized) {
            float aux_bus_voltage = aux_battery_monitor.getBusVoltage();
            // Apply the specific calibration for current: * 0.786f + 8.48E-3f
            float aux_current = aux_battery_monitor.getCurrent() * 0.786f + 0.00848f; 
            float aux_power = aux_battery_monitor.getPower();
            // float aux_shunt_voltage_mv = aux_battery_monitor.getShuntVoltage_mV(); // Optional for debugging

            buffer_current_len += snprintf(instrumentation_debug_buffer + buffer_current_len,
                                           sizeof(instrumentation_debug_buffer) - buffer_current_len,
                                           "%s[Aux Battery INA226 0x%X]\n"
                                           "  Aux V: %.2fV, Aux I: %.3fA, Aux P: %.2fW\n",
                                           (buffer_current_len == 0) ? "" : "\n",
                                           aux_battery_ina226_address,
                                           aux_bus_voltage, aux_current, aux_power);
        } else {
            // buffer_current_len += snprintf(instrumentation_debug_buffer + buffer_current_len,
            //                                sizeof(instrumentation_debug_buffer) - buffer_current_len,
            //                                "%s[Aux Battery INA226 (0x%X) not ready.]\n",
            //                                (buffer_current_len == 0) ? "" : "\n",
            //                                aux_battery_ina226_address);
        }
        #endif

        #ifdef PROPULSION_BOARD
        // --- PROPULSION ADC ---
        if (is_propulsion_adc_initialized) {
            // Read propulsion ADC channels (assuming 4 channels for motors)
            int16_t backup_potentiometer_adc = propulsionAdc.readADC_SingleEnded(0);
            int16_t helm_potentiometer_adc = propulsionAdc.readADC_SingleEnded(1);
            int16_t throttle_left_potentiometer_adc = propulsionAdc.readADC_SingleEnded(2);
            int16_t throttle_right_potentiometer_adc = propulsionAdc.readADC_SingleEnded(3);

            float backup_potentiometer_volts = LinearCorrection(backup_potentiometer_adc, 
                0.00019, 
                0.0398f); //!FIX ME LATER USING FLASH MEMORY FOR CALIBRATION

            //Get values directly in Volts without calibration since we assume 1:1 slope and intercept
            float helm_potentiometer_volts = propulsionAdc.computeVolts(helm_potentiometer_adc);
            float throttle_left_potentiometer_volts = propulsionAdc.computeVolts(throttle_left_potentiometer_adc);
            float throttle_right_potentiometer_volts = propulsionAdc.computeVolts(throttle_right_potentiometer_adc);

            // Apply low-pass IIR filtering to smooth the readings
            backup_potentiometer.filter(backup_potentiometer_volts);
            helm_potentiometer.filter(helm_potentiometer_volts);
            throttle_left_potentiometer.filter(throttle_left_potentiometer_volts);
            throttle_right_potentiometer.filter(throttle_right_potentiometer_volts);
           
            // Build the output string with propulsion ADC data
            buffer_current_len += snprintf(
                instrumentation_debug_buffer + buffer_current_len,
                sizeof(instrumentation_debug_buffer) - buffer_current_len,
                "%s[Propulsion ADC 0x%X]\n"
                "  Backup Potentiometer: %.2f V (RawADC: %d)\n"
                "  Helm Potentiometer: %.2f V (RawADC: %d)\n"
                "  Throttle Left Potentiometer: %.2f V (RawADC: %d)\n"
                "  Throttle Right Potentiometer: %.2f V (RawADC: %d)\n",
                (buffer_current_len == 0) ? "" : "\n",
                propulsion_adc_address,
                backup_potentiometer.value(), backup_potentiometer_adc,
                helm_potentiometer.value(), helm_potentiometer_adc,
                throttle_left_potentiometer.value(), throttle_left_potentiometer_adc,
                throttle_right_potentiometer.value(), throttle_right_potentiometer_adc
            );

        } else {
            buffer_current_len += snprintf(instrumentation_debug_buffer + buffer_current_len,
                                           sizeof(instrumentation_debug_buffer) - buffer_current_len,
                                           "%s[Propulsion Board ADC (0x%X) not ready.]\n",
                                           (buffer_current_len == 0) ? "" : "\n",
                                           propulsion_adc_address);
        }
        #endif // PROPULSION_BOARD

        // --- Print the accumulated debug information ---
        static unsigned long last_print_time = 0;
        if (millis() - last_print_time > instrumentation_debug_print_interval_ms) {
            last_print_time = millis(); // Update last print time
            if (buffer_current_len > 0) {
                if (!EndsWithNewline(instrumentation_debug_buffer)) {
                    instrumentation_debug_buffer[buffer_current_len++] = '\n'; // Ensure it ends with a newline
                    instrumentation_debug_buffer[buffer_current_len] = '\0'; // Null-terminate the string
                }
                // Serial.print(instrumentation_debug_buffer); // Print all accumulated data at once
            } else {
                Serial.println("[Instrumentation] No data to report this cycle.");
            }
        }

        static unsigned long last_post_time_ms = 0;
        if (millis() - last_post_time_ms < time_to_post_data_ms) {
            continue; // Skip posting if not enough time has passed
        }
        last_post_time_ms = millis(); // Update last post time

        //Pass data to queues
        unsigned long time_boot_ms = millis();

        #ifndef PROPULSION_BOARD
        message_t msg;
        msg.source = DATA_SOURCE_INSTRUMENTATION;
        auto& data = msg.payload.instrumentation;
        data.battery_current_cA = static_cast<int16_t>(battery_current.value() * 100.0f); // Convert to centiAmperes
        data.motor_current_left_cA = static_cast<int16_t>(current_motor_left.value() * 100.0f); // Convert to centiAmperes
        data.motor_current_right_cA = static_cast<int16_t>(current_motor_right.value() * 100.0f); // Convert to centiAmperes
        data.mppt_current_cA = static_cast<int16_t>(current_mppt.value() * 100.0f); // Convert to centiAmperes
        data.battery_voltage_cV = static_cast<uint16_t>(main_battery_voltage.value() * 100.0f); // Convert to centiVolts
        data.auxiliary_battery_voltage_cV = static_cast<uint16_t>(aux_battery_monitor.getBusVoltage() * 100.0f); // Convert to centiVolts
        data.auxiliary_battery_current_cA = static_cast<int16_t>(aux_battery_monitor.getCurrent() * 100.0f); // Convert to centiAmperes
        data.irradiance = static_cast<uint16_t>(irradiance.value()); // Convert to W/m^2
        data.timestamp_ms = time_boot_ms; // Timestamp in milliseconds

        msg.timestamp.epoch_seconds = get_epoch_seconds();
        msg.timestamp.epoch_ms = get_epoch_millis();
        msg.timestamp.time_since_boot_ms = time_boot_ms;

        //Send the message to the broker
        if (xQueueSend(broker_queue, &msg, pdMS_TO_TICKS(20)) != pdTRUE) {
            DEBUG_PRINTF("[INSTRUMENTATION]Error: queue is full\n");
        } 
        #endif 

        // Propulsion data message
        #ifdef PROPULSION_BOARD
        message_t propulsion_msg;
        propulsion_msg.source = DATA_SOURCE_PROPULSION;
        auto& propulsion_data = propulsion_msg.payload.propulsion;
        propulsion_data.backup_potentiometer_volts = backup_potentiometer.value() * 1000.0f;
        propulsion_data.helm_potentiometer_volts = helm_potentiometer.value() * 1000.0f;
        propulsion_data.throttle_left_potentiometer_volts = throttle_left_potentiometer.value() * 1000.0f;
        propulsion_data.throttle_right_potentiometer_volts = throttle_right_potentiometer.value() * 1000.0f;
        propulsion_data.state = propulsion_get_state();
        propulsion_msg.timestamp.epoch_ms = get_epoch_seconds();
        propulsion_msg.timestamp.epoch_ms = get_epoch_millis();
        propulsion_msg.timestamp.time_since_boot_ms = time_boot_ms;
        // Send the propulsion message to the broker
        if (xQueueSend(broker_queue, &propulsion_msg, pdMS_TO_TICKS(20)) != pdTRUE) {
            DEBUG_PRINTF("[PROPULSION]Error: queue is full\n");
        }
        #endif // PROPULSION_BOARD
    }
}