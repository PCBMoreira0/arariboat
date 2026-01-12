#include <stdbool.h>
#include "data.hpp"
#include "arariboat/mavlink.h" // Custom mavlink dialect for the boat generated using Mavgen tool.
#include <stdbool.h>
#include "mavlink_data_conversion.h"

#define PRINT_RECEIVED_INFO


/**
 * @brief Prints the contents of a mavlink_bms_t message, applying scaling.
 * @param msg A pointer to the decoded mavlink_bms_t message.
 */
void print_bms_message(const mavlink_bms_t* msg) {
    Serial.println("--- MAVLink BMS Message ---");
    Serial.printf("  SoC: %d %%\n", msg->state_of_charge);
    Serial.printf("  Current: %.1f A\n", (float)msg->current_battery / 10.0f);
    Serial.printf("  Temperatures: %d C, %d C\n", msg->temperatures[0], msg->temperatures[1]);
    uint32_t total_voltage_mv = 0;
    for (int i = 0; i < 16; i++) {
        total_voltage_mv += msg->voltages[i];
    }
    Serial.printf("  Total Voltage: %.3f V\n", (float)total_voltage_mv / 1000.0f);

    Serial.print("  Cell Voltages (V): ");
    for(int i = 0; i < 16; i++) {
        Serial.printf("%.3f", (float)msg->voltages[i] / 1000.0f);
        if (i < 15) Serial.print(", ");
    }
    Serial.println("\n--------------------------");
}

/**
 * @brief Helper function to print the meaning of each bit in a failure byte.
 * @param title The title for this section of flags.
 * @param byte The byte containing the flags.
 * @param descriptions An array of 8 strings describing each bit.
 */
void print_failure_byte(const char* title, uint8_t byte, const char* descriptions[8]) {
    bool has_error = false;
    for (int i = 0; i < 8; i++) {
        if ((byte >> i) & 0x01) {
            if (!has_error) {
                Serial.printf("  %s:\n", title);
                has_error = true;
            }
            Serial.printf("    - %s\n", descriptions[i]);
        }
    }
}

/**
 * @brief Prints the detailed contents of a mavlink_bms_status_t message.
 * @param msg A pointer to the decoded mavlink_bms_status_t message.
 */
void print_bms_status_message(const mavlink_bms_status_t* msg) {
    Serial.println("--- MAVLink BMS Status Message ---");

    // Decode and print the general status byte
    const char* state_str = "Unknown";
    uint8_t state_val = (msg->status >> 2) & 0x03;
    if (state_val == 0) state_str = "Stationary";
    else if (state_val == 1) state_str = "Charging";
    else if (state_val == 2) state_str = "Discharging";
    Serial.printf("  State: %s\n", state_str);
    Serial.printf("  Charge MOS: %s\n", (msg->status >> 4) & 0x01 ? "ON" : "OFF");
    Serial.printf("  Discharge MOS: %s\n", (msg->status >> 5) & 0x01 ? "ON" : "OFF");

    // --- Print Failure Flags ---
    const char* byte0_desc[8] = {"Cell volt high lvl 1", "Cell volt high lvl 2", "Cell volt low lvl 1", "Cell volt low lvl 2", "Sum volt high lvl 1", "Sum volt high lvl 2", "Sum volt low lvl 1", "Sum volt low lvl 2"};
    print_failure_byte("Voltage Failures", msg->failure_flags_byte0, byte0_desc);

    const char* byte1_desc[8] = {"Chg temp high lvl 1", "Chg temp high lvl 2", "Chg temp low lvl 1", "Chg temp low lvl 2", "Dischg temp high lvl 1", "Dischg temp high lvl 2", "Dischg temp low lvl 1", "Dischg temp low lvl 2"};
    print_failure_byte("Temperature Failures", msg->failure_flags_byte1, byte1_desc);

    const char* byte2_desc[8] = {"Chg overcurrent lvl 1", "Chg overcurrent lvl 2", "Dischg overcurrent lvl 1", "Dischg overcurrent lvl 2", "SOC high lvl 1", "SOC high lvl 2", "SOC low lvl 1", "SOC low lvl 2"};
    print_failure_byte("Current/SOC Failures", msg->failure_flags_byte2, byte2_desc);

    const char* byte3_desc[8] = {"Diff volt lvl 1", "Diff volt lvl 2", "Diff temp lvl 1", "Diff temp lvl 2", "Reserved", "Reserved", "Reserved", "Reserved"};
    print_failure_byte("Differential Failures", msg->failure_flags_byte3, byte3_desc);

    const char* byte4_desc[8] = {"Chg MOS temp high", "Dischg MOS temp high", "Chg MOS temp sensor err", "Dischg MOS temp sensor err", "Chg MOS adhesion err", "Dischg MOS adhesion err", "Chg MOS open circuit err", "Dischg MOS open circuit err"};
    print_failure_byte("MOSFET/Sensor Failures", msg->failure_flags_byte4, byte4_desc);

    const char* byte5_desc[8] = {"AFE collect chip err", "Voltage collect dropped", "Cell temp sensor err", "EEPROM err", "RTC err", "Precharge failure", "Communication failure", "Internal comm failure"};
    print_failure_byte("System/Component Failures", msg->failure_flags_byte5, byte5_desc);
    
    const char* byte6_desc[8] = {"Current module fault", "Sum voltage detect fault", "Short circuit protect fault", "Low volt forbidden chg", "Reserved", "Reserved", "Reserved", "Reserved"};
    print_failure_byte("Other Failures", msg->failure_flags_byte6, byte6_desc);

    if (msg->fault_code_byte7 != 0) {
        Serial.printf("  Raw Fault Code Byte 7: 0x%02X\n", msg->fault_code_byte7);
    }
    
    Serial.println("--------------------------------");
}

/**
 * @brief Prints the contents of a mavlink_ezkontrol_mcu_meter_data_i_t message, applying scaling.
 * @param msg A pointer to the decoded mavlink_ezkontrol_mcu_meter_data_i_t message.
 */
void print_motor_data_i_message(const mavlink_ezkontrol_mcu_meter_data_i_t* msg) {
    Serial.printf("--- MAVLink Motor Data I (%s) ---\n", msg->instance == 0 ? "Left" : "Right");
    Serial.printf("  Bus Voltage: %.1f V\n", (float)msg->bus_voltage / 10.0f);
    Serial.printf("  Bus Current: %.1f A\n", (float)msg->bus_current / 10.0f);
    Serial.printf("  RPM: %d\n", msg->rpm);
    Serial.printf("  Accelerator: %u %%\n", msg->accelerator_opening);
    Serial.println("---------------------------------");
}

/**
 * @brief Prints the contents of a mavlink_ezkontrol_mcu_meter_data_ii_t message, applying scaling.
 * @param msg A pointer to the decoded mavlink_ezkontrol_mcu_meter_data_ii_t message.
 */
void print_motor_data_ii_message(const mavlink_ezkontrol_mcu_meter_data_ii_t* msg) {
    Serial.printf("--- MAVLink Motor Data II (%s) ---\n", msg->instance == 0 ? "Left" : "Right");
    Serial.printf("  Controller Temp: %d C\n", msg->controller_temperature - 40);
    Serial.printf("  Motor Temp: %d C\n", msg->motor_temperature - 40);
    Serial.printf("  Status Byte: 0x%02X\n", msg->status);
    Serial.printf("  Error Flags: 0x%02X 0x%02X 0x%02X\n", msg->error_flags_byte4, msg->error_flags_byte5, msg->error_flags_byte6);
    Serial.printf("  Life Signal: %u\n", msg->life_signal);
    Serial.println("----------------------------------");
}

/**
 * @brief Prints the contents of a mavlink_temperatures_t message, applying scaling.
 * @param msg A pointer to the decoded mavlink_temperatures_t message.
 */
void print_temperatures_message(const mavlink_temperatures_t* msg) {
    Serial.println("--- MAVLink Temperatures Message ---");
    Serial.printf("  Battery (Left/Right): %.2f C, %.2f C\n", (float)msg->temperature_battery_left / 100.0f, (float)msg->temperature_battery_right / 100.0f);
    Serial.printf("  MPPT (Left/Right): %.2f C, %.2f C\n", (float)msg->temperature_mppt_left / 100.0f, (float)msg->temperature_mppt_right / 100.0f);
    Serial.println("------------------------------------");
}

/**
 * @brief Prints the contents of a mavlink_debug_t message.
 * @param msg A pointer to the decoded mavlink_debug_t message.
 */
void print_debug_message(const mavlink_debug_t* msg) {
    Serial.println("--- MAVLink Debug Message ---");
    Serial.printf("  Index: %u\n", msg->ind);
    Serial.printf("  Value: %f\n", msg->value);
    Serial.printf("  Timestamp: %lu ms\n", msg->time_boot_ms);
    Serial.println("-----------------------------");
}

/**
 * @brief Prints the contents of a mavlink_gps_t message, applying scaling.
 * @param msg A pointer to the decoded mavlink_gps_t message.
 */
void print_gps_message(const mavlink_gps_t* msg) {
    Serial.println("--- MAVLink GPS Message ---");
    Serial.printf("  Lat: %.7f deg\n", (float)msg->latitude / 10000000.0f);
    Serial.printf("  Lon: %.7f deg\n", (float)msg->longitude / 10000000.0f);
    Serial.printf("  Speed: %.2f m/s\n", (float)msg->speed / 100.0f);
    Serial.printf("  Course: %u deg\n", msg->course);
    Serial.printf("  Heading: %u deg\n", msg->heading);
    Serial.printf("  Sats Visible: %u\n", msg->satellites_visible);
    Serial.printf("  HDOP: %.2f\n", (float)msg->hdop / 100.0f); // Assuming HDOP is scaled by 100
    Serial.println("---------------------------");
}

/**
 * @brief Prints the contents of a mavlink_instrumentation_t message, applying scaling.
 * @param msg A pointer to the decoded mavlink_instrumentation_t message.
 */
void print_instrumentation_message(const mavlink_instrumentation_t* msg) {
    Serial.println("--- MAVLink Instrumentation Message ---");
    Serial.printf("  Battery Voltage: %.2f V\n", (float)msg->battery_voltage / 100.0f);
    Serial.printf("  Battery Current: %.2f A\n", (float)msg->battery_current / 100.0f);
    Serial.printf("  Motor Current (L/R): %.2f A, %.2f A\n", (float)msg->motor_current_left / 100.0f, (float)msg->motor_current_right / 100.0f);
    Serial.printf("  MPPT Current: %.2f A\n", (float)msg->mppt_current / 100.0f);
    Serial.printf("  Aux Voltage: %.2f V\n", (float)msg->auxiliary_battery_voltage / 100.0f);
    Serial.printf("  Aux Current: %.2f A\n", (float)msg->auxiliary_battery_current / 100.0f);
    Serial.printf("  Irradiance: %u W/m^2\n", msg->irradiance);
    Serial.println("---------------------------------------");
}

/**
 * @brief Prints the contents of a mavlink_mppt_t message, applying scaling.
 * @param msg A pointer to the decoded mavlink_mppt_t message.
 */
void print_mppt_message(const mavlink_mppt_t* msg) {
    Serial.println("--- MAVLink MPPT Message ---");
    Serial.printf("  PV Voltage: %.2f V\n", (float)msg->pv_voltage / 100.0f);
    Serial.printf("  PV Current: %.2f A\n", (float)msg->pv_current / 100.0f); // Assuming cA
    Serial.printf("  Battery Voltage: %.2f V\n", (float)msg->battery_voltage / 100.0f);
    Serial.printf("  Battery Current: %.2f A\n", (float)msg->battery_current / 100.0f);
    Serial.println("----------------------------");
}

/**
 * @brief Prints the detailed contents of a mavlink_mppt_state_t message.
 * @param msg A pointer to the decoded mavlink_mppt_state_t message.
 */
void print_mppt_state_message(const mavlink_mppt_state_t* msg) {
    Serial.println("--- MAVLink MPPT State Message ---");
    
    // --- Decode Battery Status (Register 3200) ---
    uint16_t b_status = msg->battery_status;
    Serial.println("  Battery Status:");
    
    uint8_t volt_status = b_status & 0x0F;
    const char* volt_str = "Fault";
    if (volt_status == 0x00) volt_str = "Normal";
    else if (volt_status == 0x01) volt_str = "Overvolt";
    else if (volt_status == 0x02) volt_str = "Under Volt";
    else if (volt_status == 0x03) volt_str = "Low Volt Disconnect";
    Serial.printf("    - Voltage: %s\n", volt_str);

    uint8_t temp_status = (b_status >> 4) & 0x0F;
    const char* temp_str = "Normal";
    if (temp_status == 0x01) temp_str = "Over Temp";
    else if (temp_status == 0x02) temp_str = "Low Temp";
    Serial.printf("    - Temperature: %s\n", temp_str);

    if ((b_status >> 8) & 0x01) {
        Serial.println("    - Internal Resistance: Abnormal");
    }
    if ((b_status >> 15) & 0x01) {
        Serial.println("    - Rated Voltage ID: Wrong");
    }

    // --- Decode Charging Equipment Status (Register 3201) ---
    uint16_t c_status = msg->charging_equipment_status;
    Serial.println("  Charging Equipment Status:");

    Serial.printf("    - State: %s\n", (c_status & 0x01) ? "Running" : "Standby");
    if ((c_status >> 1) & 0x01) {
        Serial.println("    - General Fault Detected");
    }
    
    uint8_t charge_status = (c_status >> 2) & 0x03;
    const char* charge_str = "No charging";
    if (charge_status == 0x01) charge_str = "Float";
    else if (charge_status == 0x02) charge_str = "Boost";
    else if (charge_status == 0x03) charge_str = "Equalization";
    Serial.printf("    - Charging Mode: %s\n", charge_str);

    uint8_t input_volt_status = (c_status >> 14) & 0x03;
    const char* input_volt_str = "Normal";
    if (input_volt_status == 0x01) input_volt_str = "No power connected";
    else if (input_volt_status == 0x02) input_volt_str = "Higher volt input";
    else if (input_volt_status == 0x03) input_volt_str = "Input volt error";
    Serial.printf("    - Input Voltage: %s\n", input_volt_str);

    if ((c_status >> 4) & 0x01) Serial.println("    - FAULT: PV Input Short");
    if ((c_status >> 7) & 0x01) Serial.println("    - FAULT: Load MOSFET Short");
    if ((c_status >> 8) & 0x01) Serial.println("    - FAULT: Load Short");
    if ((c_status >> 9) & 0x01) Serial.println("    - FAULT: Load Over Current");
    if ((c_status >> 10) & 0x01) Serial.println("    - FAULT: Input Over Current");
    if ((c_status >> 11) & 0x01) Serial.println("    - FAULT: Anti-reverse MOSFET Short");
    if ((c_status >> 12) & 0x01) Serial.println("    - FAULT: Charging or Anti-reverse MOSFET Short");
    if ((c_status >> 13) & 0x01) Serial.println("    - FAULT: Charging MOSFET Short");

    Serial.println("--------------------------------");
}

/**
 * @brief Prints the contents of a mavlink_named_value_float_t message.
 * @param msg A pointer to the decoded mavlink_named_value_float_t message.
 */
void print_named_value_float_message(const mavlink_named_value_float_t* msg) {
    char name_buffer[11];
    memcpy(name_buffer, msg->name, 10);
    name_buffer[10] = '\0'; // Ensure null termination
    Serial.println("--- MAVLink Named Value (Float) ---");
    Serial.printf("  Name: %s\n", name_buffer);
    Serial.printf("  Value: %f\n", msg->value);
    Serial.println("-------------------------------------");
}

/**
 * @brief Prints the contents of a mavlink_named_value_int_t message.
 * @param msg A pointer to the decoded mavlink_named_value_int_t message.
 */
void print_named_value_int_message(const mavlink_named_value_int_t* msg) {
    char name_buffer[11];
    memcpy(name_buffer, msg->name, 10);
    name_buffer[10] = '\0'; // Ensure null termination
    Serial.println("--- MAVLink Named Value (Int) ---");
    Serial.printf("  Name: %s\n", name_buffer);
    Serial.printf("  Value: %ld\n", msg->value);
    Serial.println("-----------------------------------");
}

/**
 * @brief Prints the contents of a mavlink_pumps_t message.
 * @param msg A pointer to the decoded mavlink_pumps_t message.
 */
void print_pumps_message(const mavlink_pumps_t* msg) {
    Serial.println("--- MAVLink Pumps Message ---");
    Serial.printf("  Pump Left: %s\n", (msg->pump_states & 0x01) ? "ON" : "OFF");
    Serial.printf("  Pump Right: %s\n", (msg->pump_states & 0x02) ? "ON" : "OFF");
    Serial.println("-----------------------------");
}

/**
 * @brief Prints the contents of a mavlink_statustext_t message.
 * @param msg A pointer to the decoded mavlink_statustext_t message.
 */
void print_statustext_message(const mavlink_statustext_t* msg) {
    // Copy text to a null-terminated buffer
    char text_buffer[51];
    memcpy(text_buffer, msg->text, 50);
    text_buffer[50] = '\0';

    const char* severity_str = "UNKNOWN";
    switch(msg->severity) {
        case MAV_SEVERITY_EMERGENCY: severity_str = "EMERGENCY"; break;
        case MAV_SEVERITY_ALERT:     severity_str = "ALERT"; break;
        case MAV_SEVERITY_CRITICAL:  severity_str = "CRITICAL"; break;
        case MAV_SEVERITY_ERROR:     severity_str = "ERROR"; break;
        case MAV_SEVERITY_WARNING:   severity_str = "WARNING"; break;
        case MAV_SEVERITY_NOTICE:    severity_str = "NOTICE"; break;
        case MAV_SEVERITY_INFO:      severity_str = "INFO"; break;
        case MAV_SEVERITY_DEBUG:     severity_str = "DEBUG"; break;
    }
    
    Serial.println("--- MAVLink Status Text ---");
    Serial.printf("  Severity: %s\n", severity_str);
    Serial.printf("  Text: %s\n", text_buffer);
    Serial.println("-----------------------------");
}

/**
 * @brief Safely prints a MAVLink string parameter.
 * @param param_id The char array from the MAVLink message.
 */
void print_mavlink_param_id(const char* param_id) {
    char id_buffer[17];
    memcpy(id_buffer, param_id, 16);
    id_buffer[16] = '\0';
    Serial.printf("  Param ID: %s\n", id_buffer);
}

/**
 * @brief Prints the contents of a mavlink_param_request_read_t message.
 * @param msg A pointer to the decoded mavlink_param_request_read_t message.
 */
void print_param_request_read_message(const mavlink_param_request_read_t* msg) {
    Serial.println("--- MAVLink Param Request Read ---");
    print_mavlink_param_id(msg->param_id);
    Serial.printf("  Param Index: %d\n", msg->param_index);
    Serial.println("------------------------------------");
}

/**
 * @brief Prints the contents of a mavlink_param_set_t message.
 * @param msg A pointer to the decoded mavlink_param_set_t message.
 */
void print_param_set_message(const mavlink_param_set_t* msg) {
    Serial.println("--- MAVLink Param Set ---");
    print_mavlink_param_id(msg->param_id);
    Serial.printf("  Value: %f\n", msg->param_value);
    Serial.printf("  Type: %u\n", msg->param_type);
    Serial.println("-------------------------");
}

/**
 * @brief Prints the contents of a mavlink_param_value_t message.
 * @param msg A pointer to the decoded mavlink_param_value_t message.
 */
void print_param_value_message(const mavlink_param_value_t* msg) {
    Serial.println("--- MAVLink Param Value ---");
    print_mavlink_param_id(msg->param_id);
    Serial.printf("  Value: %f\n", msg->param_value);
    Serial.printf("  Type: %u\n", msg->param_type);
    Serial.printf("  Count: %u\n", msg->param_count);
    Serial.printf("  Index: %u\n", msg->param_index);
    Serial.println("---------------------------");
}

/**
 * @brief Main dispatcher function. Decodes a MAVLink message and calls the appropriate print function.
 * @param msg A pointer to the received mavlink_message_t.
 */
void print_received_mavlink_message(const mavlink_message_t* msg) {
    switch (msg->msgid) {
        case MAVLINK_MSG_ID_BMS: {
            mavlink_bms_t bms_msg;
            mavlink_msg_bms_decode(msg, &bms_msg);
            print_bms_message(&bms_msg);
            break;
        }

        case MAVLINK_MSG_ID_BMS_STATUS: {
            mavlink_bms_status_t bms_status_msg;
            mavlink_msg_bms_status_decode(msg, &bms_status_msg);
            print_bms_status_message(&bms_status_msg);
            break;
        }

        case MAVLINK_MSG_ID_EZKONTROL_MCU_METER_DATA_I: {
            mavlink_ezkontrol_mcu_meter_data_i_t motor_i_msg;
            mavlink_msg_ezkontrol_mcu_meter_data_i_decode(msg, &motor_i_msg);
            print_motor_data_i_message(&motor_i_msg);
            break;
        }
        case MAVLINK_MSG_ID_EZKONTROL_MCU_METER_DATA_II: {
            mavlink_ezkontrol_mcu_meter_data_ii_t motor_ii_msg;
            mavlink_msg_ezkontrol_mcu_meter_data_ii_decode(msg, &motor_ii_msg);
            print_motor_data_ii_message(&motor_ii_msg);
            break;
        }
        case MAVLINK_MSG_ID_TEMPERATURES: {
            mavlink_temperatures_t temp_msg;
            mavlink_msg_temperatures_decode(msg, &temp_msg);
            print_temperatures_message(&temp_msg);
            break;
        }

        case MAVLINK_MSG_ID_DEBUG: {
            mavlink_debug_t debug_msg;
            mavlink_msg_debug_decode(msg, &debug_msg);
            print_debug_message(&debug_msg);
            break;
        }
        case MAVLINK_MSG_ID_GPS: {
            mavlink_gps_t gps_msg;
            mavlink_msg_gps_decode(msg, &gps_msg);
            print_gps_message(&gps_msg);
            break;
        }
        case MAVLINK_MSG_ID_INSTRUMENTATION: {
            mavlink_instrumentation_t inst_msg;
            mavlink_msg_instrumentation_decode(msg, &inst_msg);
            print_instrumentation_message(&inst_msg);
            break;
        }
        case MAVLINK_MSG_ID_MPPT: {
            mavlink_mppt_t mppt_msg;
            mavlink_msg_mppt_decode(msg, &mppt_msg);
            print_mppt_message(&mppt_msg);
            break;
        }

        case MAVLINK_MSG_ID_MPPT_STATE: {
            mavlink_mppt_state_t mppt_state_msg;
            mavlink_msg_mppt_state_decode(msg, &mppt_state_msg);
            print_mppt_state_message(&mppt_state_msg);
            break;
        }

        case MAVLINK_MSG_ID_NAMED_VALUE_FLOAT: {
            mavlink_named_value_float_t nvfloat_msg;
            mavlink_msg_named_value_float_decode(msg, &nvfloat_msg);
            print_named_value_float_message(&nvfloat_msg);
            break;
        }
        case MAVLINK_MSG_ID_NAMED_VALUE_INT: {
            mavlink_named_value_int_t nvint_msg;
            mavlink_msg_named_value_int_decode(msg, &nvint_msg);
            print_named_value_int_message(&nvint_msg);
            break;
        }
        case MAVLINK_MSG_ID_PUMPS: {
            mavlink_pumps_t pumps_msg;
            mavlink_msg_pumps_decode(msg, &pumps_msg);
            print_pumps_message(&pumps_msg);
            break;
        }
        case MAVLINK_MSG_ID_STATUSTEXT: {
            mavlink_statustext_t status_msg;
            mavlink_msg_statustext_decode(msg, &status_msg);
            print_statustext_message(&status_msg);
            break;
        }

        case MAVLINK_MSG_ID_PARAM_REQUEST_READ: {
            mavlink_param_request_read_t param_req_msg;
            mavlink_msg_param_request_read_decode(msg, &param_req_msg);
            print_param_request_read_message(&param_req_msg);
            break;
        }
        case MAVLINK_MSG_ID_PARAM_SET: {
            mavlink_param_set_t param_set_msg;
            mavlink_msg_param_set_decode(msg, &param_set_msg);
            print_param_set_message(&param_set_msg);
            break;
        }
        case MAVLINK_MSG_ID_PARAM_VALUE: {
            mavlink_param_value_t param_val_msg;
            mavlink_msg_param_value_decode(msg, &param_val_msg);
            print_param_value_message(&param_val_msg);
            break;
        }

        default:
            // This function does not handle the received message ID.
            break;
    }
}

/**
 * @brief Converts a message_t object to a mavlink_message_t object.
 * * @param message The input message_t object to be converted.
 * @param mavlink_msg The output mavlink_message_t object.
 * @return true if the conversion was successful, false otherwise.
 */
bool mavlink_msg_from_message_t(message_t message, mavlink_message_t *mavlink_msg) {
    // Set the system and component IDs for the MAVLink message.
    uint8_t system_id = 1; 
    uint8_t component_id = MAV_COMP_ID_ALL;

    // The switch statement determines which MAVLink message to create based on the data source.
    switch (message.source) {
        case DATA_SOURCE_BMS: {
            // Extract BMS data from the payload.
            bms_data_t bms_data = message.payload.bms;
            
            // Temporary arrays to hold cell voltages and temperatures.
            uint16_t voltages[16] = {0};
            int16_t temperatures[2] = {0}; 

            // This loop populates the voltages array from the BMS data structure.
            for (int i = 0; i < 16; i++) {
                // The cell_voltage_frame contains voltages for 3 cells.
                // We calculate the correct frame and index for each of the 16 cells.
                voltages[i] = bms_data.cell_voltage_frame[i / 3].voltages_mv[i % 3];
            }

            temperatures[0] = bms_data.temperature_frame[0].raw_temps[0];
            temperatures[1] = bms_data.temperature_frame[0].raw_temps[1];

            int8_t soc_quick_fix = bms_data.voltage_data.soc_decipercent / 10; //!Must fix mavlink dialect to uint16_t and remove this division

            // Pack the data into a MAVLink BMS message.
            mavlink_msg_bms_pack(
                system_id,
                component_id,
                mavlink_msg,
                voltages,
                temperatures,
                bms_data.voltage_data.current_deciamps,
                soc_quick_fix,
                message.timestamp.epoch_seconds,
                message.timestamp.epoch_ms
            );
            break;
        }

        case DATA_SOURCE_MOTOR_LEFT:
        case DATA_SOURCE_MOTOR_RIGHT: {
            // Extract motor data from the payload.
            motor_data_t motor_data = message.payload.motor;
            uint8_t instance = (message.source == DATA_SOURCE_MOTOR_LEFT) ? 0 : 1;

            // Pack the electrical data into the first type of MAVLink motor message.
            mavlink_msg_ezkontrol_mcu_meter_data_i_pack(
                system_id,
                component_id,
                mavlink_msg,
                motor_data.electrical_data.bus_voltage_dV,
                motor_data.electrical_data.bus_current_dA,
                motor_data.electrical_data.rpm,
                motor_data.state_data.accelerator_percent,
                instance,
                message.timestamp.epoch_seconds,
                message.timestamp.epoch_ms
            );

            // An additional MAVLink message can be sent for the motor's state data.
            // Note that the original function can only return one message.
            // To send both, the architecture would need to be adjusted.
            mavlink_message_t mavlink_msg_ii;
            mavlink_msg_ezkontrol_mcu_meter_data_ii_pack(
                system_id,
                component_id,
                &mavlink_msg_ii,
                motor_data.state_data.controller_temp_C,
                motor_data.state_data.motor_temp_C,
                motor_data.state_data.status,
                (uint8_t)(motor_data.state_data.error & 0xFF),
                (uint8_t)((motor_data.state_data.error >> 8) & 0xFF),
                (uint8_t)((motor_data.state_data.error >> 16) & 0xFF),
                0, // life_signal is not in motor_data_t
                instance,
                message.timestamp.epoch_seconds,
                message.timestamp.epoch_ms
            );
            // Since this function can only return one mavlink_message_t, 
            // the second message for motor data is created but not returned.
            // The logic would need to be modified to handle multiple messages.
            break;
        }
        // Cases for other data sources are not yet implemented.
        case DATA_SOURCE_GPS: {
            gps_data_t gps_data = message.payload.gps;
            mavlink_msg_gps_pack(
                system_id, component_id, mavlink_msg,
                gps_data.latitude_degE7,
                gps_data.longitude_degE7,
                gps_data.speed_cm_s,
                gps_data.course,
                gps_data.heading,
                gps_data.satellites_visible,
                gps_data.hdop_deciunits,
                message.timestamp.epoch_seconds,
                message.timestamp.epoch_ms
            );
            break;
        }
        case DATA_SOURCE_MPPT: {
            mppt_data_t mppt_data = message.payload.mppt;
            // Similar to BMS/Motor, MPPT data is split. We send the primary power data.
            // The MPPT_STATE message is not sent by this call.
            mavlink_msg_mppt_pack(
                system_id, component_id, mavlink_msg,
                mppt_data.electrical.pv_voltage_cV,
                mppt_data.electrical.pv_current_cA,
                mppt_data.electrical.battery_voltage_cV,
                mppt_data.electrical.battery_current_cA,
                message.timestamp.epoch_seconds,
                message.timestamp.epoch_ms
            );
            break;
        }
        case DATA_SOURCE_INSTRUMENTATION: {
            // instrumentation_data_t inst_data = message.payload.instrumentation;
            // mavlink_msg_instrumentation_pack(
            //     system_id, component_id, mavlink_msg,
            //     inst_data.battery_current_cA,
            //     inst_data.motor_current_left_cA,
            //     inst_data.motor_current_right_cA,
            //     inst_data.mppt_current_cA,
            //     inst_data.auxiliary_battery_current_cA,
            //     inst_data.battery_voltage_cV,
            //     inst_data.auxiliary_battery_voltage_cV,
            //     inst_data.irradiance,
            //     message.timestamp.epoch_seconds,
            //     message.timestamp.epoch_ms
            // );
            break;
        }
        case DATA_SOURCE_TEMPERATURES: {
            // temperature_data_t temp_data = message.payload.temperature;
            // mavlink_msg_temperatures_pack(
            //     system_id, component_id, mavlink_msg,
            //     temp_data.battery_left_cdegC,
            //     temp_data.battery_right_cdegC,
            //     temp_data.mppt_left_cdegC,
            //     temp_data.mppt_right_cdegC,
            //     message.timestamp.epoch_seconds,
            //     message.timestamp.epoch_ms
            // );
            break;
        }
        case DATA_SOURCE_PROPULSION: {
            // propulsion_data_t prop_data = message.payload.propulsion;
            // mavlink_msg_eletronic_propulsion_pack(
            //     system_id, component_id, mavlink_msg,
            //     prop_data.helm_potentiometer_volts,
            //     prop_data.throttle_left_potentiometer_volts,
            //     prop_data.throttle_right_potentiometer_volts,
            //     prop_data.state,
            //     message.timestamp.epoch_seconds,
            //     message.timestamp.epoch_ms
            // );
            break;
        }
        default:
            // If the message source is not recognized, the conversion fails.
            return false;
    }

    // A non-null message ID indicates that the packing was successful.
    return mavlink_msg->msgid != 0;
}


/**
 * @brief Converts a mavlink_message_t object to a message_t object.
 *
 * This function is the counterpart to mavlink_msg_from_message_t. It takes a MAVLink
 * message and converts it back into the internal message_t format. Note that for
 * data types that are split across multiple MAVLink messages (like the motor data),
 * this function will only populate the fields present in the given mavlink_message_t.
 * The caller may need to manage the reconstruction of the complete data structure.
 *
 * @param mavlink_msg The input mavlink_message_t object to be converted.
 * @param message The output message_t object.
 * @return true if the conversion was successful, false otherwise.
 */
bool message_t_from_mavlink_msg(const mavlink_message_t *mavlink_msg, message_t *message) {
    // Determine the message type from the MAVLink message ID and process accordingly.
    switch (mavlink_msg->msgid) {
        case MAVLINK_MSG_ID_BMS: {
            // Decode the MAVLink message into a BMS data struct.
            mavlink_bms_t bms_mavlink_data;
            mavlink_msg_bms_decode(mavlink_msg, &bms_mavlink_data);

            // Set the data source in our internal message structure.
            message->source = DATA_SOURCE_BMS;
            
            // Access the payload for writing.
            bms_data_t *bms_data = &message->payload.bms;

            // Populate the voltage and SOC data.
            bms_data->voltage_data.current_deciamps = bms_mavlink_data.current_battery;
            bms_data->voltage_data.soc_decipercent = bms_mavlink_data.state_of_charge * 10; //! Must fix mavlink dialect to uint16_t and remove this multiplication
            
            // Populate cell voltages.
            for (int i = 0; i < 16; i++) {
                bms_data->cell_voltage_frame[i / 3].voltages_mv[i % 3] = bms_mavlink_data.voltages[i];
            }
            
            // Note: The mavlink_bms_t message doesn't contain all fields from bms_data_t.
            // Fields like charge_discharge_status, bms_status, etc., are not present
            // and will remain uninitialized unless handled elsewhere.

            // Set the timestamp for the message.
            message->timestamp.epoch_seconds = bms_mavlink_data.timestamp_seconds;
            message->timestamp.epoch_ms = bms_mavlink_data.timestamp_milliseconds;
            break;
        }  
        case MAVLINK_MSG_ID_BMS_STATUS: {
            // Decode the MAVLink message into a BMS status struct.
            mavlink_bms_status_t bms_status_mavlink_data;
            mavlink_msg_bms_status_decode(mavlink_msg, &bms_status_mavlink_data);

            // Set the data source in our internal message structure.
            message->source = DATA_SOURCE_BMS;

            // Access the payload for writing.
            bms_data_t *bms_data = &message->payload.bms;

            // Populate the BMS status fields.
            bms_data->bms_status.charger_status = (bms_status_mavlink_data.status & 0x01);
            bms_data->bms_status.load_status = (bms_status_mavlink_data.status & 0x02) >> 1;

            // Populate the charge/discharge status fields
            bms_data->charge_discharge_status.state = (bms_status_mavlink_data.status >> 2) & 0x03;
            bms_data->charge_discharge_status.charge_mos = (bms_status_mavlink_data.status >> 4) & 0x01;
            bms_data->charge_discharge_status.discharge_mos = (bms_status_mavlink_data.status >> 5) & 0x01;
            
            // Populate the failure status fields.
            bms_data->failure_status.raw[0] = bms_status_mavlink_data.failure_flags_byte0;
            bms_data->failure_status.raw[1] = bms_status_mavlink_data.failure_flags_byte1;
            bms_data->failure_status.raw[2] = bms_status_mavlink_data.failure_flags_byte2;
            bms_data->failure_status.raw[3] = bms_status_mavlink_data.failure_flags_byte3;
            bms_data->failure_status.raw[4] = bms_status_mavlink_data.failure_flags_byte4;
            bms_data->failure_status.raw[5] = bms_status_mavlink_data.failure_flags_byte5;
            bms_data->failure_status.raw[6] = bms_status_mavlink_data.failure_flags_byte6;
            bms_data->failure_status.raw[7] = bms_status_mavlink_data.fault_code_byte7;

            // Set the timestamp for the message.
            message->timestamp.epoch_seconds = bms_status_mavlink_data.timestamp_seconds;
            message->timestamp.epoch_ms = bms_status_mavlink_data.timestamp_milliseconds;
            break;
        }
        case MAVLINK_MSG_ID_EZKONTROL_MCU_METER_DATA_I: {
            // Decode the MAVLink message into a motor data struct.
            mavlink_ezkontrol_mcu_meter_data_i_t motor_mavlink_data_i;
            mavlink_msg_ezkontrol_mcu_meter_data_i_decode(mavlink_msg, &motor_mavlink_data_i);

            // Determine if it's the left or right motor.
            message->source = (motor_mavlink_data_i.instance == 0) ? DATA_SOURCE_MOTOR_LEFT : DATA_SOURCE_MOTOR_RIGHT;
            
            // Access the payload for writing.
            motor_data_t *motor_data = &message->payload.motor;
            motor_data->motor = (motor_mavlink_data_i.instance == 0) ? LEFT_MOTOR : RIGHT_MOTOR;

            // Populate the electrical data part of the motor data.
            motor_data->electrical_data.bus_voltage_dV = motor_mavlink_data_i.bus_voltage;
            motor_data->electrical_data.bus_current_dA = motor_mavlink_data_i.bus_current;
            motor_data->electrical_data.rpm = motor_mavlink_data_i.rpm;
            motor_data->state_data.accelerator_percent = motor_mavlink_data_i.accelerator_opening;

            // Set the timestamp.
            message->timestamp.epoch_seconds = motor_mavlink_data_i.timestamp_seconds;
            message->timestamp.epoch_ms = motor_mavlink_data_i.timestamp_milliseconds;
            break;
        }
        case MAVLINK_MSG_ID_EZKONTROL_MCU_METER_DATA_II: {
            // Decode the second type of motor data message.
            mavlink_ezkontrol_mcu_meter_data_ii_t motor_mavlink_data_ii;
            mavlink_msg_ezkontrol_mcu_meter_data_ii_decode(mavlink_msg, &motor_mavlink_data_ii);
            
            // Determine the motor instance.
            message->source = (motor_mavlink_data_ii.instance == 0) ? DATA_SOURCE_MOTOR_LEFT : DATA_SOURCE_MOTOR_RIGHT;

            // Access the payload for writing.
            motor_data_t *motor_data = &message->payload.motor;
            motor_data->motor = (motor_mavlink_data_ii.instance == 0) ? LEFT_MOTOR : RIGHT_MOTOR;
            
            // Populate the state data part of the motor data.
            motor_data->state_data.controller_temp_C = motor_mavlink_data_ii.controller_temperature;
            motor_data->state_data.motor_temp_C = motor_mavlink_data_ii.motor_temperature;
            motor_data->state_data.status = motor_mavlink_data_ii.status;
            // Reconstruct the 32-bit error code from the three bytes in the MAVLink message.
            motor_data->state_data.error = motor_mavlink_data_ii.error_flags_byte4 |
                                           ((uint32_t)motor_mavlink_data_ii.error_flags_byte5 << 8) |
                                           ((uint32_t)motor_mavlink_data_ii.error_flags_byte6 << 16);

            // Set the timestamp.
            message->timestamp.epoch_seconds = motor_mavlink_data_ii.timestamp_seconds;
            message->timestamp.epoch_ms = motor_mavlink_data_ii.timestamp_milliseconds;
            break;
        }
        case MAVLINK_MSG_ID_GPS: {
            message->source = DATA_SOURCE_GPS;
            mavlink_gps_t gps_data;
            mavlink_msg_gps_decode(mavlink_msg, &gps_data);
            message->payload.gps.latitude_degE7 = gps_data.latitude;
            message->payload.gps.longitude_degE7 = gps_data.longitude;
            message->payload.gps.speed_cm_s = gps_data.speed;
            message->payload.gps.course = gps_data.course;
            message->payload.gps.heading = gps_data.heading;
            message->payload.gps.satellites_visible = gps_data.satellites_visible;
            message->payload.gps.hdop_deciunits = gps_data.hdop;
            message->timestamp.epoch_seconds = gps_data.timestamp_seconds;
            message->timestamp.epoch_ms = gps_data.timestamp_milliseconds;
            break;
        }
        case MAVLINK_MSG_ID_MPPT: {
            message->source = DATA_SOURCE_MPPT;
            mavlink_mppt_t mppt_data;
            mavlink_msg_mppt_decode(mavlink_msg, &mppt_data);
            message->payload.mppt.electrical.pv_voltage_cV = mppt_data.pv_voltage;
            message->payload.mppt.electrical.pv_current_cA = mppt_data.pv_current;
            message->payload.mppt.electrical.battery_voltage_cV = mppt_data.battery_voltage;
            message->payload.mppt.electrical.battery_current_cA = mppt_data.battery_current;
            message->timestamp.epoch_seconds = mppt_data.timestamp_seconds;
            message->timestamp.epoch_ms = mppt_data.timestamp_milliseconds;
            break;
        }
        case MAVLINK_MSG_ID_MPPT_STATE: {
            message->source = DATA_SOURCE_MPPT;
            mavlink_mppt_state_t mppt_state_data;
            mavlink_msg_mppt_state_decode(mavlink_msg, &mppt_state_data);
            message->payload.mppt.state.battery_status = mppt_state_data.battery_status;
            message->payload.mppt.state.charging_equipment_status = mppt_state_data.charging_equipment_status;
            message->timestamp.epoch_seconds = mppt_state_data.timestamp_seconds;
            message->timestamp.epoch_ms = mppt_state_data.timestamp_milliseconds;
            break;
        }
        case MAVLINK_MSG_ID_INSTRUMENTATION: {
            message->source = DATA_SOURCE_INSTRUMENTATION;
            mavlink_instrumentation_t inst_data;
            mavlink_msg_instrumentation_decode(mavlink_msg, &inst_data);
            message->payload.instrumentation.battery_current_cA = inst_data.battery_current;
            message->payload.instrumentation.motor_current_left_cA = inst_data.motor_current_left;
            message->payload.instrumentation.motor_current_right_cA = inst_data.motor_current_right;
            message->payload.instrumentation.mppt_current_cA = inst_data.mppt_current;
            message->payload.instrumentation.auxiliary_battery_current_cA = inst_data.auxiliary_battery_current;
            message->payload.instrumentation.battery_voltage_cV = inst_data.battery_voltage;
            message->payload.instrumentation.auxiliary_battery_voltage_cV = inst_data.auxiliary_battery_voltage;
            message->payload.instrumentation.irradiance = inst_data.irradiance;
            message->timestamp.epoch_seconds = inst_data.timestamp_seconds;
            message->timestamp.epoch_ms = inst_data.timestamp_milliseconds;
            break;
        }
        case MAVLINK_MSG_ID_TEMPERATURES: {
            message->source = DATA_SOURCE_TEMPERATURES;
            mavlink_temperatures_t temp_data;
            mavlink_msg_temperatures_decode(mavlink_msg, &temp_data);
            message->payload.temperature.battery_left_cdegC = temp_data.temperature_battery_left;
            message->payload.temperature.battery_right_cdegC = temp_data.temperature_battery_right;
            message->payload.temperature.mppt_left_cdegC = temp_data.temperature_mppt_left;
            message->payload.temperature.mppt_right_cdegC = temp_data.temperature_mppt_right;
            message->timestamp.epoch_seconds = temp_data.timestamp_seconds;
            message->timestamp.epoch_ms = temp_data.timestamp_milliseconds;
            break;
        }
        default:
            // If the MAVLink message ID is not recognized, the conversion fails.
            return false;
    }
    
    // Return true to indicate that the conversion was successful.
    return true;
}




