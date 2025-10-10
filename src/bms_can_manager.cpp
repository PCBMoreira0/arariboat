#include "bms_can_manager.h"
#include <Arduino.h>
#include "esp_log.h"

// Example CAN IDs for BMS data (replace with actual IDs from your BMS protocol)
#define BMS_VOLTAGE_ID_POLL 0x18900140
#define BMS_CHARGE_DISCHARGE_ID_POLL 0x18930140
#define BMS_STATUS_ID_POLL 0x18940140
#define BMS_CELL_VOLTAGE_ID_POLL 0x18950140
#define BMS_TEMPERATURE_ID_POLL 0x18960140
#define BMS_FAILURE_ID_POLL 0x18980140

#define BMS_VOLTAGE_ID_RESPONSE 0x18904001
#define BMS_CHARGE_DISCHARGE_ID_RESPONSE 0x18934001
#define BMS_STATUS_ID_RESPONSE 0x18944001
#define BMS_CELL_VOLTAGE_ID_RESPONSE 0x18954001
#define BMS_TEMPERATURE_ID_RESPONSE 0x18964001
#define BMS_FAILURE_ID_RESPONSE 0x18984001


static const char* TAG_BMS = "BMS_CAN";

BMSCANManager::BMSCANManager() {
    initialize_can_handlers();
}   

void BMSCANManager::print_can_handlers() {
    char buffer[256] = {0};
    snprintf(buffer, sizeof(buffer), "[BMS] CAN Handlers: ");
    for (const auto& handler : _can_handlers) {
        snprintf(buffer + strlen(buffer), sizeof(buffer) - strlen(buffer), "0x%X ", handler.first);
    }
    // Serial.println(buffer);
}

bool BMSCANManager::handle_can_frame(const twai_message_t& message) {
    if (_can_handlers.find(message.identifier) != _can_handlers.end()) {
        _can_handlers[message.identifier](message);
        return true;
    }
    return false;
}

void BMSCANManager::initialize_can_handlers() {
    _can_handlers[BMS_VOLTAGE_ID_RESPONSE] = [this](const twai_message_t& message) {
        handle_voltage_response(message);
    };
    _can_handlers[BMS_CHARGE_DISCHARGE_ID_RESPONSE] = [this](const twai_message_t& message) {
        handle_charge_discharge_response(message);
    };
    _can_handlers[BMS_STATUS_ID_RESPONSE] = [this](const twai_message_t& message) {
        handle_status_response(message);
    };
    _can_handlers[BMS_CELL_VOLTAGE_ID_RESPONSE] = [this](const twai_message_t& message) {
        handle_cell_voltage_response(message);
    };
    _can_handlers[BMS_TEMPERATURE_ID_RESPONSE] = [this](const twai_message_t& message) {
        handle_temperature_response(message);
    };
    _can_handlers[BMS_FAILURE_ID_RESPONSE] = [this](const twai_message_t& message) {
        handle_failure_response(message);
    };
}

void BMSCANManager::set_data_queue(QueueHandle_t queue) {
    _data_queue = queue;
}

// Helper to check and publish
void BMSCANManager::check_and_publish() {
    if (_received_flags == ALL_FLAGS && _data_queue) {
        xQueueSend(_data_queue, &_bms_data, 0);
        _received_flags = 0; // Reset for next cycle
    }
}

void BMSCANManager::handle_voltage_response(const twai_message_t& message) {
    _bms_data.voltage_data.cumulative_voltage_decivolts = ((message.data[0] << 8) | message.data[1]);
    _bms_data.voltage_data.current_deciamps = ((message.data[4] << 8) | message.data[5]) - 30000;
    _bms_data.voltage_data.soc_decipercent = ((message.data[6] << 8) | message.data[7]);
    // // Serial.printf("[BMS] Voltage Data: Cumulative Voltage=%.1fV, Current=%.1fA, SOC=%.1f%%\n",
    //     _bms_data.voltage_data.cumulative_voltage_decivolts / 10.f, _bms_data.voltage_data.current_deciamps / 10.f, _bms_data.voltage_data.soc_decipercent / 10.f);
    // _received_flags |= FLAG_VOLTAGE;
    check_and_publish();
}

void BMSCANManager::handle_charge_discharge_response(const twai_message_t& message) {
    _bms_data.charge_discharge_status.state = message.data[0];
    _bms_data.charge_discharge_status.charge_mos = message.data[1];
    _bms_data.charge_discharge_status.discharge_mos = message.data[2];
    _bms_data.charge_discharge_status.bms_life_cycles = message.data[3];
    _bms_data.charge_discharge_status.remaining_capacity_raw = ((message.data[4] << 24) | (message.data[5] << 16) | (message.data[6] << 8) | message.data[7]);
    // // Serial.printf("[BMS] Charge/Discharge Status: State=%d, Charge MOS=%d, Discharge MOS=%d, BMS Life Cycles=%d, Remaining Capacity=%d mAh\n",
    //     _bms_data.charge_discharge_status.state, _bms_data.charge_discharge_status.charge_mos, _bms_data.charge_discharge_status.discharge_mos,
    //     _bms_data.charge_discharge_status.bms_life_cycles, _bms_data.charge_discharge_status.remaining_capacity_raw);
    // _received_flags |= FLAG_CHARGE_DISCHARGE;
    check_and_publish();
}

void BMSCANManager::handle_status_response(const twai_message_t& message) {
    _bms_data.bms_status.num_strings = message.data[0];
    _bms_data.bms_status.num_temp_sensors = message.data[1];
    _bms_data.bms_status.charger_status = message.data[2];
    _bms_data.bms_status.load_status = message.data[3];
    _bms_data.bms_status.io_bitfield = message.data[4];
    // // Serial.printf("[BMS] BMS Status: Num Strings=%d, Num Temp Sensors=%d, Charger Status=%d, Load Status=%d, IO Bitfield=0x%02X\n",
    //     _bms_data.bms_status.num_strings, _bms_data.bms_status.num_temp_sensors, _bms_data.bms_status.charger_status,
    //     _bms_data.bms_status.load_status, _bms_data.bms_status.io_bitfield);
    _received_flags |= FLAG_STATUS;
    check_and_publish();
}

void BMSCANManager::handle_cell_voltage_response(const twai_message_t& message) {
    uint8_t frame_index = message.data[0] - 1;
    constexpr uint8_t MAX_CELLS = 16; // Maximum number of cells
    //Each frame contains up to 3 cell voltages(2 bytes each)
    for (int i = 0; i < 3; i++) {
        int cell_id = i + frame_index * 3;
        if (cell_id < MAX_CELLS) {
            uint16_t cell_mv = ((message.data[1 + i * 2] << 8) | (message.data[2 + i * 2]));
            _bms_data.cell_voltage_frame[frame_index].voltages_mv[i] = cell_mv;
        }
    }

    // Serial.printf("[BMS] Cell Voltage Frame %d: [%d mV, %d mV, %d mV]\n",
    //     frame_index,
    //     _bms_data.cell_voltage_frame[frame_index].voltages_mv[0],
    //     _bms_data.cell_voltage_frame[frame_index].voltages_mv[1],
    //     _bms_data.cell_voltage_frame[frame_index].voltages_mv[2]);
    // _received_flags |= FLAG_CELL_VOLTAGE;
    check_and_publish();
}
    
void BMSCANManager::handle_temperature_response(const twai_message_t& message) {
    uint8_t frame_index = message.data[0] - 1;
    constexpr uint8_t MAX_TEMPS = 2; // Maximum number of temperature sensors
    //Each frame contains up to 7 temperatures(1 byte each)
    for (int i = 0; i < 7; i++) {
        int temp_id = i + frame_index * 7;
        if (temp_id < MAX_TEMPS) {
            int8_t temp_c = message.data[i + 1] - 40; // Offset +40°C
            _bms_data.temperature_frame[frame_index].raw_temps[i] = temp_c;
        }
    }

    // Serial.printf("[BMS] Temperature Frame %d: [%d °C, %d °C, %d °C, %d °C, %d °C, %d °C, %d °C]\n",
    //     frame_index,
    //     _bms_data.temperature_frame[frame_index].raw_temps[0],
    //     _bms_data.temperature_frame[frame_index].raw_temps[1],
    //     _bms_data.temperature_frame[frame_index].raw_temps[2],
    //     _bms_data.temperature_frame[frame_index].raw_temps[3],
    //     _bms_data.temperature_frame[frame_index].raw_temps[4],
    //     _bms_data.temperature_frame[frame_index].raw_temps[5],
    //     _bms_data.temperature_frame[frame_index].raw_temps[6]);
    _received_flags |= FLAG_TEMPERATURE;
    check_and_publish();
}

void BMSCANManager::handle_failure_response(const twai_message_t& message) {
    memcpy(_bms_data.failure_status.raw, message.data, sizeof(_bms_data.failure_status.raw));
    char buffer[128];
    snprintf(buffer, sizeof(buffer), "[BMS] Failure Status: [");
    for (int i = 0; i < sizeof(_bms_data.failure_status.raw); i++) {
        snprintf(buffer + strlen(buffer), sizeof(buffer) - strlen(buffer),
                "0x%02X", _bms_data.failure_status.raw[i]);
        if (i < sizeof(_bms_data.failure_status.raw) - 1) {
            snprintf(buffer + strlen(buffer), sizeof(buffer) - strlen(buffer), ", ");
        }
    }
    snprintf(buffer + strlen(buffer), sizeof(buffer) - strlen(buffer), "]\n");
    // Serial.print(buffer);
    _received_flags |= FLAG_FAILURE;
    check_and_publish();
}

void BMSCANManager::poll_bms_data() {
    // Send poll commands to the BMS for each data type
    // Serial.println("[BMS] Polling BMS data...");
    send_poll_command(BMS_VOLTAGE_ID_POLL);
    send_poll_command(BMS_CHARGE_DISCHARGE_ID_POLL);
    send_poll_command(BMS_STATUS_ID_POLL);
    send_poll_command(BMS_CELL_VOLTAGE_ID_POLL);
    send_poll_command(BMS_TEMPERATURE_ID_POLL);
    send_poll_command(BMS_FAILURE_ID_POLL);
}

void BMSCANManager::send_poll_command(uint32_t data_id) {
    twai_message_t message;
    message.extd = 1; // Extended frame format
    message.identifier = data_id;
    message.data_length_code = 8; // Assuming 8 bytes of data for the poll command, all zero
    memset(message.data, 0, sizeof(message.data));

    twai_transmit(&message, pdMS_TO_TICKS(10)); // Send the message with a timeout of 1 second
}