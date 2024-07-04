#include "LoraConfigManager.hpp"
#include "Preferences.h" 
#include "event_loop.hpp"
Preferences flashMemory;

#define NVS_NAMESPACE "flash"
#define RO_MODE true
#define RW_MODE false
#define VALUE_IF_NOT_FOUND -1

#define LORA_FREQUENCY_KEY "frequency"
#define LORA_BANDWIDTH_KEY "bandwidth"
#define LORA_SPREADING_FACTOR_KEY "spreadingFactor"
#define LORA_CODING_RATE_KEY "codingRate"
#define LORA_SYNC_WORD_KEY "syncWord"
#define LORA_POWER_KEY "power"
#define LORA_CRC_ENABLED_KEY "CRCEnabled"

bool ValidateLoraConfiguration(const LoraConfiguration& config) {

    if (config.frequency != 433000000 && config.frequency != 868000000 && config.frequency != 915000000) {
        Serial.printf("[FLASH]Invalid LoRa frequency: %ld\n", config.frequency);
        return false;
    }

    if (config.bandwidth != 125000 && config.bandwidth != 250000 && config.bandwidth != 500000) {
        Serial.printf("[FLASH]Invalid LoRa bandwidth: %ld\n", config.bandwidth);
        return false;
    }

    if (config.spreadingFactor < 6 || config.spreadingFactor > 12) {
        Serial.printf("[FLASH]Invalid LoRa spreading factor: %d\n", config.spreadingFactor);
        return false;
    }

    if (config.codingRate < 5 || config.codingRate > 8) {
        Serial.printf("[FLASH]Invalid LoRa coding rate: %d\n", config.codingRate);
        return false;
    }

    if (config.syncWord < 0 || config.syncWord > 0xFF) {
        Serial.printf("[FLASH]Invalid LoRa sync word: %d\n", config.syncWord);
        return false;
    }

    if (config.power < 2 || config.power > 20) {
        Serial.printf("[FLASH]Invalid LoRa power: %d\n", config.power);
        return false;
    }

    if (config.crcEnabled != 0 && config.crcEnabled != 1) {
        Serial.printf("[FLASH]Invalid LoRa CRC enabled: %d\n", config.crcEnabled);
        return false;
    }

    return true;
}

bool TrySaveLoraConfiguration(const LoraConfiguration& config) {

    if (!ValidateLoraConfiguration(config)) {
        Serial.println("[FLASH]Invalid Lora configuration.");
        return false;
    }

    flashMemory.begin(NVS_NAMESPACE, RW_MODE);
    size_t success = true;
    success &= flashMemory.putLong(LORA_FREQUENCY_KEY, config.frequency);
    success &= flashMemory.putLong(LORA_BANDWIDTH_KEY, config.bandwidth);
    success &= flashMemory.putInt(LORA_SPREADING_FACTOR_KEY, config.spreadingFactor);
    success &= flashMemory.putInt(LORA_CODING_RATE_KEY, config.codingRate);
    success &= flashMemory.putInt(LORA_SYNC_WORD_KEY, config.syncWord);
    success &= flashMemory.putInt(LORA_POWER_KEY, config.power);
    success &= flashMemory.putBool(LORA_CRC_ENABLED_KEY, config.crcEnabled);
    flashMemory.end();
    return success;
}

bool GetLoraConfiguration(LoraConfiguration& config) {

    flashMemory.begin(NVS_NAMESPACE, RO_MODE);
    config.frequency = flashMemory.getLong(LORA_FREQUENCY_KEY, VALUE_IF_NOT_FOUND);
    config.bandwidth = flashMemory.getLong(LORA_BANDWIDTH_KEY, VALUE_IF_NOT_FOUND);
    config.spreadingFactor = flashMemory.getInt(LORA_SPREADING_FACTOR_KEY, VALUE_IF_NOT_FOUND);
    config.codingRate = flashMemory.getInt(LORA_CODING_RATE_KEY, VALUE_IF_NOT_FOUND);
    config.syncWord = flashMemory.getInt(LORA_SYNC_WORD_KEY, VALUE_IF_NOT_FOUND);
    config.power = flashMemory.getInt(LORA_POWER_KEY, VALUE_IF_NOT_FOUND);
    config.crcEnabled = flashMemory.getBool(LORA_CRC_ENABLED_KEY, VALUE_IF_NOT_FOUND);
    flashMemory.end();

    return ValidateLoraConfiguration(config);
}

// NVS namespace must be opened in read-write mode before calling this function.
bool ConfigureRadioDefaults(Preferences& flashMemory) {

    #if (DEFAULT_LORA_FREQUENCY != 433000000 && DEFAULT_LORA_FREQUENCY && 868000000 && DEFAULT_LORA_FREQUENCY != 915000000)
    #error "Invalid default LoRa frequency. Must be 433000000, 868000000, or 915000000."
    #endif

    #if (DEFAULT_LORA_BANDWIDTH != 125000 && DEFAULT_LORA_BANDWIDTH != 250000 && DEFAULT_LORA_BANDWIDTH != 500000)
    #error "Invalid default LoRa bandwidth. Must be 125000, 250000, or 500000."
    #endif

    #if (DEFAULT_LORA_SPREADING_FACTOR < 6 || DEFAULT_LORA_SPREADING_FACTOR > 12)
    #error "Invalid default Lora spreading factor. Must be between 6 and 12."
    #endif

    #if (DEFAULT_LORA_CODING_RATE < 5 || DEFAULT_LORA_CODING_RATE > 8)
    #error "Invalid default Lora coding rate. Must be between 5 and 8."
    #endif

    #if (DEFAULT_LORA_SYNC_WORD < 0 || DEFAULT_LORA_SYNC_WORD > 0xFF)
    #error "Invalid default Lora sync word. Must be between 0 and 0xFF."
    #endif

    #if (DEFAULT_LORA_POWER < 2 || DEFAULT_LORA_POWER > 20)
    #error "Invalid default Lora power. Must be between 2 and 20."
    #endif

    #if DEFAULT_LORA_CRC_ENABLED != 0 && DEFAULT_LORA_CRC_ENABLED != 1
    #error "Invalid default Lora CRC enabled. Must be 0 or 1."
    #endif

    auto defaultSettings = LoraConfiguration {
        .frequency = DEFAULT_LORA_FREQUENCY,
        .bandwidth = DEFAULT_LORA_BANDWIDTH,
        .spreadingFactor = DEFAULT_LORA_SPREADING_FACTOR,
        .codingRate = DEFAULT_LORA_CODING_RATE,
        .syncWord = DEFAULT_LORA_SYNC_WORD,
        .power = DEFAULT_LORA_POWER,
        .crcEnabled = DEFAULT_LORA_CRC_ENABLED
    };

    if (!TrySaveLoraConfiguration(defaultSettings)) {
        Serial.println("[FLASH]Failed to save default Lora configuration to flash memory.");
        return false;
    }

    return true;
}

static bool checkIfInitialized(Preferences& flashMemory) {
    flashMemory.begin(NVS_NAMESPACE, RO_MODE);
    bool hasInitializedBefore = flashMemory.isKey("initialized");
    flashMemory.end();
    return hasInitializedBefore;
}

static bool markInitialized(Preferences& flashMemory) {
    flashMemory.begin(NVS_NAMESPACE, RW_MODE);
    bool success = flashMemory.putBool("initialized", true);
    flashMemory.end();
    return success;
}

void InitializeFlashMemory() {
    
    if (checkIfInitialized(flashMemory)) return;

    vTaskDelay(pdMS_TO_TICKS(10000));
    char message[] = "blinkfast";
    esp_event_post_to(eventLoop, SERIAL_PARSER_EVENT_BASE, 0, message, strlen(message) + 1, portMAX_DELAY);
    
    Serial.printf("[FLASH]Initializing flash memory default values...\n");
    if (!ConfigureRadioDefaults(flashMemory)) {
        Serial.println("[FLASH]Failed to configure radio defaults.");
        return;
    }

    markInitialized(flashMemory);

    sprintf(message, "blinkslow");
    Serial.printf("[FLASH]Flash memory initialized.\n");
    esp_event_post_to(eventLoop, SERIAL_PARSER_EVENT_BASE, 0, message, strlen(message) + 1, portMAX_DELAY);
    
}