#pragma once
#include "led.hpp" // LED class for controlling the onboard LED.

//*********************************************************/
// #define DEBUG // Uncomment to enable debug messages globally/
//*********************************************************/

#ifdef DEBUG
#define DEBUG_PRINTF(...) Serial.printf(__VA_ARGS__)
#define DEBUG_PRINT(...) Serial.print(__VA_ARGS__)
#else
#define DEBUG_PRINTF(...)
#define DEBUG_PRINT(...)
#endif

//Preprocessor trick to convert a macro to a string
#define STRINGIFY(x) __STRINGIFY__(x)
#define __STRINGIFY__(x) #x

//Macro to compare two strings
#define STRINGS_ARE_EQUAL(a, b) strcmp(a, b) == 0


//Macro to synchronize the creation of tasks with their names and their handles
//Trivial macros to improve argument readability

#define CREATE_TASK_WITH_HANDLE(task_name, stack_size, priority) xTaskCreate(task_name, #task_name, stack_size, NULL, priority, &task_name##Handle)
#define CREATE_TASK(task_name, stack_size, priority) xTaskCreate(task_name, #task_name, stack_size, NULL, priority, NULL)

#define STACK_SIZE(x) (x) 
#define PRIORITY(x) (x)

/**
 * @brief Converts a buffer of bytes into a hexadecimal string representation (Pure C).
 *
 * This utility requires the caller to provide a destination buffer. It will
 * write the resulting null-terminated hex string into that buffer.
 *
 * @param dest A pointer to the character buffer where the hex string will be stored.
 * @param dest_size The total size of the destination buffer. Must be at least (src_len * 2 + 1).
 * @param src A pointer to the byte buffer to convert.
 * @param src_len The number of bytes in the source buffer to convert.
 * @return `true` on success, `false` on failure (e.g., destination buffer is too small).
 */
static inline bool bytes_to_hex_string(char* dest, size_t dest_size, const uint8_t* src, size_t src_len) {
    // Check for invalid arguments
    if (dest == NULL || src == NULL) {
        return false;
    }

    // Check if the destination buffer is large enough to hold the hex string
    // We need 2 characters for each source byte, plus 1 for the null terminator.
    if (dest_size < (src_len * 2 + 1)) {
        return false;
    }

    char* dest_ptr = dest;
    for (size_t i = 0; i < src_len; ++i) {
        // Format the byte as a two-digit, uppercase hexadecimal number.
        // `sprintf` could be used, but `snprintf` is safer as it prevents buffer overflows,
        // even though we've already checked the total size.
        int written = snprintf(dest_ptr, 3, "%02X", src[i]);
        
        // Move the pointer forward by the number of characters written (should always be 2)
        dest_ptr += written;
    }

    // Null-terminate the destination string.
    *dest_ptr = '\0';

    return true;
}

// --- Example Usage ---
// This demonstrates how you would use the C version of the function.
// You do not need to copy this part, just the function above.

// // Assume Serial.printf exists (as in Arduino/ESP-IDF)
// // If in pure C, you would use standard printf from <stdio.h>
// void mavlink_debug_example_c() {
//     // A sample MAVLink message buffer
//     uint8_t mav_buffer[] = {0xFE, 0x09, 0x55, 0x01, 0x01, 0x00, 0x00, 0x00, 0x00, 0x00, 0x04, 0x05, 0x03, 0x51, 0x04};
//     uint16_t mav_len = sizeof(mav_buffer);

//     // Create a destination buffer on the stack.
//     // Its size MUST be at least (mav_len * 2 + 1).
//     char hex_output_buffer[sizeof(mav_buffer) * 2 + 1];

//     // Call the conversion function
//     if (bytes_to_hex_string(hex_output_buffer, sizeof(hex_output_buffer), mav_buffer, mav_len)) {
//         // Print the result. No .c_str() is needed as it's already a char array.
//         printf("[MAV][DEBUG] Packet = %s\n", hex_output_buffer);
//     } else {
//         printf("[MAV][ERROR] Failed to convert packet to hex string.\n");
//     }
// }


void wifi_task(void* parameter);
void server_task(void* parameter);
void uart_task(void* parameter);
void TemperatureTask(void* parameter);
void GPSTask(void* parameter);
void instrumentation_task(void* parameter);
void time_manager_task(void* parameter);
void mppt_task(void *parameters);
void broker_task(void* parameter);
void led_manager_task(void* parameter);
void can_task(void* parameter);
void propulsion_task(void* parameter);


/// @brief Calibrates a reading by using a linear equation obtained by comparing the readings with a multimeter.
/// @return Calibrated reading
inline float LinearCorrection(const float input_value, const float slope, const float intercept) {
    return slope * input_value + intercept;
}

/// @brief Checks if a string ends with a newline character.
/// @param str The string to check.
/// @return true if the string ends with a newline character, false otherwise.
inline bool EndsWithNewline(const char* str) {
    if (str == nullptr) return false;
    size_t len = strlen(str);
    return len > 0 && str[len - 1] == '\n';
}

/**
 * @brief Prints a detailed report of system statistics to the Serial monitor.
 * * This function is designed to be thread-safe by formatting the entire
 * report into a single buffer before printing it. This prevents output
 * from being interleaved if other tasks are also writing to the Serial port.
 * * It gathers and displays information about:
 * - General device info (chip model, CPU freq, reset reason)
 * - Date, Time, and Location (hardcoded as an example)
 * - System uptime
 * - Memory Usage (Heap)
 * - Current Task's Stack Usage (High Water Mark)
 * * It is a valuable tool for debugging performance and memory issues.
 */
inline void print_statistics() {
  // Use a single, large buffer to format the entire string. This makes the
  // function safer to call from different tasks, as the entire block of text
  // will be sent to the Serial port in one go, preventing interleaved messages.
  char stats_buffer[1024];
  int buffer_offset = 0; // Tracks the current end of the string in the buffer

  // A. General System and Device Information
  // =================================================================
  buffer_offset += snprintf(stats_buffer + buffer_offset, sizeof(stats_buffer) - buffer_offset,
                           "--- SYSTEM STATISTICS ---\n");

  // A1. Chip Information
  esp_chip_info_t chip_info;
  esp_chip_info(&chip_info);
  buffer_offset += snprintf(stats_buffer + buffer_offset, sizeof(stats_buffer) - buffer_offset,
                           "Device Model        : %s rev %d\n", ESP.getChipModel(), chip_info.revision);
  buffer_offset += snprintf(stats_buffer + buffer_offset, sizeof(stats_buffer) - buffer_offset,
                           "Cores               : %d\n", chip_info.cores);
  buffer_offset += snprintf(stats_buffer + buffer_offset, sizeof(stats_buffer) - buffer_offset,
                           "CPU Frequency       : %d MHz\n", getCpuFrequencyMhz());

  // A2. Last Reset Reason
  esp_reset_reason_t reset_reason_code = esp_reset_reason();
  const char* reset_reason_str;
  switch (reset_reason_code) {
    case ESP_RST_UNKNOWN:    reset_reason_str = "Unknown"; break;
    case ESP_RST_POWERON:    reset_reason_str = "Power on"; break;
    case ESP_RST_EXT:        reset_reason_str = "External pin"; break;
    case ESP_RST_SW:         reset_reason_str = "Software"; break;
    case ESP_RST_PANIC:      reset_reason_str = "Panic / Exception"; break;
    case ESP_RST_INT_WDT:    reset_reason_str = "Interrupt Watchdog"; break;
    case ESP_RST_TASK_WDT:   reset_reason_str = "Task Watchdog"; break;
    case ESP_RST_WDT:        reset_reason_str = "Other Watchdog"; break;
    case ESP_RST_DEEPSLEEP:  reset_reason_str = "Exiting deep sleep"; break;
    case ESP_RST_BROWNOUT:   reset_reason_str = "Brownout"; break;
    case ESP_RST_SDIO:       reset_reason_str = "SDIO"; break;
    default:                 reset_reason_str = "N/A"; break;
  }
  buffer_offset += snprintf(stats_buffer + buffer_offset, sizeof(stats_buffer) - buffer_offset,
                           "Last Reset Reason   : %s\n", reset_reason_str);

  // A3. Time, Date, and Location
  buffer_offset += snprintf(stats_buffer + buffer_offset, sizeof(stats_buffer) - buffer_offset,
                           "--- Location & Time ---\n");
  buffer_offset += snprintf(stats_buffer + buffer_offset, sizeof(stats_buffer) - buffer_offset,
                           "Timestamp           : Thursday, June 19, 2025, 10:46 AM (-03)\n");
  buffer_offset += snprintf(stats_buffer + buffer_offset, sizeof(stats_buffer) - buffer_offset,
                           "Location            : Niterói, RJ, Brazil\n");


  // B. Uptime
  // =================================================================
  buffer_offset += snprintf(stats_buffer + buffer_offset, sizeof(stats_buffer) - buffer_offset,
                           "--- Uptime ---\n");
  uint32_t uptime_ms = millis();
  uint32_t seconds = uptime_ms / 1000;
  uint32_t minutes = seconds / 60;
  uint32_t hours = minutes / 60;
  uint32_t days = hours / 24;
  buffer_offset += snprintf(stats_buffer + buffer_offset, sizeof(stats_buffer) - buffer_offset,
                           "System Uptime       : %d days, %02d:%02d:%02d\n", days, hours % 24, minutes % 60, seconds % 60);


  // C. Memory Statistics
  // =================================================================
  buffer_offset += snprintf(stats_buffer + buffer_offset, sizeof(stats_buffer) - buffer_offset,
                           "--- Memory (Heap) ---\n");
  buffer_offset += snprintf(stats_buffer + buffer_offset, sizeof(stats_buffer) - buffer_offset,
                           "Total Heap Size     : %u bytes\n", ESP.getHeapSize());
  buffer_offset += snprintf(stats_buffer + buffer_offset, sizeof(stats_buffer) - buffer_offset,
                           "Free Heap Size      : %u bytes\n", ESP.getFreeHeap());
  buffer_offset += snprintf(stats_buffer + buffer_offset, sizeof(stats_buffer) - buffer_offset,
                           "Min Free Heap (LWM) : %u bytes\n", ESP.getMinFreeHeap());


  // D. Current Task Statistics
  // =================================================================
  buffer_offset += snprintf(stats_buffer + buffer_offset, sizeof(stats_buffer) - buffer_offset,
                           "--- Current Task ---\n");
  char* task_name = pcTaskGetName(NULL); // NULL gets the current task
  buffer_offset += snprintf(stats_buffer + buffer_offset, sizeof(stats_buffer) - buffer_offset,
                           "Task Name           : %s\n", task_name);
  
  // The "high water mark" for the stack. The minimum free stack space since the task started.
  // This is a critical metric for sizing a task's stack.
  uint32_t stack_hwm = uxTaskGetStackHighWaterMark(NULL); 
  buffer_offset += snprintf(stats_buffer + buffer_offset, sizeof(stats_buffer) - buffer_offset,
                           "Stack HWM           : %u bytes free\n", stack_hwm);
  
  buffer_offset += snprintf(stats_buffer + buffer_offset, sizeof(stats_buffer) - buffer_offset,
                           "--- END OF STATISTICS ---\n\n");

  // Print the entire formatted buffer in one atomic operation
  Serial.print(stats_buffer);
}