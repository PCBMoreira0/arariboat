#include <Arduino.h> // Main Arduino library, required for projects that use the Arduino framework.
#include "Utilities.hpp" // Custom utility macros and functions.
#include "queues.hpp" // Header file for queue initialization and definitions.
#include "ESP32Time.h" // Internal RTC timer of ESP32 to keep track of time
#include "time_manager.h" // Header file for time management tasks.
#include "esp_console.h" // ESP-IDF console library for command line interface.
#include "parameter.h" // Header file for parameter management.
#include "commands.h" // Header file for command registration prototypes

//TODO: Improve server interface for configuration and debug purposes.
//TODO: Assign better weights to task priorities via benchmarks

// --- Global Objects (Single source of truth) ---
ESP32Time RTC;
EventGroupHandle_t system_event_group;


float test_parameter = 3.14f; // Example parameter to be registered

void setup() {
    Serial.begin(BAUD_RATE);
    initialize_queues(); // Initialize the queues used for inter-task communication.
    system_event_group = xEventGroupCreate(); // Create an event group for system-wide events.

    //Console backend initialization
    const esp_console_config_t console_config = {
      .max_cmdline_length = 256,
      .max_cmdline_args = 8,
    };
    ESP_ERROR_CHECK(esp_console_init(&console_config)); // Initialize the console with the specified configuration.
    ESP_ERROR_CHECK(esp_console_register_help_command()); // Register the help command to provide usage information.

    //Register commands for the console interface
    register_instrumentation_commands();
    register_led_commands();
    register_parameter_commands();

    //Register parameters before starting the parameter manager
    // This allows the parameter manager to load and save these parameters.

    parameter_manager.register_param("test_param", &test_parameter, 3.00); // Register a test parameter with the parameter manager.

    parameter_manager.begin(); // Initialize the parameter manager to load parameters from persistent storage.

      
    CREATE_TASK(led_manager_task, STACK_SIZE(2048), PRIORITY(1));
    // CREATE_TASK(uart_task, STACK_SIZE(4096), PRIORITY(1));
    CREATE_TASK(wifi_task, STACK_SIZE(4096), PRIORITY(2));
    CREATE_TASK(server_task, STACK_SIZE(8096), PRIORITY(3));
    CREATE_TASK(time_manager_task, STACK_SIZE(4096), PRIORITY(1));
    // CREATE_TASK(TemperatureTask, STACK_SIZE(4096), PRIORITY(1));
    // CREATE_TASK(GPSTask, STACK_SIZE(4096), PRIORITY(1));
    CREATE_TASK(instrumentation_task, STACK_SIZE(8192), PRIORITY(3));
    // CREATE_TASK(mppt_task, STACK_SIZE(4096), PRIORITY(3));
    CREATE_TASK(broker_task, STACK_SIZE(4096), PRIORITY(2));
    CREATE_TASK(can_task, STACK_SIZE(4096), PRIORITY(4)); // Create the CAN task to handle CAN communication.
    CREATE_TASK(propulsion_task, STACK_SIZE(4096), PRIORITY(5)); // Create the propulsion task to handle propulsion system.
}

void loop() {

    vTaskDelay(pdMS_TO_TICKS(60000));
    print_statistics(); // Print system statistics for debugging, performance and memory allocation analysis
}



