#include <Arduino.h> // Arduino framework library for ESP32
#include "event_loop.hpp"

esp_event_loop_handle_t eventLoop; //Handle for the event loop to post and subscribe to events

void InitializeEventLoop(esp_event_loop_handle_t* eventLoop) {

    //Create an event loop with the following parameters
    esp_event_loop_args_t eventLoopArgs = {
        .queue_size = 10,
        .task_name = "event_loop",
        .task_priority = 1,
        .task_stack_size = 8192,
        .task_core_id = 0
    };

    ESP_ERROR_CHECK(esp_event_loop_create(&eventLoopArgs, eventLoop));
}