#include <Arduino.h>
#include "Utilities.hpp"
#include "propulsion.h"
#include "queues.hpp" // Include the queues header to access the system queues

const int bb_dac_pin = GPIO_NUM_25;
const int br_dac_pin = GPIO_NUM_26;

const int pot_pin = GPIO_NUM_33;

const float pot_dir_max_voltage = 3.3f;
const float pot_speed_max_voltage = 5.0f;
const int dac_max_bit = 256;
const float dead_zone_threshold = 0.5f;

int num = 0;

PROPULSION_FUNC current_function = LINEAR;

float map(float x, float in_min, float in_max, float out_min, float out_max){
    return (x - in_min) * (out_max - out_min) / (in_max - in_min) + out_min;
}

double clamp(double v, double min, double max)
{
    return std::min(max, std::max(min, v));
}

float linear(float value){

	const float angular_coeficient = 1.0f;
	const float linear_coeficient = 0.0f;

	return angular_coeficient * value + linear_coeficient;
}

float exponential(float value){
    return value*value;
}

float logarithmic(float value){
    return log10(9 * value + 1);
}

float use_function(float value){
    switch (current_function)
    {
        case LINEAR:
            return linear(value);
        case EXP:
            return exponential(value);
        case LOG:
            return logarithmic(value); 
        default:
            return linear(value);   
    }
}

void propulstion_set_function(PROPULSION_FUNC function){
    if(function == LINEAR) Serial.println("LINEAR");
    else if(function == EXP) Serial.println("EXP");
    else if(function == LOG) Serial.println("LOG");
    current_function = function;
}

void get_velocity_in_bytes(uint8_t& bb_vel, uint8_t& br_vel, float bb_pot, float br_pot, float dir_pot){

    float speed_porc = clamp((float)bb_pot / pot_speed_max_voltage, 0.0f, 1.0f);

    float dir_central_voltage = pot_dir_max_voltage / 2.0f;

    if((dir_pot > (dir_central_voltage - dead_zone_threshold)) && (dir_pot < (dir_central_voltage + dead_zone_threshold))){	
        bb_vel = 255 * speed_porc;
        br_vel = 255 * speed_porc;
    }
    else if(dir_pot < (dir_central_voltage - dead_zone_threshold)){
        uint8_t bb_value = 255 * use_function(map(dir_pot, 0, dir_central_voltage - dead_zone_threshold, 0, pot_dir_max_voltage) / pot_dir_max_voltage);
        bb_vel = bb_value * speed_porc;
        br_vel = 255 * speed_porc;
    }
    else{
        uint8_t br_value = 255 * use_function(map(dir_pot, pot_dir_max_voltage, dir_central_voltage + dead_zone_threshold, 0, pot_dir_max_voltage) / pot_dir_max_voltage);
        bb_vel = 255 * speed_porc;
        br_vel = br_value * speed_porc;
    }
}

void propulsion_task(void* parameter) {
    Serial.println("\n[propulsion_task] Starting...");

    // Main loop for the propulsion task
    for (;;) {
        
        // Wait indefinitely for a message to arrive on the propulsion queue.
        message_t received_message;
        if (xQueueReceive(propulsion_queue, &received_message, portMAX_DELAY) == pdPASS) {
            // Process the received message
            // Serial.printf("[propulsion_task] Received message from source: %s\n", DATA_SOURCE_NAMES[received_message.source]);
            
            if(received_message.source != DATA_SOURCE_PROPULSION) continue;
            

            // Here you can add logic to control the propulsion system based on the received message
            // For example, if the message contains a command to start or stop the motors, handle it accordingly.
        }


        propulsion_data_t propulsion_data = received_message.payload.propulsion;

        // float bb_pot = propulsion_data.backup_potentiometer_volts;
        float bb_pot = 5.0f;
        float br_pot = propulsion_data.throttle_right_potentiometer_volts;
        float dir_pot = propulsion_data.helm_potentiometer_volts;

        // Serial.println(dir_pot);

        uint8_t bb_vel, br_vel;
        get_velocity_in_bytes(bb_vel, br_vel, bb_pot, br_pot, dir_pot);

        dacWrite(bb_dac_pin, bb_vel);
        dacWrite(br_dac_pin, br_vel);
        
        // Serial.printf("\nBombordo: %.2f V\n"
        //                 "Direcao: %d\n"
        //               "Boreste: %.2f V\n"
        //               "Porc: %.2f\n",
        //               adc2voltage(bb_pot), dir_pot, adc2voltage(br_pot), speed_porc);

        vTaskDelay(pdMS_TO_TICKS(1));
    }
}