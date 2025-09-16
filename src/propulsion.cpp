#include <Arduino.h>
#include "Utilities.hpp"
#include "propulsion.h"
#include "queues.hpp" // Include the queues header to access the system queues
#include <iostream>   // to change dinamicaly value of motor minimum and angular coef

static const int bb_dac_pin = GPIO_NUM_25;
static const int be_dac_pin = GPIO_NUM_26;

static const int propulsion_voltage_unit = 1000; // mV

static const float pot_dir_max_voltage = 3.3f * propulsion_voltage_unit;   // 3300 mV
static const float pot_speed_max_voltage = 5.0f * propulsion_voltage_unit; // 5000 mV
static const int dac_max_bit = 256;
static const float dead_zone_threshold = 0.5f * propulsion_voltage_unit;

float min_motor_factor = 0.3f; // min velocity is 30%
float angular_coef = 0.5f;     // example

static uint8_t state = 0; // 0 - off, 1 - on
static PROPULSION_FUNC current_function = LINEAR;

void propulsion_set_angular_coef()
{
    std::cout << "Digite novo valor para min_motor_factor (ex: 0.3): ";
    std::cin >> min_motor_factor;

    std::cout << "Digite novo valor para angular_coef (ex: 0.5): ";
    std::cin >> angular_coef;

    std::cout << "Valores atualizados:\n";
    std::cout << "min_motor_factor = " << min_motor_factor << "\n";
    std::cout << "angular_coef = " << angular_coef << "\n";
}

// utility functions
static float map(float x, float in_min, float in_max, float out_min, float out_max)
{
    return (x - in_min) * (out_max - out_min) / (in_max - in_min) + out_min;
}

static double clamp(double v, double min, double max)
{
    return std::min(max, std::max(min, v));
}

// propulsion control functions
static float linear(float value)
{

    const float angular_coeficient = 1.0f;
    const float linear_coeficient = 0.0f;

    return angular_coeficient * value + linear_coeficient;
}

static float exponential(float value)
{
    const float base = 50.0f;
    return (base * value - 1) / (base - 1);
}

static float logarithmic(float value)
{
    return log10(9 * value + 1);
}

static float use_function(float value)
{
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

// sets and gets
void propulstion_set_function(PROPULSION_FUNC function)
{
    if (function == LINEAR)
        Serial.println("LINEAR");
    else if (function == EXP)
        Serial.println("EXP");
    else if (function == LOG)
        Serial.println("LOG");
    current_function = function;
}

uint8_t propulsion_get_state()
{
    return state;
}

static void get_velocity_in_bytes(uint8_t &bb_vel, uint8_t &be_vel, float bb_pot, float br_pot, float dir_pot, float angular_coef)
{

    float speed_porc = clamp((float)bb_pot / pot_speed_max_voltage, 0.0f, 1.0f);

    float dir_central_voltage = pot_dir_max_voltage / 2.0f;

    if ((dir_pot > (dir_central_voltage - dead_zone_threshold)) && (dir_pot < (dir_central_voltage + dead_zone_threshold)))
    {
        state = 0; // propulsion off

        bb_vel = 255 * speed_porc;
        be_vel = 255 * speed_porc;
        be_vel = 255 * speed_porc;
    }
    else
    {
        state = 1; // propulsion on

        if (dir_pot < (dir_central_voltage - dead_zone_threshold)) // bb changing
        {
            uint8_t bb_value = 255 * max(angular_coef * use_function(map(dir_pot, 0, dir_central_voltage - dead_zone_threshold, 0.0f, 1.0f)) + (1.0f - angular_coef), min_motor_factor) * speed_porc; // goes linearly from 1 to 0 but if it's lower than the min established it sends the min
            Serial.println(bb_value);
            bb_vel = bb_value * speed_porc;
            be_vel = 255 * speed_porc;
        }
        else // be changing
        {
            uint8_t be_value = 255 * max(angular_coef * use_function(map(dir_pot, pot_dir_max_voltage, dir_central_voltage + dead_zone_threshold, 0.0f, 1.0f)) + (1.0f - angular_coef), min_motor_factor) * speed_porc;
            Serial.println(be_value);
            bb_vel = 255 * speed_porc;
            be_vel = be_value * speed_porc;
        }
    }
}

void propulsion_task(void *parameter)
{
    Serial.println("\n[propulsion_task] Starting...");

    // Main loop for the propulsion task
    for (;;)
    {

        // Wait indefinitely for a message to arrive on the propulsion queue.
        message_t received_message;
        if (xQueueReceive(propulsion_queue, &received_message, portMAX_DELAY) == pdPASS)
        {
            // Process the received message
            // Serial.printf("[propulsion_task] Received message from source: %s\n", DATA_SOURCE_NAMES[received_message.source]);

            if (received_message.source != DATA_SOURCE_PROPULSION)
                continue;

            // Here you can add logic to control the propulsion system based on the received message
            // For example, if the message contains a command to start or stop the motors, handle it accordingly.
        }

        propulsion_data_t propulsion_data = received_message.payload.propulsion;

        float bb_pot = propulsion_data.backup_potentiometer_volts;
        float be_pot = propulsion_data.throttle_right_potentiometer_volts;
        float dir_pot = propulsion_data.helm_potentiometer_volts;

        uint8_t bb_vel, be_vel;
        get_velocity_in_bytes(bb_vel, be_vel, bb_pot, be_pot, dir_pot, angular_coef);

        dacWrite(bb_dac_pin, bb_vel);
        dacWrite(be_dac_pin, be_vel);

        // Serial.printf("\nBombordo: %.2f V\n"
        //                 "Direcao: %d\n"
        //               "Boreste: %.2f V\n"
        //               "Porc: %.2f\n",
        //               adc2voltage(bb_pot), dir_pot, adc2voltage(br_pot), speed_porc);

        vTaskDelay(pdMS_TO_TICKS(1));
    }
}