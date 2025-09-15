#pragma once

// Propulsion data structure
typedef struct {
    uint16_t backup_potentiometer_volts; // Volts from backup potentiometer
    uint16_t helm_potentiometer_volts; // Volts from helm potentiometer
    uint16_t throttle_left_potentiometer_volts; // Volts from left throttle potentiometer
    uint16_t throttle_right_potentiometer_volts; // Volts from right throttle potentiometer
    uint8_t state; // State of the propulsion system. 0=off, 1=on
} propulsion_data_t;