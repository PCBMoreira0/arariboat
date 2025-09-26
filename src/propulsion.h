#pragma once

enum PROPULSION_FUNC {
    LINEAR,
    EXP,
    LOG
};

void propulstion_set_function(PROPULSION_FUNC function);
void propulstion_set_dead_zone(float deadzone);
void propulstion_set_cut_zone(float cutzone);
void propulstion_set_angular_coef(float angular_coef);
uint8_t propulsion_get_state();