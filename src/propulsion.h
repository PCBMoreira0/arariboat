#pragma once

enum PROPULSION_FUNC {
    LINEAR,
    EXP,
    LOG
};

void propulstion_set_function(PROPULSION_FUNC function);
uint8_t propulsion_get_state();