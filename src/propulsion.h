#pragma once

enum PROPULSION_FUNC {
    LINEAR,
    EXP,
    LOG
};

void propulstion_set_function(PROPULSION_FUNC function);