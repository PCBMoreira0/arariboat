#include "data.hpp"

SystemData::SystemData() {
    instrumentation = {};
    gps = {};
    temperature = {};
    controlSystem = {};
}

SystemData& SystemData::getInstance() {
    static SystemData instance;
    return instance;
}