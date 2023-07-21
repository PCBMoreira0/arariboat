#include "arariboat/mavlink.h"

/// @brief Singleton class to hold system data.
/// @details This class is a singleton, meaning that there can only be one instance of it.
/// It should hold mavlink messages that are exchanged between the different parts of the system and the ground station.
class SystemData {

public:
    static SystemData& getInstance() {
        static SystemData instance;
        return instance;
    }

    enum debug_print_flags : uint16_t {
        None = 0b0000000000,
        Wifi = 0b0000000001,
        Server = 0b0000000010,
        Vpn = 0b0000000100,
        Serial = 0b0000001000,
        Temperature = 0b0000010000,
        Gps = 0b0000100000,
        Instrumentation = 0b0001000000,
        Auxiliary = 0b0010000000,
        Encoder = 0b0100000000,
        High_water = 0b1000000000,
        All = 0b1111111111
    };

    debug_print_flags debug_print = debug_print_flags::All;
    mavlink_instrumentation_t instrumentationSystem;
    mavlink_gps_info_t gpsSystem;
    mavlink_temperatures_t temperatureSystem;
    mavlink_aux_system_t auxiliarySystem;
    mavlink_control_system_t controlSystem;

    
private:
    SystemData() { // Private constructor to avoid multiple instances.
        instrumentationSystem = { 0 };
        gpsSystem = { 0 };
        temperatureSystem = { 0 };
        auxiliarySystem = { 0 };
        controlSystem = { 0 };
    }
    SystemData(SystemData const&) = delete; // Delete copy constructor.
    SystemData& operator=(SystemData const&) = delete; // Delete assignment operator.
    SystemData(SystemData&&) = delete; // Delete move constructor.
};

SystemData& systemData = SystemData::getInstance(); // Get a reference to the singleton instance to make accessing it easier.