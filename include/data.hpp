#include "arariboat/mavlink.h" // Custom mavlink dialect for the boat generated using Mavgen tool.

// Singleton class for storing system-data that needs to be accessed by multiple tasks.
class SystemData {

public:
    static SystemData& getInstance();

    mavlink_instrumentation_t instrumentation;
    mavlink_gps_info_t gps;
    mavlink_temperatures_t temperature;

    
private:
    SystemData(); // Private constructor to avoid multiple instances.
    SystemData(SystemData const&) = delete; // Delete copy constructor.
    SystemData& operator=(SystemData const&) = delete; // Delete assignment operator.
    SystemData(SystemData&&) = delete; // Delete move constructor.
};