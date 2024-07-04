#include "data.hpp" // Singleton class for storing system-data that needs to be accessed by multiple tasks.
#include "event_loop.hpp" //Event loop to handle events between tasks. This allows tasks to communicate with each other with loosely coupled code.
#include "logger.hpp" // Logger class for logging messages at a specified interval.
#include "blinker.hpp" // Blinker class for controlling the onboard LED.

//*********************************************************/
#define DEBUG // Uncomment to enable debug messages globally/
//*********************************************************/

#ifdef DEBUG
#define DEBUG_PRINTF(message, ...) Serial.printf(message, __VA_ARGS__)
#define DEBUG_PRINT(message, ...) Serial.print(message, __VA_ARGS__)
#else
#define DEBUG_PRINTF(message, ...)
#define DEBUG_PRINT(message, ...)
#endif


extern void LedBlinkerTask(void* parameter);
extern void WifiTask(void* parameter);
extern void ServerTask(void* parameter);
extern void SerialReaderTask(void* parameter);
extern void TemperatureReaderTask(void* parameter);
extern void GPSReaderTask(void* parameter);
extern void InstrumentationReaderTask(void* parameter);
extern void TimeReaderTask(void* parameter);

// Declare a handle for each task to allow manipulation of the task from other tasks, such as sending notifications, resuming or suspending.
// The handle is initialized to nullptr to avoid the task being created before the setup() function.
// Each handle is then assigned to the task created in the setup() function.

extern TaskHandle_t ledBlinkerHandle;
extern TaskHandle_t wifiTaskHandle;
extern TaskHandle_t serverTaskHandle;
extern TaskHandle_t serialReaderTaskHandle;
extern TaskHandle_t temperatureReaderTaskHandle;
extern TaskHandle_t gpsReaderTaskHandle;
extern TaskHandle_t instrumentationReaderTaskHandle;
extern TaskHandle_t timeReaderTaskHandle;

/// @brief Calibrates a reading by using a linear equation obtained by comparing the readings with a multimeter.
/// @return Calibrated reading
inline float LinearCorrection(const float input_value, const float slope, const float intercept) {
    return slope * input_value + intercept;
}

