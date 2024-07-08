#include "data.hpp" // Singleton class for storing system-data that needs to be accessed by multiple tasks.
#include "event_loop.hpp" //Event loop to handle events between tasks. This allows tasks to communicate with each other with loosely coupled code.
#include "logger.hpp" // Logger class for logging messages at a specified interval.
#include "blinker.hpp" // Blinker class for controlling the onboard LED.
#include "Debug.hpp" // Debug macros for printing debug messages to the serial port.

//Preprocessor trick to convert a macro to a string
#define STRINGIFY(x) __STRINGIFY__(x)
#define __STRINGIFY__(x) #x

void DisplayScreenTask(void* parameter);
void LedBlinkerTask(void* parameter);
void WifiTask(void* parameter);
void ServerTask(void* parameter);
void SerialReaderTask(void* parameter);
void RadioTask(void* parameter);

// Declare a handle for each task to allow manipulation of the task from other tasks, such as sending notifications, resuming or suspending.
// The handle is initialized to nullptr to avoid the task being created before the setup() function.
// Each handle is then assigned to the task created in the setup() function.

extern TaskHandle_t ledBlinkerHandle;
extern TaskHandle_t displayScreenHandle;
extern TaskHandle_t wifiHandle;
extern TaskHandle_t serverHandle;
extern TaskHandle_t serialReaderHandle;
extern TaskHandle_t radioHandle;

/// @brief Calibrates a reading by using a linear equation obtained by comparing the readings with a multimeter.
/// @return Calibrated reading
inline float LinearCorrection(const float input_value, const float slope, const float intercept) {
    return slope * input_value + intercept;
}

