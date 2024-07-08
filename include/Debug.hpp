#pragma once
#include <Arduino.h>

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
