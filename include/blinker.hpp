#ifndef BLINKER_HPP
#define BLINKER_HPP
#include <Arduino.h>

struct BlinkCommand {
    long offTime;
    long onTime;
    int repeats;
};

extern QueueHandle_t blinkerQueue;
extern void Blink(int, int, int);
void BlinkerTask(void* ledPin);
#endif