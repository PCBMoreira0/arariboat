#include <Arduino.h>
#include "Utilities.hpp"

constexpr int frequencyPinLeft = GPIO_NUM_12;
constexpr int frequencyPinRight = GPIO_NUM_14;

volatile unsigned long lastMeasureLeft = 0;
volatile unsigned long periodSumLeft = 0;
volatile unsigned int counterLeft = 0;

volatile unsigned long lastMeasureRight = 0;
volatile unsigned long periodSumRight = 0;
volatile unsigned int counterRight = 0;


int block = 0;

static float PeriodToFrequency(float value) {
	return value == 0 ?  0 : 1000000.0 / value;	
}

static void ResetValues() {
	block = 1;
	periodSumLeft = 0;
	periodSumRight = 0;
	counterLeft = 0;
	counterRight = 0;
	block = 0;
}

static void handleFrequencyInterruptLeft() 
{
    if (block) return;
	
    unsigned long current = micros();
    periodSumLeft += current - lastMeasureLeft;
    lastMeasureLeft = current;
    counterLeft++;
}

static void handleFrequencyInterruptRight() {

	if (block) return;
    
    unsigned long current = micros();
    periodSumRight += current - lastMeasureRight;
    lastMeasureRight = current;
    counterRight++;	
}

static float FrequencyToRPM(float frequency) {
    constexpr float conversion_factor_slope = 43.0; // Conversion factor from frequency to RPM
    constexpr float conversion_factor_offset = 0.0; // Offset for the conversion factor

    return LinearCorrection(frequency, conversion_factor_slope, conversion_factor_offset);
}

static void UpdateSystemData(float rpm_left_motor, float rpm_right_motor) {

    SystemData::getInstance().all_info.rpm_left = rpm_left_motor;
    SystemData::getInstance().all_info.rpm_right = rpm_right_motor;
} 

static float CalculatePeriod(unsigned long periodSum, unsigned int counter) {
    return counter == 0 ? 0 : (float)periodSum / counter;
}

static void HandleReadings() {
    
    float periodLeft = CalculatePeriod(periodSumLeft, counterLeft);
    float periodRight = CalculatePeriod(periodSumRight, counterRight);

    float frequencyLeft = PeriodToFrequency(periodLeft);
    float frequencyRight = PeriodToFrequency(periodRight);

    float rpmLeftMotor = FrequencyToRPM(frequencyLeft);
    float rpmRightMotor = FrequencyToRPM(frequencyRight);

    UpdateSystemData(rpmLeftMotor, rpmRightMotor);

    DEBUG_PRINTF("\nFrequency L: %.1f\tRPM: %.0f\nFrequency R: %.1f\tRPM: %.0f\n", frequencyLeft, rpmLeftMotor, frequencyRight, rpmRightMotor);

    ResetValues();
}

void FrequencyCounterTask(void* parameter) {
    
    pinMode(frequencyPinLeft, INPUT);
    pinMode(frequencyPinRight, INPUT);

    attachInterrupt(digitalPinToInterrupt(frequencyPinLeft), handleFrequencyInterruptLeft, RISING);
    attachInterrupt(digitalPinToInterrupt(frequencyPinRight), handleFrequencyInterruptRight, RISING);

    constexpr int printInterval = 1250;
    unsigned long lastTime = 0;

    while (true) {
        
        if (millis() - lastTime >= printInterval) {
            lastTime = millis();
            HandleReadings();
        }
        vTaskDelay(pdMS_TO_TICKS(100));
    }
}
