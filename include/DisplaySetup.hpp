/**
 * Header for display settings
 */

#ifndef DISPLAYSETUP_H
#define DISPLAYSETUP_H

#include <Arduino.h>
#include <TFT_eSPI.h>
#include <SPI.h>
#include <lvgl.h>
#include <../lib/DisplayUI/ui/ui.h>
#include <freertos/FreeRTOS.h>

extern SemaphoreHandle_t lv_mutex;

void DisplayInit();

#endif