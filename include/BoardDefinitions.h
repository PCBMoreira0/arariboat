#include <Arduino.h>

#ifdef SCREEN_ENABLED
#include <Wire.h> 
#include "SSD1306Wire.h" 

// TFT display with I2C interface
#define OLED_CLASS_OBJ  SSD1306Wire
#define OLED_ADDRESS    0x3C
#define OLED_SDA    21
#define OLED_SCL    22
#define OLED_RST    -1

#endif

//LORA_V1_6_OLED + SDCard + SX1276 LoRa
#define LORA_V1_6_OLED  1
#define LORA_PERIOD 915     
#define SYNC_WORD 0xFE

// Pins for SX1276 LoRa modem
#define CONFIG_MOSI 27
#define CONFIG_MISO 19
#define CONFIG_CLK  5
#define CONFIG_NSS  18
#define CONFIG_RST  23
#define CONFIG_DIO0 26 //OnTxDone and OnRxDone interrupt flag pins

// Pins for SD card
#define SDCARD_MOSI 15
#define SDCARD_MISO 2
#define SDCARD_SCLK 14
#define SDCARD_CS   13

#if LORA_PERIOD == 433
#define BAND 433E6
#elif LORA_PERIOD == 868
#define BAND 868E6
#elif LORA_PERIOD == 915
#define BAND 915E6
#else
#error "Please select the correct LoRa frequency"
#endif
