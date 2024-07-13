//I2C pins to communicate with OLED display
#define OLED_ADDRESS    0x3C
#define OLED_SDA    21
#define OLED_SCL    22

// SPI pins to communicate with SX1276 LoRa modem's configuration registers and FIFO buffer
#define LORA_MOSI 27
#define LORA_MISO 19
#define LORA_CLK  5
#define LORA_CS  18
#define LORA_RST  23
#define LORA_DIO0 26 //OnTxDone and OnRxDone interrupt flag pins

// SPI pins for SD card
#define SDCARD_MOSI 15
#define SDCARD_MISO 2
#define SDCARD_SCLK 14
#define SDCARD_CS   13

