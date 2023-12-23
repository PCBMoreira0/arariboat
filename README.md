Main repository for the project. Each branch holds the code responsible for each code present in the boat.
There is a main ESP32 that gets I2C readings from an external ADC module, responsible for obtaining
the readings of the current and voltage sensors of the instrumentation board. Besides that,
it also reads the auxiliary battery  voltage through a voltage divider and its current via a shunt resistance.
Temperate measurements are taken using a standard DS18B20 probe array.

The main board then packets the data into Mavlink packets, optimized for binary transmission and low overhead,
and sends them to the Lora chip being controlled by another ESP32. It then receives the data and modulates it
using proprietary Chirp Spread Spectrum technology from Lora at 433/868/915MHz. A radio receiver on land captures
the data and sends it to the main interface running on a computer. 

Data can also be sent using the networking capabilities of the ESP32 chip, which is connected to the internet
via an access point provided by a 4G router on the boat, with a SIM card available for insertion. A virtual
private network (VPN) is used to allow the ESP32 inside the boat and any other device also connected within
this VPN to communicate with each other, as if they were on the same LAN network.

The main interface creates two background worker threads to listen to incoming data from the receiver radio
connected to a USB port or by network communication. This is done to prevent the UI thread from being blocked
while it waits for any data to show up.

Besides that, the main board also sends the data to another ESP32 present in the cockpit, which has a TFT display to show
system status to the pilot, as well as the measurements taken by the instrumentation  board.
