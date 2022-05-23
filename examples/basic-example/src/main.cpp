#include <Arduino.h>
#include <si5351.h>

// Si5351 correction value
// measure a 10 MHz signal from one of the clock outputs
// (in Hz, or better resolution if you can measure it),
// scale it to parts-per-billion, then use it in the set_correction()
// method in future use of this particular reference oscillator
uint32_t frequencyCorrection = 0;

void setup() {
  // initializes Si5351 on the standard ESP32 I2C pins (SDA: 21, SCL: 22)
  si5351_Init(frequencyCorrection);

  // 28 MHz @ ~7 dBm
  si5351_SetupCLK0(28000000, SI5351_DRIVE_STRENGTH_4MA);

  // 144 MHz @ ~7 dBm
  si5351_SetupCLK2(144000000, SI5351_DRIVE_STRENGTH_4MA);

  // Enable CLK0 and CLK2
  si5351_EnableOutputs((1<<0) | (1<<2));
}

void loop() {
  // put your main code here, to run repeatedly:
}