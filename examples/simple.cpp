#include <Arduino.h>
#include <ESPHUB75.h>

#define WIDTH 128
#define HEIGHT 32

// Init Display: Width, Height, RowsPerMux, PrimaryColors, Pin Latch, Pins A-E
ESPHUB75 display(WIDTH, HEIGHT, ESPHUB75_SCAN_16, ESPHUB75_COLOR_RGB, ESPHUB75_LAT, ESPHUB75_A, ESPHUB75_B, ESPHUB75_C, ESPHUB75_D);

void setup() {
  display.begin(3); // Select from 1-5 Bits color depth
  display.setTextColor(0xFFFF);
  display.setTextSize(4);
}

void loop() {
  static uint16_t cnt = 0;
  if(display.readyForDrawing()) {
    display.copyBuffer();
    display.clear();
    display.setCursor(0,0);
    display.print(cnt++);
    display.requestBufferSwap();
  }
}

