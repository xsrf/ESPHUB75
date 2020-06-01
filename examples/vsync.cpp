#include <Arduino.h>
#include <ESPHUB75.h>

#define WIDTH 128
#define HEIGHT 32

// Init Display: Width, Height, RowsPerMux, PrimaryColors, Pin Latch, Pins A-E
ESPHUB75 display(WIDTH, HEIGHT, ESPHUB75_SCAN_16, ESPHUB75_COLOR_RGB, ESPHUB75_LAT, ESPHUB75_A, ESPHUB75_B, ESPHUB75_C, ESPHUB75_D);


void setup() {
  display.begin(3,false,false); // ( ColorDepth in Bits per primary color, use DoubleBuffering )
}

// Select your preferred loop() at the bottom!

void loop1() {
  // This is the most basic sketch to show the display works.
  // Since doubleBuffering is not enabled, this will display Buffer "A" constantly, which is the only one available
  display.loop();
}

void drawMovingLine() {
    static uint8_t cnt = 0;
    cnt++;
    cnt %= display.width();
    display.clear(); // clears the whole buffer... 
    display.drawLine(cnt,0,(display.width()-1-cnt),(display.height()-1),0xFFFF);
}

void loop2() {
  // If you just draw stuff, you'll end up with tearing and strange artifacts
  display.loop();
  drawMovingLine();
}

void loop3() {
  // This is how you actually should work if you don't use doubleBuffering
  // readyForDrawing() will actually wait for VSync to be ready. You then have
  // to draw your frame at once
  display.loop();
  if(display.readyForDrawing()) {
    drawMovingLine();
  }
}

void loop4() {
  // If you're happy with 30fps or want a constant framerate (not refresh rate)
  display.loop();
  if(display.readyForFPS(30)) {
    drawMovingLine();
  }
}

void loop() {
  loop3();
}