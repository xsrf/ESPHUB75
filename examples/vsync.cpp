#include <Arduino.h>
#include <ESPHUB75.h>

#define P_LAT  16 // D0
#define P_A     5 // D1
#define P_B     4 // D2
#define P_C    15 // D8
#define P_D    12 // D6 (Only for 1/16 or 1/32 panels)
#define P_E     0 // D3 (Only for 1/32 panles)
#define P_OE    2 // D4 (Documentation only, this Pin is hardwired and cannot be changed!)
#define P_CLK  14 // D5 (Documentation only, this Pin is hardwired and cannot be changed!)
#define P_DATA 13 // D7 (Documentation only, this Pin is hardwired and cannot be changed!)

// Init Display: Width, Height, RowsPerMux, PrimaryColors, Pin Latch, Pins A-E
ESPHUB75 display(64,32,2,3,P_LAT,P_A,P_B,P_C,P_D);

void setup() {
  display.begin(3,false); // ( ColorDepth in Bits per primary color, use DoubleBuffering )
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