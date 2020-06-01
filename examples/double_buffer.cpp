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
  display.begin(3,true); // ( ColorDepth in Bits per primary color, use DoubleBuffering )
}

// Select your preferred loop() at the bottom!

void loop1() {
  // This is the most basic sketch to show the display works.
  // Since doubleBuffering is enabled, this will display Buffer "B" constantly
  display.loop();
}

void loop2() {
  // This time we will request a BufferSwap, which will take place every time a frame is dsiplayed
  // You'll see Buffer "A" and "B" switching extremely fast
  display.loop();
  display.requestBufferSwap();
}

void drawMovingLine() {
    static uint8_t cnt = 0;
    cnt++;
    cnt %= display.width();
    display.clear(); // clears the whole buffer... 
    display.drawLine(cnt,0,(display.width()-1-cnt),(display.height()-1),0xFFFF);
}

void loop3() {
  // This is how you actually should work if you use doubleBuffering
  // When you've drawn you frame, you have to requestBufferSwap(), which will take place after the current
  // frame displayed actually finished. Until the buffer actually swapped, you must not draw to the Buffer!
  // This is why you have to wait until the drawing buffer is ready again!
  display.loop();
  if(display.readyForDrawing()) {
    drawMovingLine();
    display.requestBufferSwap();
  }
  display.loop();
}

void loop4() {
  // If you don't wait for drawing, you'll get tearing or strange artifacts...
  display.loop();
  drawMovingLine();
  display.requestBufferSwap();
}

void loop5() {
  // If you're happy with 30fps or want a constant framerate (not refresh rate)
  display.loop();
  if(display.readyForFPS(30)) {
    drawMovingLine();
    display.requestBufferSwap();
  }
}

void loop() {
  loop5();
}