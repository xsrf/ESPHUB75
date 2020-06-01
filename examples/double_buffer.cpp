#include <Arduino.h>
#include <ESPHUB75.h>

#define WIDTH 128
#define HEIGHT 32

// Init Display: Width, Height, RowsPerMux, PrimaryColors, Pin Latch, Pins A-E
ESPHUB75 display(WIDTH, HEIGHT, ESPHUB75_SCAN_16, ESPHUB75_COLOR_RGB, ESPHUB75_LAT, ESPHUB75_A, ESPHUB75_B, ESPHUB75_C, ESPHUB75_D);


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