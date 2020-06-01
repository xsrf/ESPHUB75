#include <Arduino.h>
#include <ESPHUB75.h>

#define WIDTH 128
#define HEIGHT 32

// Init Display: Width, Height, RowsPerMux, PrimaryColors, Pin Latch, Pins A-E
ESPHUB75 display(WIDTH, HEIGHT, ESPHUB75_SCAN_16, ESPHUB75_COLOR_RGB, ESPHUB75_LAT, ESPHUB75_A, ESPHUB75_B, ESPHUB75_C, ESPHUB75_D);

void drawColorBars();

void setup() {
  display.begin(5); // Select from 1-5 Bits color depth
  display.setLEDPulseDuration(100); // 100µs LED on time for MSB
  display.clear();
  drawColorBars();
  display.requestBufferSwap();
}

void drawColorBars() {
  uint8_t y_step = display.height()/6;
  for(uint8_t i=0; i<display.width(); i++) {
    uint8_t c; 
    c = map(i,0,display.width()-1,0,31);
    display.drawLine(i,y_step*0,i,(y_step*1)-1,c<<11);
    display.drawLine(i,y_step*1,i,(y_step*2)-1,c<<6);
    display.drawLine(i,y_step*2,i,(y_step*3)-1,c);
    display.drawLine(i,y_step*3,i,(y_step*4)-1,c|(c<<6)|(c<<11));
    uint8_t r,g,b = 0;
    if(i<display.width()/2) {
      r = map(i,0,(display.width()/2)-1,31,0);
      g = map(i,0,(display.width()/2)-1,0,31);
      b = 0;
    } else {
      r = 0;
      g = map(i,display.width()/2,display.width()-1,31,0);
      b = map(i,display.width()/2,display.width()-1,0,31);
    }
    display.drawLine(i,y_step*4,i,(y_step*5)-1,b|(g<<6)|(r<<11));
  }  
}

void drawMovingBox() {
  uint8_t y_off = (display.height()/6)*5;
  static uint8_t cnt = 0;
  display.fillRect(cnt,y_off,display.height()-y_off,display.height()-y_off,0x0000);
  cnt++;
  cnt %= display.width()-(display.height()-y_off)+1;
  display.fillRect(cnt,y_off,display.height()-y_off,display.height()-y_off,0xFFFF);
}

void loop() {
  if(display.readyForDrawing()) {
    display.copyBuffer(); // Copy display buffer to drawing buffer, because our drawing functions relies on the buffer contents
    drawMovingBox();
    display.requestBufferSwap();
  }
}

