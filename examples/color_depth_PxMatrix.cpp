/*
  This uses PxMatrix and is for comparison only!
  PxMatrix uses PxMATRIX_COLOR_DEPTH in steps, not bits!
  PxMATRIX_COLOR_DEPTH 8 is 3 Bits of color depth
  PxMATRIX_COLOR_DEPTH 16 is 4 Bits of color depth
  PxMATRIX_COLOR_DEPTH 32 is 5 Bits of color depth
*/

#include <Arduino.h>
#define PxMATRIX_COLOR_DEPTH 8
#include <PxMatrix.h>

#define P_LAT  16 // D0
#define P_A     5 // D1
#define P_B     4 // D2
#define P_C    15 // D8
#define P_D    12 // D6 (Only for 1/16 or 1/32 panels)
#define P_E     0 // D3 (Only for 1/32 panles)
#define P_OE    2 // D4 (Documentation only, this Pin is hardwired and cannot be changed!)
#define P_CLK  14 // D5 (Documentation only, this Pin is hardwired and cannot be changed!)
#define P_DATA 13 // D7 (Documentation only, this Pin is hardwired and cannot be changed!)

PxMATRIX display(64,32,P_LAT,P_OE,P_A,P_B,P_C,P_D);

void drawColorBars();

void setup() {
  display.begin(16);
  drawColorBars();
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
  static uint8_t loopcnt = 0;
  display.display(40);
  if(loopcnt++ == PxMATRIX_COLOR_DEPTH) {
    // To draw one complete frame, PxMatrix needs PxMATRIX_COLOR_DEPTH calls!
    // So we move the box only every PxMATRIX_COLOR_DEPTH iterations
    drawMovingBox();
    loopcnt = 0;
  }
}

