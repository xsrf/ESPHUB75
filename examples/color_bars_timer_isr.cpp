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

void drawColorBars();

// Execute ISR as often as possible to increase refresh rate
// Executing this even if LEDs are still lit is fine, because it returns immediately in this case!
ICACHE_RAM_ATTR void draw() {
  display.loop(); 
}

void setup() {
  display.begin(5,true); // Select from 1-5 Bits color depth
  display.setLEDPulseDuration(100); // 100µs LED on time for MSB
  display.clear();
  drawColorBars();
  display.requestBufferSwap();
  timer1_attachInterrupt(draw); // Add ISR Function
  timer1_enable(TIM_DIV16, TIM_EDGE, TIM_LOOP);
  timer1_write(5*25); // 5 ticks per µs, execute every 25µs
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

