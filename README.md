ESPHUB75 Matrix Panel Driver
============================
This started as a weekend project of mine, trying to run my 64x32 RGB LED Matrix with an ESP8266.
I've now added support for ESP32 too.

If you're looking for a library that actually also works with your Panel,
you'd better have a loook at [PxMatrix](https://github.com/2dom/PxMatrix/) by 2dom.

But why?
--------

Why a new library if PxMatrix works? Well, I've seen that PxMatrix has some [performance drawbacks](https://twitter.com/atc1441/status/1248583908305838082) as shown by [@atc1441](https://twitter.com/atc1441).
I didn't like the fact that PxMatrix is actually blocking the CPU for SPI transfer and also while enabling the LEDs, so I tried to do this in a non-blocking way using the ESP8266s hardware peripherials.

Sending SPI data without blocking the CPU is possible, but only up to 64 Bytes at once. However, this is enough for my display - and any display with maximum of 170 RGB LEDs lit at once during multiplexing. I later found out how to send even more data in chunks of 64 bytes using SPI Interrups and DAM on the ESP32.

Creating a precisely timed pulse to enable the LEDs without blocking CPU is also possible using UART1, which is conveniently tied to GPIO 02 / D4 on the ESP8266 which everybody is using as OE Pin anyways - like on [@witnessmenow](https://twitter.com/witnessmenow)s [D1 Mini Matrix Shield](https://www.tindie.com/products/brianlough/d1-mini-matrix-shield/).

By using this approach, you basically don't gain much performance regarding the plain refresh rate. But you gain more time for your CPU to do other stuff.
On my display, PxMatrix spends about 75µs on CPU to draw one line. This library only takes 25µs. PxMatrix achieves a refresh rate of ~13kHz with 100% CPU blocked for drawing, whereas my library achieves ~16kHz with only 50% CPU load, leaving 50% for your code to actually create graphics or the ESP to do Wifi stuff.

To really benefit from this, the display code should run in an ISR with ~10kHz or more. This is possible using hardware timers on the ESP.

Color Depth
-----------
PxMatrix, if I understand the code right, creates color depth by using shades of each primary color. By default 8 shades per primary color mean that a pixel at its darkes shade is only lit every 8 frames drawn to the display. This is basic PWM princible.
But that imploes, for N shades, you have to allocate N times as many memory and you have to draw a frame N times to be complete. This doesn't scale very well.

I went with a different apploach here... To create 8 shades per primary color, I created only 3 different buffers. The 8 shades are stored in 3 bits binary representation. The first buffer holds the most significant bit of all pixels colors, the second one holds the middle bit and the third one holds the least significant bit.
These three buffers are displayed using different on durations for the LEDs. e.g. the first one is displayed with 40µs LEDs on, the second one with 20µs and the third one with 10µs.
This scales way better, requiring only ld(N) times the memory space and only ld(n) drop in framerate.

Using 5 Bit color depth / 32 shades per color / 512 colors in total is no problem with ESPHUB75, but impossible for PxMatrix.

Examples
--------
Feel free to try the examples provided. It is also based on Adafruit GFX, so most stuff should just work.

Wiring
------

Basically the same as for the Matrix Shield above

HUB75     | ESP8266 | ESP32 | WeMos D1 Mini R2 | ESP Function
----------|---------|-------|------------------|-------------
R0 / DIN  | 13*     | 13    | D7*              | HSPI Data
SCK / CLK | 14*     | 14    | D5*              | HSPI Clock
OE        | 2*      | 2     | D4*              | UART1 TX
LAT / STB | 16      | 22    | D0               | 
A         | 5       | 19    | D1               | 
B         | 4       | 23    | D2               | 
C         | 15      | 18    | D8               | 
D         | 12      | 5     | D6               | 
E         | 0       | 15    | D3               | 

*) PIN cannot be changed due to hardware limitations of the ESP8266

