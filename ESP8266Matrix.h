#pragma once

/*
    https://github.com/xsrf/ESP8266Matrix

    This Library is designed to drive Shift-Register based multiplexing
    LED Matrix panles with as little CPU load as possible by using
    hardware peripherials of the ESP8266 (only at the moment).

    The UART0 hardware is used to generate an acurate pulse for
    OE / enabling the LEDs.
    This means that OE MUST be connected to D4 / IO02.

    Data transfer is handled by SPI1 hardware, but we're not using the
    official SPI lib, because it only uses one Byte of the hardware
    SPI buffer and is also blocking while sending data.
    The hardware SPI1 buffer is is actually 16 Words / 64 Bytes big.
    We'll copy the data into the registers SPI1W0 to SPI1W15 directly.
    This allows us to send 64 Bytes of data without blocking the CPU.
    At the moment, 64 Bytes is the maximum, so this limits the panels
    supported. 64 bytes is enough for RGB Panels where 170 Pixels are
    lit at once, e.g. 64x32 with 1/16, 64x64 with 1/32, 32x32 ...
    64x64 with 1/16 won't work, because 256 Pixels are lit at once!

    Timing sequence:
    The main function that actually controls the matrix is following
    this main sequence:
    1) We assume data for physical line X of the display is in the
       shift registers.
    2) We select line X with mux pins A-E
    3) We latch data from the shift registers to their outputs
    4) We copy the Data for line X+1 into the SPI1 Buffer
    5) We init SPI1 data transfer (non blocking)
    6) We tell UART0 to generate an OE pulse (non blocking)
    That's it. The only CPU intensive operation is actually filling the
    SPI1 buffer.
    There are no delays needed at all and no blocking operations.
    SPI transfer and LED pulse are handled by hardware after the
    function executed, leaving the CPU to to other stuff.

    We only send one Line per execution. This means this function has to
    be executed several times to display a complete frame.

    If the line buffer exceeds 64 bytes (170 RGB pixels), e.g. on 64px
    wide panels that drive 4 lines per buffer, we have to execute the
    function twice per line. (NOT IMPLEMENTED YET!)

    The code was meant to be triggered by a timer ISR to maintain a constant
    refresh rate. But it may also be called within a main loop.

    Memory layout:
    Since we have to send data to the matrix panel way more often than
    we have to update the content, the memory layout is optimized
    for sending data to the panel.
    All data that needs to be sent to the panel to fill all shift
    registers is layed out in one continous buffer.
    If we have a 64x32 RGB panel where one mux setting always selects two
    physical rows (16 mux), we have to send 128 pixels at once.
    This means the buffer will contain 128 bits blue, then 128 bits green,
    then 128 bits red.
    This will be layed out line after line.
    If we have color depth > 1, these lines will follow after the frame.
    The first byte in the Buffer, which will get sent out first, will end
    on the end of the last channel B2 on the matrix, which is bottom left,
    because data is clocked in from the right.
    So the memory layout is:
    Line 16 and 0: B2B2...B2B2B1B1...B1B1G2G2...G1G1R2R2...R2R2R1R1...R1R1

    Other panels may require different memory layout. This is all handled
    by drawPixel();

*/

#include "Adafruit_GFX.h"
#include "Arduino.h"
#include <SPI.h>

class ESP8266Matrix : public Adafruit_GFX {
    public:
        uint16_t getPixel(int16_t x, int16_t y);
        ESP8266Matrix(uint16_t panelWidth, uint16_t panelHeight, uint8_t rowsPerMux, uint8_t colorChannels, uint8_t LATCH, uint8_t A, uint8_t B, uint8_t C, uint8_t D, uint8_t E);
        void begin(uint8_t colorDepth, boolean doubleBuffer);
        void loop();
        void drawPixel(int16_t x, int16_t y, uint16_t color);
        void requestBufferSwap();
        void requestVSync();
        boolean readyForDrawing();
        void setBusyModeBlock();
        void setBusyModeSkip();
        void setDoubleBufferModeCopy();
        void setDoubleBufferModeSwap();
        void copyBuffer();
        void initBuffer();
        void clear();
        boolean isBusy();
        boolean readyForFPS(uint8_t fps);
        void setLEDPulseDuration(uint8_t onTime, uint8_t bit);
    private:
        boolean _doubleBuffer = false;
        boolean _initialized = false;
        boolean _constructed = false;
        boolean _copyBuffer = true;
        boolean _blockIfBusy = false;
        volatile boolean _requestBufferSwap = false;
        volatile boolean _VSyncReady = false;
        uint8_t _rowsPerMux; // How many physical rows are lit for one mux setting?
        uint16_t _width; // Physical with of Panel in Pixels
        uint16_t _height; // Physical height of Panel in Pixels
        uint8_t _colorChannels = 3; // primary color channels (usually 3)
        uint8_t _colorDepth = 3; // color depth per primary color in bits
        uint16_t _fb_lines; // How many lines create one image?
        uint16_t _fbs_memory; // Complete memory size in Bytes
        uint16_t _fbs_color; // Size/Offset within Framebuffer where next color starts
        uint16_t _fbs_depth; // Size/Offset within Framebuffer where next color depth starts
        uint16_t _fbs_line; // Size/Offset within Framebuffer where next line starts
        uint8_t* _frameBufferA; // Buffer A/B will always stay the same
        uint8_t* _frameBufferB;
        uint8_t* _frameBufferDisplay; // These will swap between A/B
        uint8_t* _frameBufferDraw;
        uint8_t _PIN_LATCH;
        uint8_t _PIN_A;
        uint8_t _PIN_B;
        uint8_t _PIN_C;
        uint8_t _PIN_D;
        uint8_t _PIN_E;
        const uint8_t _PIN_OE = 2; // Cannot change, this is hardwired to UART1
        uint8_t _BitDurations[8] = {40,20,10,5,2,1,1,1}; // use setLEDPulseDuration()
        uint16_t _gpio15_latch;
        uint16_t _gpio16_latch;
        uint16_t _gpio15_mux[32]; // we support 5 mux channels; 2^5=32 options
        uint16_t _gpio16_mux[32]; // we support 5 mux channels; 2^5=32 options
        uint16_t _gpio15_mux_mask;
        uint16_t _gpio16_mux_mask;
        uint8_t _mux_pins[5];
        volatile uint32_t _lastFPSExecuted = 0;
        void selectMux(uint8_t row);
        void setLatch(boolean on);
        void SPIblock(uint8_t *data, uint16_t size);
        void SPInoblock(uint8_t *data, uint16_t size);
        void SPIwriteBytes(const uint8_t * data, uint8_t size);
};

ESP8266Matrix::ESP8266Matrix(uint16_t panelWidth, uint16_t panelHeight, uint8_t rowsPerMux, uint8_t colorChannels, uint8_t LATCH, uint8_t A = 0xFF, uint8_t B = 0xFF, uint8_t C = 0xFF, uint8_t D = 0xFF, uint8_t E = 0xFF) : Adafruit_GFX(panelWidth,panelHeight) {
    // Here we set all electrical/physical definitions, since these values should never change:
    // panelWidth = How many LEDs/Pixels does a row (where LEDs are lit at once during multiplexing) have?
    // panelHeight = How many rows does the panel have?
    // rowsPerMux = How many rows are lit at once when one mux channel is selected? For a 32x32 panel with 1/16 this is 2.
    // colorChannels = how many primary colors does the panel have? For RGB this is 3, for monochrome ones this is 1.
    // LATCH : The LATCH Pin, usually IO16 / D0
    // A : Mux A Pin, usually IO05 / D1 for 1/2 and above
    // B : Mux B Pin, usually IO04 / D2 for 1/4 and above
    // C : Mux C Pin, usually IO15 / D8 for 1/8 and above
    // D : Mux D Pin, usually IO12 / D6 for 1/16 and above
    // E : Mux E Pin, usually IO00 / D3 for 1/32
    // OE : OE Pin CANNOT BE SET! It is hardwired to IO02 / D4!
    // DATA : DATA Pin CANNOT BE SET! It is hardwired to IO13 / D7!
    // CLK : CLK Pin CANNOT BE SET! It is hardwired to IO14 / D5!
    _constructed = true;
    _PIN_LATCH = LATCH;
    _PIN_A = A;
    _PIN_B = B;
    _PIN_C = C;
    _PIN_D = D;
    _PIN_E = E;
    _mux_pins[0] = A;
    _mux_pins[1] = B;
    _mux_pins[2] = C;
    _mux_pins[3] = D;
    _mux_pins[4] = E;
    _rowsPerMux = rowsPerMux;
    _colorChannels = colorChannels;
    _width = panelWidth;
    _height = panelHeight;
    if(_colorChannels < 1) _colorChannels = 1;
    if(_colorChannels > 4) _colorChannels = 3; // RGBW/RGBY might exist, but if higher we assume the user did something wrong...
}

void ESP8266Matrix::begin(uint8_t colorDepth = 3, boolean doubleBuffer = false) {
    // Here we set all values we need to construct the framebuffer(s) and create it
    // colorDepth : Bits of colorDepth per primary color! 3 results in 8 shades per primary color, thus 512 colors. Max is 5!
    if(!_constructed) return;
    if(_initialized) return;
    _initialized = true;
    _doubleBuffer = doubleBuffer;
    _colorDepth = colorDepth;
    if(_colorDepth < 1) _colorDepth = 1;
    if(_colorDepth > 5) _colorDepth = 5;
    _fb_lines = _height / _rowsPerMux; // How many buffer lines make one picture (excluding depth)
    _fbs_line = (_width * _rowsPerMux * _colorChannels) / 8; // Line size in Bytes
    _fbs_color = (_width * _rowsPerMux) / 8;
    _fbs_depth = _fbs_line * _fb_lines; // Color depth starts after the complete frame
    _fbs_memory = _fbs_depth * _colorDepth;
    _frameBufferA = (uint8_t*) calloc(_fbs_memory, sizeof(byte)); // Complete memory size in Bytes
    if(_doubleBuffer) {
        _frameBufferB = (uint8_t*) calloc(_fbs_memory, sizeof(byte)); // Complete memory size in Bytes
    } else {
        _frameBufferB = _frameBufferA;
    }
    _frameBufferDisplay = _frameBufferA;
    _frameBufferDraw = _frameBufferB;
    pinMode(_PIN_OE, OUTPUT);
    digitalWrite(_PIN_OE, HIGH);
    SPI.begin();
    SPI.setDataMode(SPI_MODE1);
    SPI.setHwCs(false); // Disable CS on D8 / IO15 / CS / HSPI_CS
    SPI.setFrequency(20e6);

    if(_PIN_A != 0xFF) pinMode(_PIN_A, OUTPUT);
    if(_PIN_B != 0xFF) pinMode(_PIN_B, OUTPUT);
    if(_PIN_C != 0xFF) pinMode(_PIN_C, OUTPUT);
    if(_PIN_D != 0xFF) pinMode(_PIN_D, OUTPUT);
    if(_PIN_E != 0xFF) pinMode(_PIN_E, OUTPUT);
    if(_PIN_LATCH != 0xFF) pinMode(_PIN_LATCH, OUTPUT);

    pinMode(2, SPECIAL); // Set GPIO2 as UART1 TX
    U1S |= 0x01 << USTXC; // Set UART1 TX FIFO length
    U1C0 |= 1 << UCLBE; // enable loobback mode so RX mirrors TX

    // Init IO Mux Lookup Tables
    for(uint8_t i = 0 ; i<32; i++) _gpio15_mux[i] = 0;
    for(uint8_t i = 0 ; i<32; i++) _gpio16_mux[i] = 0;
    if(_PIN_LATCH == 16) {
        _gpio15_latch = 0;
        _gpio16_latch = 1;
    }
    if(_PIN_LATCH < 16) {
        _gpio15_latch = (1 << _PIN_LATCH);
        _gpio16_latch = 0;
    }
    for(uint8_t p=0; p<5; p++) {
        if(_mux_pins[p] == 16) {
            for(uint8_t i = 0 ; i<32; i++) _gpio16_mux[i] |= (1 & ( i >> p ));
        }
        if(_mux_pins[p] < 16) {
            for(uint8_t i = 0 ; i<32; i++) _gpio15_mux[i] |= ( (1 & ( i >> p)) << _mux_pins[p] );
        }
    }
    for(uint8_t i = 0 ; i<32; i++) _gpio15_mux_mask |= _gpio15_mux[i];
    for(uint8_t i = 0 ; i<32; i++) _gpio16_mux_mask |= _gpio16_mux[i];


    initBuffer();
    setLEDPulseDuration(40,0);
}

void ESP8266Matrix::drawPixel(int16_t x, int16_t y, uint16_t color) {
    if(!_initialized) return;
    uint16_t _x = x;
    uint16_t _y = y;
    if((x >= _width) || (y >= _height)) return;
    // extract RGB color values
    uint8_t col[3];
    // RRRRRGGG GGGBBBBB -> 8 bit color values
    col[2] = (color >> 8) & B11111000;
    col[1] = (color >> 3) & B11111100;
    col[0] = (color << 3) & B11111000;

    uint8_t _fbl = _y % _fb_lines; // which buffer line

    uint8_t _fbls = (_y / _fb_lines); // which section within the color are we, if it is spread among several lines?
    // since sections are in reverse order in the buffer (B2..B2B1..B1..) we have to reverse this index
    _fbls = _rowsPerMux - _fbls - 1;
    uint8_t _fbx_byte = _x / 8;
    uint8_t _fbx_bit = _x % 8;
    uint16_t pos; // byte position within line
    for(uint8_t _d = 0; _d < _colorDepth; _d++) {
        // MSB to LSB;
        for(uint8_t _c = 0; _c < _colorChannels; _c++) {
            if(_c > 2) break;
            pos = _fbx_byte + _fbl*_fbs_line + _fbls*(_fbs_color/_rowsPerMux) + _fbs_color*_c + _fbs_depth*_d;
            if((col[_c] & (B10000000 >> _d)) > 0 ) {
                _frameBufferDraw[pos] |= (B10000000 >> _fbx_bit); // set 1
            } else {
                _frameBufferDraw[pos] &= ~(B10000000 >> _fbx_bit); // set 0
            }
        }
    }
}

void ESP8266Matrix::initBuffer() {
    // Put arrows in each corner and label Buffers A/B
    // You can remove this stuff using clear()
    // This helps if someone fails to use doubleBuffering
    if(!_initialized) return;
    drawPixel(0,0,0xF800);
    drawPixel(1,0,0xF800);
    drawPixel(0,1,0xF800);
    drawPixel(_width-2,0,0x07E0);
    drawPixel(_width-1,0,0x07E0);
    drawPixel(_width-1,1,0x07E0);
    drawPixel(0,_height-2,0x001F);
    drawPixel(0,_height-1,0x001F);
    drawPixel(1,_height-1,0x001F);
    drawPixel(_width-1,_height-2,0xFFFF);
    drawPixel(_width-1,_height-1,0xFFFF);
    drawPixel(_width-2,_height-1,0xFFFF);
    setTextColor(0xFFFF,0x0000);
    setCursor(_width/2+1,_height/2-4);
    print("B");
    if(_doubleBuffer) {
        copyBuffer();
    }
    setTextColor(0xFFFF,0x0000);
    setCursor(_width/2-6,_height/2-4);
    print("A");
}

void ESP8266Matrix::clear() {
    if(!_initialized) return;
    memset(_frameBufferDraw,0,_fbs_memory);
}

void ESP8266Matrix::requestBufferSwap() {
    // Will request a BufferSwap which will happen on next vsync
    if(!_doubleBuffer) return;
    _requestBufferSwap = true;
}


boolean ESP8266Matrix::readyForDrawing() {
    if(_doubleBuffer) return !_requestBufferSwap;
    return _VSyncReady;
}

inline void ESP8266Matrix::setLatch(boolean on) {
    if(on) {
        GPOS = _gpio15_latch;
        GP16O |= _gpio16_latch;
    } else {
        GPOC = _gpio15_latch;
        GP16O &= ~_gpio16_latch;
    }
}

inline void ESP8266Matrix::selectMux(uint8_t row) {
    // clear all mux pins
    GPOC = _gpio15_mux_mask;
    GP16O &= ~_gpio16_mux_mask;
    // set mux pins
    GPOS = _gpio15_mux[row];
    GP16O |= _gpio16_mux[row];
}

inline boolean ESP8266Matrix::isBusy() {
    if((SPI1CMD & SPIBUSY)) return true; // SPI still sending
    if(!(U1S & (1<<USRXD))) return true; // UART1 TX still low / LEDs on
    return false;
}

inline boolean ESP8266Matrix::readyForFPS(uint8_t fps) {
    // Wait for this to be true if you want to draw with a constant fps
    if(!readyForDrawing()) return false;
    if((micros() - _lastFPSExecuted) > (1e6/fps) ) {
        _lastFPSExecuted = micros();
        return true;
    }
    return false;
}

void ESP8266Matrix::setBusyModeBlock() {
    // If you call loop() again before SPI transfer is done / LED Pulse finished,
    // this mode will block loop() until it can run safely
    _blockIfBusy = true;
}

void ESP8266Matrix::setBusyModeSkip() {
    // If you call loop() again before SPI transfer is done / LED Pulse finished,
    // this mode will skip loop() and return until it can run safely again
    _blockIfBusy = false;
}

void ESP8266Matrix::setDoubleBufferModeCopy() {
    // In this mode, the drawing buffer will be copied to the display buffer
    // This is slower - it takes ~24µs for 48 Byte Buffer - but the user will
    // always have a concistent framebuffer to work with
    if(!_doubleBuffer) return;
    _copyBuffer = true;
}

void ESP8266Matrix::setDoubleBufferModeSwap() {
    // In this mode, the drawing buffer and the display buffer are exchanged
    // This is faster, but the user will be presented a drawing buffer from the
    // previous frame, which might be confusing
    if(!_doubleBuffer) return;
    _copyBuffer = false;
}

inline void ESP8266Matrix::copyBuffer() {
    // This will copy the drawing buffer to the display buffer immediately
    if(!_initialized) return;
    if(!_doubleBuffer) return;
    memcpy(_frameBufferDisplay,_frameBufferDraw,_fbs_memory);
}

void ESP8266Matrix::setLEDPulseDuration(uint8_t onTime, uint8_t bit = 0) {
    // This will set the On-Time for the LEDs for the MSB and half it
    // for all less significant bits if using colorDepth.
    // You can set the length for each bit individually if you call
    // setLEDPulseDuration(40);
    // setLEDPulseDuration(15,1);
    // setLEDPulseDuration(6,2);
    // etc...
    while (bit < 8) {
        _BitDurations[bit] = onTime / (1<<bit);
        if(_BitDurations[bit] == 0) _BitDurations[bit] = 1; // ensure minimum of 1
        bit++;
    }
}

ICACHE_RAM_ATTR void ESP8266Matrix::loop() {
    static uint8_t lineIdx = 0;
    static uint8_t _colorDepthIdx = 0;
    static uint8_t line = 0;

    while(isBusy()) {
        if(!_blockIfBusy) return;
    }

    // select Line and latch data; 6,6µs
    setLatch(true);
    selectMux(line);
    setLatch(false);

    U1D = 10*_BitDurations[_colorDepthIdx];

    // get next line
    lineIdx++;
    if(lineIdx >= _fb_lines) {
        lineIdx = 0;
        _colorDepthIdx++;
        if(_colorDepthIdx >= _colorDepth) _colorDepthIdx = 0;
    }
    line = lineIdx;

    // Enable the LEDs, enough time went by for Latch/Mux... if not, put it at the END
      U1F = 0x80; // LED Pulse


    // When we're about to send the first line to the display, we swap buffer if requested
    if(_requestBufferSwap && (lineIdx == 0) && (_colorDepthIdx == 0) ) {
        _requestBufferSwap = false;
        if(_copyBuffer) {
            copyBuffer();
        } else {
            if(_frameBufferDisplay == _frameBufferA) {
                _frameBufferDisplay = _frameBufferB;
                _frameBufferDraw = _frameBufferA;
            } else {
                _frameBufferDisplay = _frameBufferA;
                _frameBufferDraw = _frameBufferB;
            }
        }
    }

    // Now send the framebuffer line via SPI
    SPInoblock(_frameBufferDisplay + line*_fbs_line + _colorDepthIdx*_fbs_depth, _fbs_line);

    // When we've sent the data for the last line to the display, we are ready for vsync
    if(lineIdx == _fb_lines-1 && _colorDepthIdx == _colorDepth-1 ) {
        _VSyncReady = true;
    } else {
        _VSyncReady = false;
    }

}

inline void ESP8266Matrix::SPIblock(uint8_t *data, uint16_t size) {
    SPInoblock(data, size);
    while(SPI1CMD & SPIBUSY);
}

inline void ESP8266Matrix::SPInoblock(uint8_t *data, uint16_t size) {
    // See SPIClass::writeBytes
    while(size) {
        if(size > 64) {
            SPIwriteBytes(data, 64);
            size -= 64;
            data += 64;
        } else {
            SPIwriteBytes(data, size);
            size = 0;
        }
    }
}

inline void ESP8266Matrix::SPIwriteBytes(const uint8_t * data, uint8_t size) {
    while(SPI1CMD & SPIBUSY);
    // Set Bits to transfer
    const uint32_t bits = (size * 8) - 1;
    const uint32_t mask = ~(SPIMMOSI << SPILMOSI);
    SPI1U1 = ((SPI1U1 & mask) | (bits << SPILMOSI));

    uint32_t * fifoPtr = (uint32_t*)&SPI1W0;
    const uint32_t * dataPtr = (uint32_t*) data;
    uint32_t dataSize = ((size + 3) / 4);

    while(dataSize--) {
        *fifoPtr = *dataPtr;
        dataPtr++;
        fifoPtr++;
    }

    __sync_synchronize();
    SPI1CMD |= SPIBUSY;
}
