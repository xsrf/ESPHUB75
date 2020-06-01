#pragma once

/*
    https://github.com/xsrf/ESP8266Matrix

    This Library is designed to drive Shift-Register based multiplexing
    LED Matrix panles with as little CPU load as possible by using
    hardware peripherials of the ESP8266 and ESP32.

    The UART1 hardware is used to generate an acurate pulse for
    OE / enabling the LEDs.
    This means that OE MUST be connected to D4 (WeMos) / IO_02 on
    the ESP8266.

    Data transfer is handled by SPI1 hardware, but we're not using the
    official Arduino SPI lib, because it only has blocking functions.
    On the ESP8266 we use the full 64 bytes hardware buffer to send
    64 bytes of data at once without blocking. Interrups are used to
    queue up additional data when the buffer was sent, allowing us to
    send unlimited data without blocking but only short ISR code every
    64 bytes.    
    On the ESP32 DMA is used for SPI transfer. This has a slightly
    longer setup time, but allows sending unlimited data with only
    one call and no additional ISRs.
    
    Timing sequence:
    The main function that actually controls the matrix is following
    this sequence:
    1) We assume data for physical line X of the display is in the
       shift registers.
    2) We select line X with mux pins A-E
    3) We latch data from the shift registers to their outputs
    4) We tell UART1 to generate an OE pulse (non blocking)
    5) We send the Data for line X+1 via SPI (non blocking)
    That's it. The only CPU intensive operation is actually filling the
    SPI1 buffer on the ESP8266 / preparing DMA on ESP32.
    There are no delays needed at all and no blocking operations.
    SPI transfer and LED pulse are handled by hardware after the
    function executed, leaving the CPU to to other stuff.

    We only send one Line per execution. This means this function has to
    be executed several times to display a complete frame.

    The code was meant to be triggered by a timer ISR to maintain a constant
    refresh rate. But it may also be called within a main loop.

    The main function will return without doing anything if SPI transfer
    from the previous call is still not finished or the LEDs are still enabled.
    This allows the Timer ISR to call it more often than always needed to adapt
    for shorter runs.

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

    Standard Wiring:
    
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

*/

#include <Arduino.h>
#include <SPI.h>
#include "nbSPI.h"
#include "Adafruit_GFX.h"

#ifdef ESP32
    #include <soc/uart_reg.h>
    #include <esp32-hal-uart.c>
    #define _GPOS(val) GPIO.out_w1ts = val
    #define _GPOC(val) GPIO.out_w1tc = val
#endif

#ifdef ESP8266
    #define _GPOS(val) GPOS = val; GP16O |= ((val >> 16) & 1)
    #define _GPOC(val) GPOC = val; GP16O &= ~((val >> 16) & 1)
#endif

class ESP8266Matrix : public Adafruit_GFX {
    public:
        uint16_t getPixel(int16_t x, int16_t y);
        ESP8266Matrix(uint16_t panelWidth, uint16_t panelHeight, uint8_t rowsPerMux, uint8_t colorChannels, uint8_t LATCH, uint8_t A, uint8_t B, uint8_t C, uint8_t D, uint8_t E);
        void begin(uint8_t colorDepth, bool doubleBuffer);
        void loop();
        void drawPixel(int16_t x, int16_t y, uint16_t color);
        void requestBufferSwap();
        void requestVSync();
        bool readyForDrawing();
        void setBusyModeBlock();
        void setBusyModeSkip();
        void copyBuffer(bool reverse);
        void initBuffer();
        void clear();
        void clearDisplay();
        bool isBusy();
        bool readyForFPS(uint8_t fps);
        void setLEDPulseDuration(uint8_t onTime, uint8_t bit);
        uint16_t color565(uint8_t r, uint8_t g, uint8_t b);
        void drawPixelRGB888(uint16_t x, uint16_t y, uint8_t r, uint8_t g, uint8_t b);
    private:
        bool _doubleBuffer = false;
        bool _initialized = false;
        bool _constructed = false;
        bool _blockIfBusy = false;
        volatile bool _requestBufferSwap = false;
        volatile bool _VSyncReady = false;
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
        uint8_t _PIN_OE = 2; // UART1 TX, cannot change on ESP8266
        uint8_t _PIN_CLK = 14; // SPI Clock PIN, cannot change on ESP8266
        uint8_t _PIN_DAT = 13; // SPI Data PIN, cannot change on ESP8266
        uint8_t _BitDurations[8] = {40,20,10,5,2,1,1,1}; // use setLEDPulseDuration()
        uint32_t _gpio_latch;
        uint32_t _gpio_mux[32]; // we support 5 mux channels; 2^5=32 options
        uint32_t _gpio_mux_mask;
        uint8_t _mux_pins[5];
        uint8_t _spi_mode = SPI_MODE0;
        uint32_t _spi_freq = 20e6;
        volatile uint32_t _lastFPSExecuted = 0;
        void selectMux(uint8_t row);
        void setLatch(bool on);
        void _initSPI(uint32_t freq, uint8_t mode, uint8_t pin_clk, uint8_t pin_data);
        void _initStrobe(uint8_t pin_oe);
        void strobe(uint16_t length_us);
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

void ESP8266Matrix::begin(uint8_t colorDepth = 3, bool doubleBuffer = false) {
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

    _initSPI(_spi_freq, _spi_mode, _PIN_CLK, _PIN_DAT);
    _initStrobe(_PIN_OE);

    if(_PIN_A != 0xFF) pinMode(_PIN_A, OUTPUT);
    if(_PIN_B != 0xFF) pinMode(_PIN_B, OUTPUT);
    if(_PIN_C != 0xFF) pinMode(_PIN_C, OUTPUT);
    if(_PIN_D != 0xFF) pinMode(_PIN_D, OUTPUT);
    if(_PIN_E != 0xFF) pinMode(_PIN_E, OUTPUT);
    if(_PIN_LATCH != 0xFF) pinMode(_PIN_LATCH, OUTPUT);

    // Init IO Mux Lookup Tables
    for(uint8_t i = 0 ; i<32; i++) _gpio_mux[i] = 0;
    _gpio_latch = (1 << _PIN_LATCH);
    for(uint8_t p=0; p<5; p++) {
        for(uint8_t i = 0 ; i<32; i++) _gpio_mux[i] |= ( (1 & ( i >> p)) << _mux_pins[p] );
    }
    for(uint8_t i = 0 ; i<32; i++) _gpio_mux_mask |= _gpio_mux[i];

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

void ESP8266Matrix::drawPixelRGB888(uint16_t x, uint16_t y, uint8_t r, uint8_t g, uint8_t b) {
    drawPixel(x,y,color565(r,g,b));
}

uint16_t ESP8266Matrix::color565(uint8_t r, uint8_t g, uint8_t b) {
    uint16_t col16 = 0;
    col16 |= (r>>3)<<11;
    col16 |= (g>>2)<<5;
    col16 |= (b>>3);
    return col16;
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
        copyBuffer(true);
    }
    setTextColor(0xFFFF,0x0000);
    setCursor(_width/2-6,_height/2-4);
    print("A");
}

void ESP8266Matrix::clear() {
    if(!_initialized) return;
    memset(_frameBufferDraw,0,_fbs_memory);
}

void ESP8266Matrix::clearDisplay() {
    clear();
}

void ESP8266Matrix::requestBufferSwap() {
    // Will request a BufferSwap which will happen on next vsync
    if(!_doubleBuffer) return;
    _requestBufferSwap = true;
}


bool ESP8266Matrix::readyForDrawing() {
    if(_doubleBuffer) return !_requestBufferSwap;
    return _VSyncReady;
}

inline void ESP8266Matrix::setLatch(bool on) {
    if(on) {
        _GPOS(_gpio_latch);
    } else {
        _GPOC(_gpio_latch);
    }
}

inline void ESP8266Matrix::selectMux(uint8_t row) {
    // clear all mux pins
    _GPOC(_gpio_mux_mask);
    // set mux pins
    _GPOS(_gpio_mux[row]);
}

inline bool ESP8266Matrix::isBusy() {
    if(nbSPI_isBusy()) return true; // SPI still sending
    #ifdef ESP8266
        if(!(U1S & (1<<USRXD))) return true; // UART1 TX still low / LEDs on
    #endif
    #ifdef ESP32
        if(!(READ_PERI_REG(UART_STATUS_REG(1)) & UART_TXD)) return true; // UART1 TX still low / LEDs on
    #endif
    return false;
}

inline bool ESP8266Matrix::readyForFPS(uint8_t fps) {
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

inline void ESP8266Matrix::copyBuffer(bool reverse = false) {
    // This will copy the display buffer to the drawing buffer immediately (or inverse)
    // Use this in your application before drawing, if you rely on the display buffer
    // to stay the same after every frame
    if(!_initialized) return;
    if(!_doubleBuffer) return;
    if(reverse) {
        memcpy(_frameBufferDisplay,_frameBufferDraw,_fbs_memory);
    } else {
        memcpy(_frameBufferDraw,_frameBufferDisplay,_fbs_memory);
    }
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

    // Enable the LEDs, enough time went by for Latch/Mux... if not, put it at the END
    strobe(_BitDurations[_colorDepthIdx]);

    // get next line
    lineIdx++;
    if(lineIdx >= _fb_lines) {
        lineIdx = 0;
        _colorDepthIdx++;
        if(_colorDepthIdx >= _colorDepth) _colorDepthIdx = 0;
    }
    line = lineIdx;

    // When we're about to send the first line to the display, we swap buffer if requested
    if(_requestBufferSwap && (lineIdx == 0) && (_colorDepthIdx == 0) ) {
        _requestBufferSwap = false;
        if(_frameBufferDisplay == _frameBufferA) {
            _frameBufferDisplay = _frameBufferB;
            _frameBufferDraw = _frameBufferA;
        } else {
            _frameBufferDisplay = _frameBufferA;
            _frameBufferDraw = _frameBufferB;
        }
    }

    // Now send the framebuffer line via SPI
    nbSPI_writeBytes(_frameBufferDisplay + line*_fbs_line + _colorDepthIdx*_fbs_depth, _fbs_line);

    // When we've sent the data for the last line to the display, we are ready for vsync
    if(lineIdx == _fb_lines-1 && _colorDepthIdx == _colorDepth-1 ) {
        _VSyncReady = true;
    } else {
        _VSyncReady = false;
    }

}

void ESP8266Matrix::_initSPI(uint32_t freq, uint8_t mode, uint8_t pin_clk, uint8_t pin_data) {
    #ifdef ESP32
        nbSPI_init(freq, mode, pin_clk, pin_data);
    #endif
    #ifdef ESP8266
        SPI.begin();
        SPI.setDataMode(mode);
        SPI.setHwCs(false); // Disable CS on D8 / IO15 / CS / HSPI_CS
        SPI.setFrequency(freq);
    #endif
}

void ESP8266Matrix::_initStrobe(uint8_t pin_oe) {
    #ifdef ESP32
        DPORT_SET_PERI_REG_MASK(DPORT_PERIP_CLK_EN_REG, DPORT_UART1_CLK_EN); // Enable UART1 Clock
        DPORT_CLEAR_PERI_REG_MASK(DPORT_PERIP_RST_EN_REG, DPORT_UART1_RST); // Enable UART1 Hardware
        pinMode(pin_oe, OUTPUT);
        pinMatrixOutAttach(pin_oe, UART_TXD_IDX(1), false, false); // UART_TXD_IDX(u) / U1TXD_OUT_IDX
        WRITE_PERI_REG(UART_MEM_CONF_REG(1),  1 << UART_TX_SIZE_S); // Set TX Buffer to 1
    #endif
    #ifdef ESP8266
        pinMode(2, SPECIAL); // Set GPIO2 as UART1 TX, pin cannot be changed
        U1S |= 0x01 << USTXC; // Set UART1 TX FIFO length
        U1C0 |= 1 << UCLBE; // enable loobback mode so RX mirrors TX
    #endif 
}

inline void ESP8266Matrix::strobe(uint16_t length_us) {
    #ifdef ESP32
        WRITE_PERI_REG(UART_CLKDIV_REG(1), 10*length_us);
        WRITE_PERI_REG(UART_FIFO_REG(1), 0x80);
        CLEAR_PERI_REG_MASK( UART_IDLE_CONF_REG(1), UART_TX_BRK_NUM_M );
        CLEAR_PERI_REG_MASK( UART_IDLE_CONF_REG(1), UART_TX_IDLE_NUM_M );
    #endif 
    #ifdef ESP8266
        U1D = 10*length_us; // Pulse length
        U1F = 0x80; // Pulse
    #endif 
}