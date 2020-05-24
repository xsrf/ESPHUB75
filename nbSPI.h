/*
    Interrupt/DMA based SPI writeBytes() function for ESP8266 / ESP32
    -----------------------------------------------------------------
    This library is meant for sending data via SPI without blocking
    the CPU. It just sends data, no reading, no chip-select.

    Use nbSPI_writeBytes(data, size) instead of SPI.writeBytes(data, size).
    nbSPI_writeBytes() will return almost immediatly and NOT block the CPU
    until all data is sent.

    You can init the SPI with nbSPI_init().

    You can query if SPI is still busy with nbSPI_isBusy().

    For ESP8266:
    ------------
    Sending 80 Bytes via nbSPI_writeBytes blocks the CPU for 10µs (6.6µs + 3.5µs).
    Sending 80 Bytes @ 20MHz via SPI.writeBytes blocks the CPU for 40µs.
    Sending 80 Bytes @ 8MHz via SPI.writeBytes blocks the CPU for 88µs.

    For ESP32:
    ----------
    Sending 80 Bytes via nbSPI_writeBytes blocks the CPU for 17µs.
    Sending 80 Bytes @ 20MHz via SPI.writeBytes blocks the CPU for 40µs.
    Sending 80 Bytes @ 8MHz via SPI.writeBytes blocks the CPU for 88µs.
*/

#pragma once

volatile bool _nbspi_isbusy = false;

void nbSPI_writeBytes(uint8_t *data, uint16_t size);
bool nbSPI_isBusy();

#ifdef ESP8266
#include <SPI.h>

// Set an IO to measure timing
#ifdef NBSPI_TDBG_IO
    #if NBSPI_TDBG_IO < 16
        #define NBSPI_TDBG_HIGH GPOS = (1<<NBSPI_TDBG_IO) // 0.1 µs
        #define NBSPI_TDBG_LOW GPOC = (1<<NBSPI_TDBG_IO) // 0.1 µs
    #else
        #error "Use IO 0-15!"
    #endif
#else
    #define NBSPI_TDBG_HIGH
    #define NBSPI_TDBG_LOW
#endif

volatile uint16_t _nbspi_size = 0;
volatile uint32_t * _nbspi_data;

void nbSPI_writeChunk();
void nbSPI_ISR();

uint8_t* nbSPI_malloc(uint16_t size) {
    return (uint8_t*) calloc(size, sizeof(byte));
}

void nbSPI_init(uint32_t freq = 20e6, uint8_t mode = 0) {
    SPI.begin();
    SPI.setFrequency(freq);
    SPI.setDataMode(mode);
}

ICACHE_RAM_ATTR boolean nbSPI_isBusy() {
    if(_nbspi_isbusy) return true; // true while not all data put into buffer
    return (SPI1CMD & SPIBUSY); // true while SPI sends data
}

// This will accept data of ary length to send via SPI
ICACHE_RAM_ATTR void nbSPI_writeBytes(uint8_t *data, uint16_t size) {
    NBSPI_TDBG_HIGH;
    _nbspi_isbusy = true;
    _nbspi_size = size;
    _nbspi_data = (uint32_t*) data;
    SPI0S &= ~(0x1F); // Disable and clear all interrupts on SPI0
    SPI1S &= ~(0x1F); // Disable and clear all interrupts on SPI1
    ETS_SPI_INTR_ATTACH(nbSPI_ISR, NULL); // Set Interrupt handler
    ETS_SPI_INTR_ENABLE(); // Enable SPI Interrupts
    nbSPI_writeChunk(); // Write first chunk of data
    NBSPI_TDBG_LOW;
}

// This will send up to 64 bytes via SPI
ICACHE_RAM_ATTR void inline nbSPI_writeChunk() {    
    uint16_t size = _nbspi_size;
    if(size > 64) size = 64;
    _nbspi_size -= size;
    const uint32_t bits = (size * 8) - 1;
    const uint32_t mask = ~(SPIMMOSI << SPILMOSI);
    SPI1U1 = ((SPI1U1 & mask) | (bits << SPILMOSI));

    uint32_t * fifoPtr = (uint32_t*)&SPI1W0;
    uint8_t dataSize = ((size + 3) / 4); // Words to copy to SPI buffer / will actually read beyond your data if it is not 4 Byte aligned
    while(dataSize--) {
        *fifoPtr = *_nbspi_data;
        _nbspi_data++;
        fifoPtr++;
    }
    __sync_synchronize();

    if(_nbspi_size > 0) {
        // There is more data to send after this one, enable Interrupt
        SPI1S |= SPISTRIE; // Enable Transmission End interrupt on SPI1, SPI_TRANS_DONE_EN
    } else {
        // All data is sent after this one
        ETS_SPI_INTR_DISABLE(); // Disable SPI Interrupts
    }

    SPI1CMD |= SPIBUSY; // Start sending data

    if(_nbspi_size == 0) _nbspi_isbusy = false; // Clear flag after starting SPI
}

// Interrupt Handler gets called when SPI finished sending data
ICACHE_RAM_ATTR void nbSPI_ISR() {
    NBSPI_TDBG_HIGH;
    if(SPIIR & (1 << SPII0)) {
        // SPI0 Interrupt
        SPI0S &= ~(0x1F); // Disable and clear all interrupts on SPI0
    }
    if(SPIIR & (1 << SPII1)) {
        // SPI1 Interrupt
        SPI1S &= ~(0x1F); // Disable and clear all interrupts on SPI1
        nbSPI_writeChunk(); // Write remaining data
    }
    NBSPI_TDBG_LOW;
}

#endif

#ifdef ESP32

#include "driver/spi_master.h"

// Set an IO to measure timing
#ifdef NBSPI_TDBG_IO
    #if NBSPI_TDBG_IO < 32
        #define NBSPI_TDBG_HIGH GPIO.out_w1ts = (1<<NBSPI_TDBG_IO) // 0.1 µs
        #define NBSPI_TDBG_LOW GPIO.out_w1tc = (1<<NBSPI_TDBG_IO) // 0.1 µs
    #else
        #error "Use IO 0-31!"
    #endif
#else
    #define NBSPI_TDBG_HIGH
    #define NBSPI_TDBG_LOW
#endif

spi_device_handle_t _nbspi_device_handle;
spi_bus_config_t _nbspi_bus;
spi_device_interface_config_t _nbspi_device_config;
spi_transaction_t _nbspi_trans;

void nbSPI_ISR(spi_transaction_t *t);

uint8_t* nbSPI_malloc(uint16_t size) {
    return (uint8_t*)heap_caps_malloc(size*sizeof(uint8_t), MALLOC_CAP_DMA);
}

void nbSPI_init(uint32_t freq = 20e6, uint8_t mode = 0, uint8_t io_sclk = 14, uint8_t io_data = 13) {
    _nbspi_bus.miso_io_num = -1; // no MISO pin
    _nbspi_bus.mosi_io_num = io_data;
    _nbspi_bus.sclk_io_num = io_sclk;
    _nbspi_bus.max_transfer_sz = 4094;
    _nbspi_device_config.clock_speed_hz = freq;
    _nbspi_device_config.mode = mode;
    _nbspi_device_config.spics_io_num = -1; // no CS pin
    _nbspi_device_config.queue_size = 1;
    _nbspi_device_config.post_cb = nbSPI_ISR;
    spi_bus_initialize(HSPI_HOST, &_nbspi_bus, 2); // DMA 2
    spi_bus_add_device(HSPI_HOST, &_nbspi_device_config, &_nbspi_device_handle);
}

IRAM_ATTR void nbSPI_ISR(spi_transaction_t *t) {
    NBSPI_TDBG_HIGH;
    _nbspi_isbusy = false;
    NBSPI_TDBG_LOW;
}

void nbSPI_writeBytes(uint8_t *data, uint16_t size) {
    NBSPI_TDBG_HIGH;
    _nbspi_isbusy = true;
    _nbspi_trans.length = size*8;
    _nbspi_trans.tx_buffer = data;
    spi_device_queue_trans(_nbspi_device_handle, &_nbspi_trans, portMAX_DELAY); // 16µs 
    NBSPI_TDBG_LOW;
}

bool nbSPI_isBusy() {
    return _nbspi_isbusy;
}

#endif