/*
 * max31865.c
 *
 * Created: 13.9.2019 12:34:48
 * Author : strakos.p
 */

// Includes
#include "max31865.h"
#include "io_manipulation.h"

#include "../usart.h"

// Reading data from SPI
char max_spi_read(char addr) {

    RESET(MAX_CS);              // CS_LOW - activate slave

    SPDR = addr;                // register address to read
    while(!(SPSR & (1<<SPIF))); // wait for complete sending

    SPDR = 0xFF;                // dummy data to provide SPI clock signals to read
    while(!(SPSR & (1<<SPIF))); // wait for complete sending

    SET(MAX_CS);                // CS_HIGH - deactivate slave
    return SPDR;                // delete flag SPIF and return data
}

// Writing data to SPI
void max_spi_write(char addr, char data) {

    RESET(MAX_CS);              // CS_LOW - activate slave

    SPDR = addr;                // register address to write
    while(!(SPSR & (1<<SPIF))); // wait for complete sending

    SPDR = data;                // data to write
    while(!(SPSR & (1<<SPIF))); // wait for complete sending

    SET(MAX_CS);                // CS_HIGH - deactivate slave
}

// Port initialization
void max_init_port(void) {

    // set outputs
    SET_OUTPUT(MAX_MOSI);
    SET_OUTPUT(MAX_SCK);
    SET_OUTPUT(MAX_CS);
    SET(MAX_CS);

    SET_OUTPUT(MAX_SS); // needs to be either put on HIGH or set as output!

    // set inputs
    SET_INPUT(MAX_DRDY);
    RESET(MAX_DRDY);
    SET_INPUT(MAX_MISO);
    RESET(MAX_MISO);

    // SPI setting
    /* Enable SPI, Master, set mode 3, set clock rate fck/128 */
    SPCR =
        (0 << SPIE) |
        (1 << SPE)  |   // SPI enabled
        (0 << DORD) |
        (1 << MSTR) |   // Master
        (1 << CPOL) |   // Clock polarity
        (1 << CPHA) |   // Clock phase
        (1 << SPR1) |
        (1 << SPR0);    // Clock F_CPU/128
}



/**
 * Initializes communication with max
 * @return 1 if communication is done properly, otherwise returns 0
 */
uint8_t init_max(void) {
    uint8_t conf = 0;
    max_spi_write(CONFIGURATION, 0b10000000);   // Enable V bias
    conf = max_spi_read(READ_CONFIGURATION);    // Reading Configuration register to verify communication

    if(conf == 0b10000000) {
        max_spi_write(WRITE_HIGH_FAULT_TRESHOLD_MSB, 0xFF);     // Writing High Fault Threshold MSB
        max_spi_write(WRITE_HIGH_FAULT_TRESHOLD_LSB, 0xFF);     // Writing High Fault Threshold LSB
        max_spi_write(WRITE_LOW_FAULT_TRESHOLD_MSB, 0x00);      // Writing Low Fault Threshold MSB
        max_spi_write(WRITE_LOW_FAULT_TRESHOLD_LSB, 0x00);      // Writing Low Fault Threshold LSB
        return 1;
    } else {
        return 0;
    }
}

// Fault test
uint8_t max_fault_test(void) {
    /*
        Returns 0 if there is not fault detected
    */

    uint8_t  lsb_rtd;

    lsb_rtd = max_spi_read(READ_RTD_LSB);

    return lsb_rtd&0x01;
}

// Reading data from max
long max_get_data(char datatype) {
    /*
        Parameter datatype is for choose between temperature and RTD as return value, type:
        'r' for RTD
        't' for temperature.

        In normal operation function returns RTD or temperature multiplexed by 10
        If new conversion result is not available - DRDY is high, returns 1000
    */

    uint8_t lsb_rtd, msb_rtd;
    uint8_t DRDY_state;

    int RTD;
    long temperature;

    DRDY_state = IS_SET(MAX_DRDY);

    if (DRDY_state == 0) {
        lsb_rtd = max_spi_read(READ_RTD_LSB);
        msb_rtd = max_spi_read(READ_RTD_MSB);

        RTD = ((msb_rtd << 7)+((lsb_rtd & 0xFE) >> 1));
        if (datatype == 'r') return RTD;

        // Basic temperature calculation, the accuracy for temperatures
        // between -75°C - 100°C is max +-1.5°C
        temperature = ( (long)10*RTD >> 5) - 2560;  // RTD / 32 - 256
        if (datatype == 't') return temperature;
    }
    // return 1000 if not conversion available
    return 1000;
}

