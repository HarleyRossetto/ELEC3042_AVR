/*
 * File:   22S1_ELEC3042_SPI.c
 * Author: rex
 *
 * Created on 5 March 2021, 7:57 PM
 * 
 * This file talks SPI to an MCP23S17 dual port expander chip
 * Slave select for the SPI chip is on PB2
 */

#include <xc.h>
#include <stdint.h>

/**
 * Transfer a byte of data across the SPI bus.
 * We return the byte of data returned (as SPI is synchronous)
 * @param data to transmit
 * @return data returned by slave
 */
uint8_t SPI_transfer(uint8_t data) {
    
    SPDR = data;
    while ((SPSR & _BV(SPIF)) == 0) {
        ;   // wait until transfer completed
    }
    return SPDR;
}

/**
 * Send a command/data byte pair to the MCP23S17
 * 
 * @param reg command register to which we will be writing.
 * @param data value to write to command register
 */
void SPI_Send_Command(uint8_t reg, uint8_t data) {
    // Send a command + byte to SPI interface
    // TODO
}

/**
 * Read the value of a register on the MCP23S17
 * 
 * @param reg data register we wish to read
 * @return value of the register we read
 */
uint8_t SPI_Read_Command(uint8_t reg) {   
    // Read a command output from SPI interface
    // TODO
}

/**
 * Set up the SPI bus.
 * We assume a 16MHz IOclk rate, and that Port B Pin 2 is the SS output
 */
void setup_SPI() {
    // TODO

/*
 * Now that the SPI interface is configured we need to send SPI commands to
 * configure the MCP23S17 port expander IC
 *
 * We will configure port A as all outputs
 * Port B 0-3 are outputs and 4-7 are inputs
 * We turn on pullup resistors on Port B
 */
    SPI_Send_Command(0x00, 0x00);   // register IODIRA (port A data direction)
    SPI_Send_Command(0x01, 0xf0);   // register IODIRB (port B data direction)
    SPI_Send_Command(0x0d, 0xff);   // register GPPUB (port B GPIO Pullups)
}
