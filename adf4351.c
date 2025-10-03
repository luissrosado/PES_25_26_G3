/*
 * File:   adf4351.c
 * Author: simas
 *
 * Created on 26 September 2025, 20:27
 */


#include "xc.h"
#include "adf4351.h"
#include <libpic30.h>

//! Register values for 1 GHz output, programmed in the order R5, R4, R3, ...
const uint32_t adf4351_reg_values[6] = {
    0x00580005, // Register 5: LD Pin Mode = Digital Lock Detect
    0x00AC8024, // Register 4: Pwr=-4dBm
    0x000004B3, // Register 3
    0x00004E42, // Register 2: R=1, PD Pol=Pos, Low Noise Mode
    0x08008011, // Register 1: Prescaler=8/9
    0x00500000  // Register 0
};

/*
 * @brief Performs a hardware reset to ADF4351 synth
 * @note To make sure the ADF4351 starts in a known default state before config
 */
void ADF4351_Hardware_Reset(void) {
    ADF4351_CE_LAT = 0; // Make Chip Enable (CE) low to power down
    __delay_ms(1);
    ADF4351_CE_LAT = 1; // CE to 1 to power the device
    __delay_ms(1);      // For internal oscillator stabilisation
}

/*
 * @brief Writes a 32-bit register value to the ADF4351 synth
 * @param reg_value The 32-bit word to be programmed into the ADF
 */
void ADF4351_Write_Register(uint32_t reg_value) {
    uint16_t high_word = (uint16_t)((reg_value >> 16) & 0xFFFF);
    uint16_t low_word  = (reg_value & 0xFFFF);
    
    // ADF expects MSB of each word first -> send high word first
    
    //ADF4351_LE_LAT = 1; // Latch data on rising edge
    
    // Send high 16 bits
    SPI1BUFL = high_word;
    while(!SPI1STATLbits.SPIRBF); // Wait for transfer to complete
    (void) SPI1BUFL; // Clear rcv buffer
    
    // Send low 16 bits
    SPI1BUFL = low_word;
    while(!SPI1STATLbits.SPIRBF); // Wait for transfer to complete
    (void) SPI1BUFL; // Clear rcv buffer
    
    ADF4351_LE_LAT = 1; // Latch data on rising edge
    __delay_us(10);
    ADF4351_LE_LAT = 0; // Set LE low
    __delay_us(10);
}
