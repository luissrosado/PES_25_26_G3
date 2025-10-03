#ifndef SYSTEM_CONFIG_H
#define SYSTEM_CONFIG_H

// System Configuration
#define FOSC 8000000UL  // Inform the compiler of the 8MHz
#define FCY (FOSC / 2)

// Application Configuration
#define TIMER1_TICK_HZ  0.5     // 1/2 Hz = 2 seconds
#define SPI_BAUD_RATE   1000000 // SPI clock speed in Hz
#define SPI_TIMEOUT     1000    // Timeout counter value for SPI

// Constant calculation
#define TIMER1_PERIOD   (FCY / 256 / TIMER1_TICK_HZ) - 1
#define SPI1_BRG_VALUE  ((FCY / (2 * SPI_BAUD_RATE)) - 1) // Baud rate gwnerator (equation 17-1)
#define SPI2_BRG_VALUE  ((FCY / (2 * SPI_BAUD_RATE)) - 1)

// Pin Definitions
#define ADF4351_LE_TRIS TRISBbits.TRISB6  // LE (output) is on RB6/RP6
#define ADF4351_LE_LAT  LATBbits.LATB6
#define ADF4351_LD_TRIS TRISBbits.TRISB9  // LD (input) is on RB9/RP9 // TODO: Está ligado ao PDRF!!!
#define ADF4351_LD_PORT PORTBbits.RB9
#define ADF4351_CE_TRIS TRISBbits.TRISB11 // CE (output) on RB11
#define ADF4351_CE_LAT  LATBbits.LATB11
#define LOCK_LED_TRIS   TRISBbits.TRISB15 // Success indicator LED is on RB15/RP15
#define LOCK_LED_LAT    LATBbits.LATB15

#endif
