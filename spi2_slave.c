/*
 * File:   spi2_slave.c
 * Author: simas
 *
 * Created on 24 September 2025, 16:07
 */

#include "spi2_slave.h"

typedef enum {
    STATE_WAITING_FOR_ADDR,         // 0
    STATE_WAITING_FOR_DATA_WRITE    // 1
} SpiSlaveState;

// Private variables
static volatile SpiSlaveState spi2_state = STATE_WAITING_FOR_ADDR;
static volatile uint8_t current_reg_addr = 0;
static volatile uint8_t received_byte_from_isr = 0;

// Public variables
volatile uint8_t spi_registers[16]; // Storage for registers
volatile uint8_t spi2_received_data  = 0; // Data from the ISR
volatile bool spi2_transaction_start_flag = false; // Set by IOC on SS falling edge
volatile bool spi2_byte_received_flag = false; // Set by SPI ISR when a byte arrives

/*
 * @brief SPI2 slave events
 * @note Fires when a full byte/word has been received from the master
 */
void __attribute__((__interrupt__, no_auto_psv)) _SPI2Interrupt(void) {
    /* REAL 
    spi2_received_data = SPI2BUFL;  // Read received data + clear SPIRBF flag
    spi2_byte_received_flag = true; // Set flag
    IFS2bits.SPI2IF = 0;            // Clear master SPI2 interrupt flag
    */
    
    /* TEST 1: ECHO */
    (void)SPI2BUFL;  // Read and discard whatever the master sent
    SPI2BUFL = 0x00AA; // ALWAYS respond with 0xAA
    spi2_byte_received_flag = true;
    IFS2bits.SPI2IF = 0;
}

/*
 * @brief Initialise SPI2 in PIC24FJ256GA702
 * @param spi2_brg Baud Rate Generator for SPI2
 */
void SPI2_Slave_Init(void) {
    IEC2bits.SPI2IE = 0; // Disable interrupt for SPI2
    IFS2bits.SPI2IF = 0; // Clear interrupt flag
    
    SPI2CON1Lbits.SPIEN = 0;    // Turn off and resets module
    
    // PPS Configuration
    __builtin_write_OSCCONL(OSCCON & ~_OSCCON_IOLOCK_MASK); // Unlock PPS registers
    RPOR1bits.RP3R     = 10;    // SDO2 -> pin RP3/RB3
    RPINR22bits.SDI2R  = 2;     // SDI2 -> pin RP2/RB2
    RPINR22bits.SCK2R  = 13;    // SCK2IN -> pin RP13/RB13
    RPINR23bits.SS2R   = 5;     // SS2IN -> pin RP5/RB5
    __builtin_write_OSCCONL(OSCCON | _OSCCON_IOLOCK_MASK); // Lock PPS registers
    
    // SPI control register configuration
    SPI2STATL = 0;            // Clear status flags
    SPI2CON1L = 0;
    (void) SPI2BUFL;          // Clear buffer (cast to void)
    SPI2CON1Lbits.MSTEN  = 0; // Slave mode
    SPI2CON1Lbits.SSEN   = 1; // SS enabled
    SPI2CON1Lbits.MODE16 = 0; // 8 bit communication mode TEST - CHANGE TO 16 AFTER
    SPI2CON1Lbits.CKP    = 0;
    SPI2CON1Lbits.CKE    = 1;
    SPI2CON1H = 0; // SPI2CON1H: Control register 1 high (not used right now)
    
    // SPI2BUFL = 0xAA; // TEST ONLY!!!!
    
    IOCNBbits.IOCNB5 = 1; // Interrupt on negative edge for RB5
    IOCPBbits.IOCPB5 = 0; // Disable interrupt on positive for R5
    
    // SPI2 interrupt config
    IFS2bits.SPI2IF = 0;
    IPC8bits.SPI2IP = 2;
    IEC2bits.SPI2IE = 1;

    SPI2CON1Lbits.SPIEN = 1;    // Enable SPI module   
}

/**
 * @brief Processes the SPI slave state machine
 * @note Called in the main loop
 */
void SPI2_Slave_Process(void) {
    // Check if a new transaction has just started
    if (spi2_transaction_start_flag) {
        spi2_transaction_start_flag = false; // Consume the flag
        spi2_state = STATE_WAITING_FOR_ADDR; // Reset the state machine
    }

    // Check if a new byte has arrived
    if (spi2_byte_received_flag) {
        spi2_byte_received_flag = false; // Consume the flag

        switch (spi2_state) {
            case STATE_WAITING_FOR_ADDR:
            {
                bool is_write = (received_byte_from_isr & 0x80) != 0; // 0x80 = 10000000
                current_reg_addr = received_byte_from_isr & 0x7F; // 0x7F = 01111111

                if (current_reg_addr < 16) { 
                    if (is_write) {
                        spi2_state = STATE_WAITING_FOR_DATA_WRITE;
                    } else { 
                        SPI2BUFL = spi_registers[current_reg_addr];
                    }
                }
                break;
            }
            case STATE_WAITING_FOR_DATA_WRITE:
            {
                if (current_reg_addr < REG_ADC_RESULT_L) {
                    spi_registers[current_reg_addr] = received_byte_from_isr;
                }
                spi2_state = STATE_WAITING_FOR_ADDR;
                break;
            }
        }
    }
}
