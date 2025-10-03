/*
 * File:   main.c
 * Author: simas
 *
 * Created on 09 September 2025, 17:38
 */

/* TODO:
 *  - ADF4351_LD_TRIS está ligado ao PDRF, trocar para o correto
 */

#include "system_config.h"
#include "xc.h"
#include <stdbool.h>
#include "adc.h"
#include "adf4351.h"
#include "spi2_slave.h"
#include <libpic30.h> // For __delay_xx() functions

// Configuration bits
#pragma config FNOSC    = FRC // Fast RC oscillator
#pragma config FWDTEN   = OFF // Disable watchdog timer
#pragma config JTAGEN   = OFF // Disable JTAG port (shared pin with ICSP)

// Global variables (for main <-> ISR communication)
volatile bool timer_flag = false;
volatile uint8_t seconds_counter = 0;

/*
 * @brief Activates the timer flag and resets the interrupt flag
 * @param no_auto_psv promise the ISR will not require Program Space Visibility PSV
 * @note Fires exactly every 2 seconds
 */
void __attribute__((__interrupt__, no_auto_psv)) _T1Interrupt(void) {
    seconds_counter++;
    if (seconds_counter >= 5) { // Every 5 * 2 = 10 seconds
        seconds_counter = 0;
        timer_flag = true;      // Signal to main 10 seconds have passed
    }
    IFS0bits.T1IF = 0; // Clear Timer1 interrupt flag
}

/*
 * @brief IOC interrupt for SPI2 SS pin
 * @note Fires on falling edge of SS to signal the start of a new transition
 */
void __attribute__((__interrupt__, no_auto_psv)) _IOCInterrupt(void) {
    // Check if it was the SS pin
    if (IOCFBbits.IOCFB5 == 1) {
        spi2_transaction_start_flag = true; // Flag for main
        IOCFBbits.IOCFB5 = 0; // Reset pin flag
    }
    IFS1bits.IOCIF = 0; // Reset master flag
}

/*
 * @brief Initialise Timer1
 */
void Timer1_Init(void) {
    T1CON   = 0; // Stop timer and reset control register
    TMR1    = 0; // Clear content of timer register
    
    // Set timer and interrupt values
    T1CONbits.TCKPS = 0b11;     // Pre scaler to 1:256
    T1CONbits.TCS   = 0;        // Clock source to FCY
    PR1             = TIMER1_PERIOD; // 2 seconds
    IPC0bits.T1IP   = 0x01;     // Timer1 interrupt priority level
    IFS0bits.T1IF   = 0;        // Clear Timer1 interrupt status flag
    IEC0bits.T1IE   = 1;        // Enable Timer1 interrupts
    
    T1CONbits.TON   = 1;    // Start timer
}

/*
 * @brief Initialise IO peripherals of PIC24FJ256GA702
 */
void IO_Init(void) {
    // Configure all ports to digital mode
    ANSELA = 0;
    ANSELB = 0;
    
    // Configure directions
    LOCK_LED_TRIS    = 0; // RB15 output for the Lock LED
    ADF4351_LE_TRIS  = 0; // RB6 output for Latch Enable (LE)
    ADF4351_CE_TRIS  = 0; // RB11 output for CE (Slave Select)
    TRISBbits.TRISB7 = 0; // RB7/RP7 output for SCK1
    TRISBbits.TRISB8 = 0; // RB8/RP8 output for SDO1
    
    ADF4351_LD_TRIS   = 1;   // RB9/RP9 input for Lock Detect
    TRISBbits.TRISB10 = 1;   // RB10/RP10 input SDI1 (not used)
    
    // SPI2
    TRISBbits.TRISB2 = 1;   // SDI2
    TRISBbits.TRISB3 = 0;   // SDO2
    TRISBbits.TRISB13 = 1;   // SCK2
    TRISBbits.TRISB5 = 1;   // SS2
    
    // Initial states
    LOCK_LED_LAT = 0;       // LED off
    ADF4351_LE_LAT = 0;     // Latch Enable (LE)
    ADF4351_CE_LAT = 1;     // Chip Enable (CE) to active high
    LATBbits.LATB5 = 1;     // SS2 active high
    
    // Analog pins configuration
    ANSELBbits.ANSB14 = 1; // RB14/AN6 to analog
    TRISBbits.TRISB14 = 1; // input
    
    // IoC interrupt config
    IFS1bits.IOCIF = 0;
    IEC1bits.IOCIE = 1;
    IPC4bits.IOCIP = 3;
}

/*
 * @brief Initialise SPI1 in PIC24FJ256GA702
 * 
 * Map with Table 11-6 and 11-7
 */
void SPI1_Init(void) {
    IEC0bits.SPI1IE = 0; // Disable interrupt for SPI1
    
    SPI1CON1Lbits.SPIEN = 0;    // Turn off and resets module
    
    // PPS Configuration
    __builtin_write_OSCCONL(OSCCON & ~_OSCCON_IOLOCK_MASK); // Unlock PPS registers
    RPINR20bits.SDI1R = 10;  // SDI1 -> pin RP10/RB10
    RPOR3bits.RP7R  = 8;     // SCK1 -> pin RP7/RB7
    RPOR4bits.RP8R  = 7;     // SDO1 -> pin RP8/RB8
    RPOR5bits.RP11R = 9;     // SS1OUT -> pin RP11/RB11
    __builtin_write_OSCCONL(OSCCON | _OSCCON_IOLOCK_MASK); // Lock PPS registers
    
    // SPI control register configuration
    SPI1STATL = 0;            // Clear status flags
    (void) SPI1BUFL;          // Clear buffer (cast to void)
    SPI1CON1Lbits.MSTEN  = 1; // Master mode enable
    SPI1CON1Lbits.MODE16 = 1; // 16 bit communication mode
    SPI1CON1Lbits.CKP    = 0; // Clock polarity, idle at low level
    SPI1CON1Lbits.CKE    = 0; // Clock edge, TX happens from idle -> active CLK
    SPI1CON1Lbits.ENHBUF = 0; // Disable enhanced buffer (normal mode)
    SPI1CON1H = 0; // SPI1CON1H: Control register 1 high (not used right now)
    
    SPI1BRGL = SPI1_BRG_VALUE;  // Baud rate
    SPI1CON1Lbits.SPIEN = 1;    // Enable SPI module
}

/*
 * @brief main function of the code
 */
int main(void) {
    bool pll_locked = false;
    uint16_t power_reading = 0; // Store sensor value
    //spi_registers[REG_PLL_LOCK_STATUS] = 0xDE; // TEST 3: REGISTER READ
    
    // System initialisation
    IO_Init();
    SPI1_Init();
    SPI2_Slave_Init();
    ADC_Init();
    //Timer1_Init();
    //ADF4351_Hardware_Reset();
    
    __builtin_enable_interrupts(); // Enable global interrupts
    
    LOCK_LED_LAT = 1;
    __delay_ms(250);
    LOCK_LED_LAT = 0;
    
    // Power up sequence for ADF4351 synth
    for (int i = 0; i < 6; i++) {
        ADF4351_Write_Register(adf4351_reg_values[i]);
    }   
    
    while (ADF4351_LD_PORT == 0) Nop(); // Wait for PLL to lock with LD pin
    __delay_ms(100);
    
    if (ADF4351_LD_PORT == 1) {
        // SUCCESS: The PLL is locked, turns on LED
        pll_locked = true;
        power_reading = ADC_Read(6); // Read from AN6/RP14
        LOCK_LED_LAT = 1;
    }
    
    while(1) {
        // Do nothing, wake up on interrupt
        Idle();
        
        // Periodic timer fire
        if (timer_flag) {
            /*LOCK_LED_LAT = 1;
            __delay_ms(500);
            LOCK_LED_LAT = 0;
            __delay_ms(500);*/
            timer_flag = false; // Consume the flag
            // TODO: Handle periodic tasks if needed
        } 
        
        // SPI2 event occurred (start of transaction or byte received)
        if (spi2_transaction_start_flag || spi2_byte_received_flag) {
            LOCK_LED_LAT = 1;
            SPI2_Slave_Process();
            // TODO: Save the values form master to program the synths
        }
    }
}
