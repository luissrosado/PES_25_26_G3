/*
 * File:   adc.c
 * Author: simas
 *
 * Created on 26 September 2025, 20:27
 */


#include "xc.h"
#include "adc.h"

/*
 * @brief Initialise the ADC module for single ended manual trigger conversions
 */
void ADC_Init(void) {
    AD1CON1bits.ADON = 0; // ADC OFF
    
    // Select voltage reference source
    AD1CON2 = 0; // Use AVDD and AVSS
    
    // Select positive and negative MUX inputs for each channel
    AD1CHS0 = 0; // Start from clear
    
    // Select analog conversion clock (PIC TABLE 32-35 for minimum AD50)
    AD1CON3bits.ADCS = 0b000000010; // TAD = 3*TCY
    
    // Select sample/conversion sequence
    AD1CON1bits.SSRC = 0b0111;  // Auto-convert
    AD1CON3bits.SAMC = 0b00001; // 1 * TAD
    
    // Select how conversion results are presented in the buffer
    AD1CON1bits.FORM = 0b00; // Integer (absolute decimal)
    
    // Manual mode sampling (for now)
    AD1CON1bits.SAMP = 0; // Auto sampling off
    
    // Select 12 bit mode
    AD1CON1bits.MODE12 = 1;
    
    // Explicity disable all auto scan and compare features
    //AD1CON5 = 0;
    
    AD1CON1bits.ADON = 1; // ADC ON
}

/*
 * @brief Read a conversion from a specified ADC channel
 * @param channel The analog channel number to read
 * @return 12 bit digital result of the conversion
 */
uint16_t ADC_Read(uint8_t channel) {
    AD1CHS0bits.CH0SA = channel; // Select the channel
    AD1CON1bits.SAMP  = 1;       // Start sampling
    
    // Small delay to allow the sample and hold capacitor to charge
    // TODO: Change when adding the sensor, PIC24 specifies the time!
    for (int i = 0; i < 100; i++) Nop();
    
    AD1CON1bits.SAMP = 0;       // Stop sampling
    while (!AD1CON1bits.DONE);  // Wait for the conversion to finish
    
    return ADC1BUF0;
}
