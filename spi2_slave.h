#ifndef SPI2_SLAVE_H
#define SPI2_SLAVE_H

#include "system_config.h"
#include "xc.h"
#include <stdbool.h>
#include <stdint.h>

// Read only registers
#define REG_ADC_RESULT_L    0x10

// Public variables
extern volatile uint8_t spi_registers[16];
extern volatile bool spi2_transaction_start_flag;
extern volatile bool spi2_byte_received_flag;
extern volatile uint8_t spi2_received_data;

void SPI2_Slave_Init();

void SPI2_Slave_Process(void);

#endif
