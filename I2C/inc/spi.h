/********************************************************************************
Description
This file has the interrupt handler for USART1 interrupt the initialization of USART1

Author
Tarun Singh

Date
26 Sep 2017
********************************************************************************/

#ifndef INC_SPI_H_
#define INC_SPI_H_

#include "adc.h"
#include "em_letimer.h"
#include "main.h"
#include "em_cmu.h"
#include "main.h"
#include "em_timer.h"
#include "em_usart.h"
#include "bg_types.h"

void spi_setup();
void spi_write_data(uint8 data, uint8 address);
uint8 spi_read_data(uint8 address);
void SPI_Init1(void);
void bma280_setup();

#endif /* INC_SPI_H_ */
