/********************************************************************************
Description
This file has the interrupt handler for USART1 interrupt the initialization of USART1

Author
Tarun Singh

Date
26 Sep 2017
********************************************************************************/

#include "spi.h"
#define READ 1
#define WRITE 0

void spi_write_data(uint8 data, uint8 address)
{
	uint16 data_t;

	while (!(USART1->STATUS & USART_STATUS_TXBL));

	// 0 : Write
	// 1 : Read
	data_t = ((uint16)(data << 8)| address) & ~(1 << 8);
	USART1->TXDOUBLE = data_t;

	// Wait for transmission to complete
	while (!(USART1->STATUS & USART_STATUS_TXC));

	// Clearing the Rx buffer
	USART1->CMD |= _USART_CMD_CLEARRX_MASK;
	USART1->CMD &= ~(_USART_CMD_CLEARRX_MASK);
}

uint8 spi_read_data(uint8 address)
{
	uint16 dummy_data;
	uint8 received_data;

	dummy_data = (address << 8);
	USART1->TXDOUBLE = dummy_data | (1 << 15);

	received_data = USART1->RXDOUBLE;
	received_data = received_data >> 8;

	return received_data;
}

void SPI_Init1(void){
	USART_InitSync_TypeDef spi_init;

	GPIO_Mode_TypeDef gpioModeMosi;
	GPIO_Mode_TypeDef gpioModeMiso;
	GPIO_Mode_TypeDef gpioModeCs;
	GPIO_Mode_TypeDef gpioModeClk;

	/* Set GPIO config to master */
    gpioModeMosi = gpioModePushPull;
    gpioModeMiso = gpioModeInput;
    gpioModeCs   = gpioModePushPull;
    gpioModeClk  = gpioModePushPull;

	//SPI pins for BMA280 are mapped to the ports as follows
	//SPI_CS	PC9
	//SPI_SCLK	PC8
	//SPI_MISO	PC7
	//SPI_MOSI	PC6
	//setting these pins to appropriate GPIO mode.
    GPIO_PinModeSet(gpioPortC, 9, gpioModeCs,1);
	GPIO_PinModeSet(gpioPortC, 8, gpioModeClk,1);
	GPIO_PinModeSet(gpioPortC, 7, gpioModeMiso,0);
	GPIO_PinModeSet(gpioPortC, 6, gpioModeMosi,1);

	CMU_ClockEnable(cmuClock_HFPER, true);
	CMU_ClockEnable(cmuClock_USART1, true);


	spi_init.enable = usartDisable;		//Enable USART after init
	spi_init.refFreq = 0;
	spi_init.baudrate = 100000;			//set the baudrate to 10Mhz
	spi_init.databits =usartDatabits8;	//set 8 bit transmit mode
	spi_init.master = 1;				//set the device as master
	spi_init.msbf= 1; 					//send lsb first;
	spi_init.clockMode = usartClockMode3;
	spi_init.prsRxEnable = 0;
	spi_init.prsRxCh = usartPrsRxCh0;
	spi_init.autoTx = 0;
	spi_init.autoCsEnable = 1;			//Enable auto chip select
	spi_init.autoCsHold = 0;
	spi_init.autoCsSetup = 0;

	USART_InitSync(USART1, &spi_init);
	USART1->ROUTELOC0 = USART_ROUTELOC0_CLKLOC_LOC11 | USART_ROUTELOC0_TXLOC_LOC11 | USART_ROUTELOC0_RXLOC_LOC11 | USART_ROUTELOC0_CSLOC_LOC11;
	USART1->ROUTEPEN = USART_ROUTEPEN_CLKPEN | USART_ROUTEPEN_TXPEN | USART_ROUTEPEN_RXPEN | USART_ROUTEPEN_CSPEN;

	USART_Enable(USART1, usartEnable);
	//The accelerometer interrupts are connected to GPIO port D11
}

void bma280_setup()
{
	// Range +- 4g
	spi_write_data(0x05, 0x0F);

	// Bandwidth 125 Hz
	spi_write_data(0x0C, 0x10);

	// Single tap interrupt enable
	spi_write_data(0x30, 0x16);

	// Single tap interrupt flag
	spi_write_data(0x30, 0x19);

	// Tap quiet 30 ms, Tap shock 50 ms, Tap duration 200 ms
	spi_write_data(0x03, 0x2A);

	// Tap samples 4, Tap threshold 250 mg
	spi_write_data(0x48, 0x2B);

	// Start with suspend mode
	spi_write_data(0x80, 0x11);
}
