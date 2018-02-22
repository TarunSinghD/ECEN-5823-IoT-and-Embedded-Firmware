/*
 * i2c.c
 *
 *  Created on: Oct 7, 2017
 *      Author: tarun
 */

#include "i2c.h"
#include "timer.h"
extern float temperatureC ;
int enable = 0;

void delay_80ms(void)
{
	CORE_ATOMIC_IRQ_ENABLE();

	// Set TIMER Top value - Top Value / (2MHz / 1024 (prescaler))
	// For 80 ms Seconds Top Value= 157
	TIMER_TopSet(TIMER0, 157);

	//Start Timer
	TIMER0->CMD = TIMER_CMD_START;
	//TIMER0->CNT = 0x00;

	blockSleepMode(EM1);
}

float read_temperature()
{
	uint16_t read_data;

	// The read sequence
	// S SA W		CMD		SR SA R		SR SA R		SR SA R			A		NA P
	//			A		A			NA			NA....		A	MSB		LSB

	I2C0->TXDATA = (SI7021_ADDR << 1) | 0x00 ;//start with write
	//send the START bit
	I2C0->CMD = I2C_CMD_START;
	while ((I2C0->IF & I2C_IF_ACK) == 0);
	I2C0->IFC = I2C_IFC_ACK;


	I2C0->TXDATA = 0xE3;
	I2C0->CMD = I2C_CMD_START;
	while ((I2C0->IF & I2C_IF_ACK) == 0);
	I2C0->IFC = I2C_IFC_ACK;

	I2C0->TXDATA = (0x40 << 1) | 0x01 ;//read
	I2C0->CMD = I2C_CMD_START;
	while ((I2C0->IF & I2C_IF_ACK) == 0);
	I2C0->IFC = I2C_IFC_ACK;
	//	I2C0->CMD = I2C_CMD_ACK;

	while((I2C0->IF & I2C_IF_RXDATAV) ==0);
	read_data = I2C0->RXDATA;
	read_data = read_data<<8;              //receive MSB
	I2C0->CMD = I2C_CMD_ACK;
	while((I2C0->IF & I2C_IF_RXDATAV) ==0);
	read_data = read_data | I2C0->RXDATA;

	I2C0->CMD = I2C_CMD_NACK;
	I2C0->CMD = I2C_CMD_STOP;

	temperatureC = (175.72 * read_data ) / 65536 - 46.85;

	return temperatureC;
}

void i2c_init(void){
	//i2c is on PC11 and PC 10 on location 16
	I2C_Init_TypeDef i2c_init = I2C_INIT_DEFAULT;

	CMU_ClockEnable(cmuClock_I2C0, true);

	//set gpio pin modes
	GPIO_PinModeSet(gpioPortC, SCL_PIN, gpioModeWiredAnd, 1);//pull up SCL
	GPIO_PinModeSet(gpioPortC, SDA_PIN, gpioModeWiredAnd, 1);//SDA

	i2c_init.clhr = i2cClockHLRStandard;
	i2c_init.master = true;
	i2c_init.refFreq = 0;
	i2c_init.freq = 1000000; //check the datasheet
	i2c_init.enable = false;//

	//route the pins to respective location
	I2C0 -> ROUTELOC0 = I2C_ROUTELOC0_SDALOC_LOC16 |I2C_ROUTELOC0_SCLLOC_LOC14;
	I2C0 -> ROUTEPEN = I2C_ROUTEPEN_SCLPEN | I2C_ROUTEPEN_SDAPEN;

	//init I2C
	I2C_Init(I2C0, &i2c_init);

	//clear buffers in I2C slave
	for (int i=0; i<9; i++){
		GPIO_PinOutClear(gpioPortC, SCL_PIN);
		GPIO_PinOutSet(gpioPortC, SCL_PIN);
	}

	//reset the I2C bus
	if(I2C0->STATE & I2C_STATE_BUSY){
		I2C0->CMD = I2C_CMD_ABORT;
	}
}

void i2c_enable(void)
{
	GPIO_PinOutSet(gpioPortD, 9);
	delay_80ms();

	i2c_init();
	enable = 1;
	I2C_Enable(I2C0, true);
}

void load_power_enable(void)
{
	GPIO_PinOutSet(gpioPortD, 9);
	delay_80ms();

	GPIO_PinModeSet(gpioPortC, SCL_PIN, gpioModeWiredAnd, 1);//pull up SCL
	GPIO_PinModeSet(gpioPortC, SDA_PIN, gpioModeWiredAnd, 1);//SDA

	//clear buffers in I2C slave
	for (int i=0; i<9; i++)
	{
		GPIO_PinOutClear(gpioPortC, 10);
		GPIO_PinOutSet(gpioPortC, 10);
	}

	//reset the I2C bus
	if(I2C0->STATE & I2C_STATE_BUSY)
	{
		I2C0->CMD = I2C_CMD_ABORT;
	}
	enable = 1;
	I2C_Enable(I2C0, true);
}

void load_power_disable(void)
{
	enable = 0;
	GPIO_PinModeSet(gpioPortC, SCL_PIN, gpioModeDisabled, 1);//pull up SCL
	GPIO_PinModeSet(gpioPortC, SDA_PIN, gpioModeDisabled, 1);//SDA

	I2C_Enable(I2C0, false);
	GPIO_PinOutClear(gpioPortD, 9);
	delay_80ms();
}

