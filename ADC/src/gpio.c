/********************************************************************************
Description
This file includes the function to initialize the GPIO pin PortF pin 4 that is
connected to LED0 and set it to push pull mode with high drive strength

Author
Tarun Singh

Date
13 Sep 2017
********************************************************************************/

//***********************************************************************************
// Include files
//***********************************************************************************
#include "gpio.h"
#include "main.h"

//***********************************************************************************
// functions
//***********************************************************************************

void gpio_init(void)
{
	GPIO_DriveStrengthSet(LED0_port, gpioDriveStrengthStrongAlternateStrong);
	GPIO_PinModeSet(LED0_port, LED0_pin, gpioModePushPull, LED0_default);

	GPIO_DriveStrengthSet(LED1_port, gpioDriveStrengthStrongAlternateStrong);
	GPIO_PinModeSet(LED1_port, LED1_pin, gpioModePushPull, LED1_default);

	GPIO_PinModeSet(ADC0_port, ADC0_pin, gpioModeDisabled, ADC0_default);
	/*Over voltage disable for analog pin PortA pin 0*/
	GPIO->P[gpioPortA].OVTDIS = 0x0001;
}

void led0(led_status status)
{
	if (status == ON)
	{
		GPIO_PinOutSet(LED0_port, LED0_pin);
	}

	else if (status == OFF)
	{
		GPIO_PinOutClear(LED0_port, LED0_pin);
	}
}

void led1(led_status status)
{
	if (status == ON)
	{
		GPIO_PinOutSet(LED1_port, LED1_pin);
	}

	else if (status == OFF)
	{
		GPIO_PinOutClear(LED1_port, LED1_pin);
	}
}
