/********************************************************************************
Description
This file has the interrupt handler for LETIMER0 interrupt the initialization of LETIMER0

Author
Tarun Singh

Date
13 Sep 2017
********************************************************************************/

#include "letimer.h"
#include "sleep.h"
#include "gpio.h"
#include "main.h"
#include "em_i2c.h"

extern int previous_direction;
extern float ontime;
int prescalar = 1;
int frequency;
int comp0, comp1;
extern float temperatureC ;
extern float desired_temperature;
extern int enable;

// Function to automatically calculate the prescalar and the values to be assigned to
// COMP0 and COMP1 registers based on the user defined Period and Duty cycle
void calculate_register_values(void)
{
	// Need to choose the ULFRCO as clock source if minimum energy mode is EM3
	// ULFRCO = 1000 Hz
	if (LETIMER_ENERGY_MODE == EM3)
	{
		frequency = ULFRCO;
	}

	// If energy mode is higher than EM3, LFXO is the clock source
	else
	{
		frequency = LFXO;
	}

	// Calculate the value of the prescalar needed based on the clock source and period required
	while ((TOTAL_TIME * (frequency / prescalar)) > (LETIMER0_MAX_VAL))
	{
		prescalar = prescalar << 1;
	}

	// Calculate the values need to be assigned to the compare registers based on the period and duty cycle requirements
	comp0 = (TOTAL_TIME) * (frequency / prescalar);
	comp1 = (ontime) * (frequency / prescalar);
}

const LETIMER_Init_TypeDef letimerInit =
{
		.enable         = false,                //Start counting when init completed
		.debugRun       = false,                //Counter shall not keep running during debug halt
		.comp0Top       = true,                 //Load COMP0 register into CNT when counter underflows. COMP0 is used as TOP
		.bufTop         = false,                //Don't load COMP1 into COMP0 when REP0 reaches 0
		.out0Pol        = 0,                    //Idle value for output 0.
		.out1Pol        = 0,                    //Idle value for output 1.
		.ufoa0          = letimerUFOANone,      //PWM output on output 0
		.ufoa1          = letimerUFOANone,      //Pulse output on output 1
		.repMode        = letimerRepeatFree     //Count until stopped
};

void LETIMER0_IRQHandler(void)
{
	// Variable to temporarily store flags
	int intFlags;

	// Disable interrupts as we want to make this IRQ atomic
	CORE_ATOMIC_IRQ_DISABLE();

	// Read the interrupt flags
	intFlags = LETIMER_IntGet(LETIMER0);

	// Flags need to be cleared manually for the interrupt to be triggered again
	LETIMER_IntClear(LETIMER0, intFlags);

	// Reset to help debounce the Joystick
	previous_direction = RESET;

	// Interrupt caused because of COMP1?
//	if(intFlags & LETIMER_IF_COMP1)
//	{
//		led0(ON);
//	}
//
//	// Interrupt caused because of UF?
//	else if(intFlags & LETIMER_IF_UF)
//	{
//		led0(OFF);
//	}

	if (enable)
	{
		read_temperature();

		if ( desired_temperature > temperatureC )
		{
			led1(ON);
		}

		else
		{
			led1(OFF);
		}
	}

	//Re-enable interrupts
	CORE_ATOMIC_IRQ_ENABLE();
}

void letimer_setup()
{
	LETIMER_Init(LETIMER0, &letimerInit);

	calculate_register_values();

	/*Prescale the clock based on the frequency*/
	CMU_ClockDivSet(cmuClock_LETIMER0, prescalar);

	LETIMER_CompareSet(LETIMER0, 0, comp0);
	//LETIMER_CompareSet(LETIMER0, 1, comp1);

	while(LETIMER0->SYNCBUSY);

	LETIMER_IntClear(LETIMER0, LETIMER_IFC_UF | LETIMER_IFC_COMP0 | LETIMER_IFC_COMP1);
	LETIMER_IntEnable(LETIMER0 , LETIMER_IEN_UF);
	NVIC_EnableIRQ(LETIMER0_IRQn);

	/* Start LETIMER0*/
	LETIMER0->CMD = LETIMER_CMD_START;

	blockSleepMode(LETIMER_ENERGY_MODE);
}
