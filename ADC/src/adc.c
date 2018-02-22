/********************************************************************************
Description
This file has the interrupt handler for ADC0 interrupt the initialization of ADC0

Author
Tarun Singh

Date
13 Sep 2017
********************************************************************************/
#include <adc.h>

// Function to change the on_time of the LED dynamically as the Joystick keys are pressed
void change_on_time(void)
{
	calculate_register_values();
	LETIMER_CompareSet(LETIMER0, 1, comp1);
}

// Interrupt Handler for ADC
void ADC0_IRQHandler(void)
{
	// Variable to temporarily store flags
	uint32_t intFlags;

	// Disable interrupts as we want to make this IRQ atomic
	CORE_ATOMIC_IRQ_DISABLE();

	// Read the interrupt flags
	intFlags = ADC_IntGet(ADC0);

	// Read the value sampled by the ADC
	uint32_t ADC_data = ADC_DataSingleGet(ADC0);

	// Flags need to be cleared manually for the interrupt to be triggered again
	ADC_IntClear(ADC0, intFlags);

	// Was the interrupt triggered because of the sample being within the window of interest?
	if(intFlags & ADC_IF_SINGLECMP)
	{
		// UP button pressed on the Joystick?
		if ((ADC_data > UP_MIN) && (ADC_data < UP_MAX))
		{
			// Turn ON LED1
			led1(ON);
		}

		// Down button pressed on the Joystick?
		else if (ADC_data > DOWN_MIN && ADC_data < DOWN_MAX)
		{
			// TURN OFF LED1
			led1(OFF);
		}

		// Left button pressed on the Joystick?
		else if (ADC_data > LEFT_MIN && ADC_data < LEFT_MAX)
		{
			// If left button is pressed decrease on time by 500 ms
			// Keep track of the previous button pressed which will help debounce the Joystick
			if (previous_direction != LEFT)
			{
				ontime = ontime - ON_TIME_CHANGE;

				// on time should never be less than 200 ms, limit it
				if (ontime <= ON_TIME_MIN)
				{
					ontime = ON_TIME_MIN;
				}

				// change appropriate registers in LETIMER0 to accommodate the change in on time
				change_on_time();
				previous_direction = LEFT;
			}
		}

		// Right button pressed on the Joystick?
		else if(ADC_data > RIGHT_MIN && ADC_data < RIGHT_MAX)
		{
			// If right button is pressed increase on time by 500 ms
			// Keep track of the previous button pressed which will help debounce the Joystick
			if (previous_direction != RIGHT)
			{
				ontime = ontime + ON_TIME_CHANGE;

				// on time should never be greater than 2.8 sec, limit it
				if (ontime >= ON_TIME_LIMIT)
				{
					ontime = ON_TIME_LIMIT;
				}

				// change appropriate registers in LETIMER0 to accommodate the change in on time
				change_on_time();
				previous_direction = RIGHT;
			}
		}

		// Center button pressed on Joystick?
		else if (ADC_data < RESET_MAX)
		{
			ontime = ON_TIME_MIN;

			// change appropriate registers in LETIMER0 to accommodate the change in on time
			change_on_time();
			previous_direction = RESET;
		}
	}

	CORE_ATOMIC_IRQ_ENABLE();
}

// Function to setup the ADC according to the requirement
void ADC_Setup()
{
	ADC_Init_TypeDef Init =
	{
			.ovsRateSel = adcOvsRateSel2,
			.prescale = _ADC_CTRL_PRESC_DEFAULT,
			.warmUpMode = adcWarmupNormal,
			.timebase = ADC_TimebaseCalc(0),
			.tailgate = false,
			.em2ClockConfig = adcEm2ClockAlwaysOn
	};

	ADC_InitSingle_TypeDef singleInit =
	{
			.acqTime = adcAcqTime32,
			.reference = adcRefVDD,
			.resolution = adcRes12Bit,
			.posSel = adcPosSelAPORT3XCH8,
			.negSel=adcNegSelVSS,
			.diff = false,
			.prsEnable = false,
			.leftAdjust = false ,
			.rep = true,
			.singleDmaEm2Wu = false,
			.fifoOverwrite = true
	};

	ADC_Init(ADC0, &Init);
	ADC_InitSingle(ADC0, &singleInit);

	// Set the upper and lower thresholds for the window compare function
	// If the sampled ADC value is within this window then an interrupt will be triggered
	ADC0->CMPTHR = _ADC_CMPTHR_RESETVALUE;
	ADC0->CMPTHR = (ADGT << _ADC_CMPTHR_ADGT_SHIFT) + (ADLT << _ADC_CMPTHR_ADLT_SHIFT);

	// Enable compare logic
	ADC0->SINGLECTRL |=  ADC_SINGLECTRL_CMPEN;

	// Clear all interrupt flags
	ADC_IntClear(ADC0, ADC_IFC_PROGERR | ADC_IFC_VREFOV | ADC_IFC_SCANCMP | ADC_IFC_SINGLECMP | ADC_IFC_SCANUF | ADC_IFC_SINGLEUF | ADC_IFC_SCANOF | ADC_IFC_SINGLEOF);

	// Enable interrupt to be triggered on single compare
	ADC_IntEnable(ADC0, ADC_IEN_SINGLECMP );

	// Define the lowest energy mode that ADC module can run in
	blockSleepMode(EM3);

	NVIC_EnableIRQ(ADC0_IRQn);

	// Start the ADC
	ADC_Start(ADC0, adcStartSingle);

	CORE_ATOMIC_IRQ_ENABLE();
}
