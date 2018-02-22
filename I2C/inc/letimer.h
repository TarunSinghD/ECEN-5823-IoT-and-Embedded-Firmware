/********************************************************************************
Description
This file has the interrupt handler for LETIMER0 interrupt the initialization of LETIMER0

Author
Tarun Singh

Date
13 Sep 2017
********************************************************************************/
/*Include guard */
#ifndef LETIMER_H_
#define LETIMER_H_

#include "adc.h"
#include "em_letimer.h"
#include "main.h"
#include "em_cmu.h"

extern int ON_period, TOTAL_period, Prescaler, CurrentFrequency;

/************************************************************************************
Name: le_timer_init
Description : Initializes the LETIMER0 and loads the appropriate values in the
			  corresponding registers
Inputs: N/A
Returns N/A
************************************************************************************/

void letimer_setup();

void calculate_register_values(void);

#endif /* INC_LETIMER_H_ */
