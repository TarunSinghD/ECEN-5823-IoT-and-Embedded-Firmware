/********************************************************************************
Description
This file has the interrupt handler for LETIMER0 interrupt the initialization of LETIMER0

Author
Tarun Singh

Date
25 Sep 2017
********************************************************************************/

#ifndef INC_TIMER_H_
#define INC_TIMER_H_

#include "adc.h"
#include "em_letimer.h"
#include "main.h"
#include "em_cmu.h"
#include "main.h"
#include "em_timer.h"
#include "sleep.h"

void timer0_setup();

#endif /* INC_TIMER_H_ */
