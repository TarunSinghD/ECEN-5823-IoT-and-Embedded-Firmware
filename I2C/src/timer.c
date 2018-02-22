/********************************************************************************
Description
This file has the interrupt handler for TIMER0 interrupt the initialization of TIMER0

Author
Tarun Singh

Date
25 Sep 2017
********************************************************************************/

#include "timer.h"

void TIMER0_IRQHandler(void)
{
	CORE_ATOMIC_IRQ_DISABLE();

	/* Clear flag for TIMER0 overflow interrupt */
	TIMER_IntClear(TIMER0, TIMER_IF_OF);

	// Stop the timer
	TIMER0->CMD = TIMER_CMD_STOP;

	TIMER0->CNT = 0x00;

	unblockSleepMode(EM1);

	CORE_ATOMIC_IRQ_ENABLE();
}

void timer0_setup()
{
	CMU_ClockEnable(cmuClock_HFPER, true);
	CMU_ClockEnable(cmuClock_TIMER0, true);

	//Select TIMER0 parameters
	  TIMER_Init_TypeDef timerInit =
	  {
	    .enable     = false,
	    .debugRun   = true,
	    .prescale   = timerPrescale1024,
	    .clkSel     = timerClkSelHFPerClk,
	    .fallAction = timerInputActionNone,
	    .riseAction = timerInputActionNone,
	    .mode       = timerModeUp,
	    .dmaClrAct  = false,
	    .quadModeX4 = false,
	    .oneShot    = false,
	    .sync       = false,
	  };

	TIMER_IntEnable(TIMER0, TIMER_IEN_OF);
	NVIC_EnableIRQ(TIMER0_IRQn);

	// Set TIMER Top value - Top Value / (2MHz / 1024 (prescaler))
	// For 2 ms Seconds Top Value= 4
	TIMER_TopSet(TIMER0, 4);

	/* Configure TIMER */
	TIMER_Init(TIMER0, &timerInit);

	// Set it later
	NVIC_SetPriority(TIMER0_IRQn, 0);
}
