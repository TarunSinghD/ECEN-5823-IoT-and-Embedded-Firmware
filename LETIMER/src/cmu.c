//***********************************************************************************
// Include files
//***********************************************************************************
#include "cmu.h"
#include "em_letimer.h"

//***********************************************************************************
// defined files
//***********************************************************************************
#define COMP1 410
#define COMP0 20480

//***********************************************************************************
// global variables
//***********************************************************************************


//***********************************************************************************
// function prototypes
//***********************************************************************************
void cmu_init(void){
	// By default, HFRCO is enabled  cmuHFRCOFreq_19M0Hz
	CMU_HFRCOBandSet(cmuHFRCOFreq_19M0Hz); 				// Set HFRCO frequency
	CMU_OscillatorEnable(cmuOsc_HFXO, false, false);	// Disable HFXO

	// By default, LFRCO is enabled
	CMU_OscillatorEnable(cmuOsc_LFXO, false, false);		// Disable LFXO

	// Peripheral clocks enabled
	CMU_ClockEnable(cmuClock_GPIO, true);
}

void letimer0_init(void)
{
	CMU_OscillatorEnable(cmuOsc_LFXO, true, true);

	CMU_ClockSelectSet(cmuClock_LFA, cmuSelect_LFXO);

	// Dividing 32 kHz clock by 4
	CMU_ClockDivSet(cmuClock_LFA, 4);

	/* Enable necessary clocks */

	CMU_ClockEnable(cmuClock_CORELE, true);
	CMU_ClockEnable(cmuClock_LETIMER0, true);
	CMU_ClockEnable(cmuClock_GPIO, true);

	LETIMER_CompareSet(LETIMER0, 0, COMP0);
	LETIMER_CompareSet(LETIMER0, 1, COMP1);

#if 0
	/* Set configurations for LETIMER 0 */
	const LETIMER_Init_TypeDef letimerInit =
	{
	.enable         = true,                   /* Start counting when init completed. */
	.debugRun       = false,                  /* Counter shall not keep running during debug halt. */
	.rtcComp0Enable = false,                  /* Don't start counting on RTC COMP0 match. */
	.rtcComp1Enable = false,                  /* Don't start counting on RTC COMP1 match. */
	.comp0Top       = true,                   /* Load COMP0 register into CNT when counter underflows. COMP0 is used as TOP */
	.bufTop         = false,                  /* Don't load COMP1 into COMP0 when REP0 reaches 0. */
	.out0Pol        = 0,                      /* Idle value for output 0. */
	.out1Pol        = 0,                      /* Idle value for output 1. */
	.ufoa0          = letimerUFOAPwm,         /* PWM output on output 0 */
	.ufoa1          = letimerUFOAPulse,       /* Pulse output on output 1*/
	.repMode        = letimerRepeatFree       /* Count until stopped */
	};
#endif

	// Clear previous flags if any
	LETIMER0->IFC = 0x00;

	/* Enable underflow interrupt */
	LETIMER_IntEnable(LETIMER0, LETIMER_IF_UF);

	/* Enable comp0 interrupt */
	LETIMER_IntEnable(LETIMER0, LETIMER_IF_COMP0);

	// Wait for the LETIMER0 synch bit is cleared before proceeding by accessing the register LETIMER0->SYNCBUSY
	while (LETIMER0->SYNCBUSY);

	// Make COMP0 value as the top value
	LETIMER0->CTRL |= LETIMER_CTRL_COMP0TOP;

	NVIC_EnableIRQ(LETIMER0_IRQn);
}

