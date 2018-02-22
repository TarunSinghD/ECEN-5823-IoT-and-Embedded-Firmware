/********************************************************************************
Description
This file has the initializations for clocks to various peripherals on the Blue gecko

Author
Tarun Singh

Date
13 Sep 2017
********************************************************************************/

//***********************************************************************************
// Include files
//***********************************************************************************
#include <cmu.h>

void cmu_init(void)
{
	/*Set the band for HFRCO*/
	CMU_HFRCOBandSet(cmuHFRCOFreq_19M0Hz);

	/*Disable autostart for HFXO*/
	CMU_HFXOAutostartEnable(0, false, false);

	/*Enable HFRCO*/
	CMU_OscillatorEnable(cmuOsc_HFRCO, true, true);

	// Auxilliary frequency for ADC
	CMU_AUXHFRCOBandSet(cmuAUXHFRCOFreq_1M0Hz);
	CMU_OscillatorEnable(cmuOsc_AUXHFRCO, true, true);
	CMU_ClockEnable(cmuClock_ADC0, true);

	// Auxilliary clock always on
	CMU->ADCCTRL = CMU_ADCCTRL_ADC0CLKSEL_AUXHFRCO;

	// High frequency tree will use HFRCO
	CMU_ClockSelectSet(cmuClock_HF,cmuSelect_HFRCO);

	/*With HFRCO turned ON, it is now safe to disable HFXO*/
	CMU_OscillatorEnable(cmuOsc_HFXO, false, false);

	// LETIMER0 clock setup
	if (LETIMER_ENERGY_MODE == EM3)
	{
		/*Using ULFRCO for EM3 energy mode for LETIMER0*/
		CMU_OscillatorEnable(cmuOsc_ULFRCO, true, true);
		CMU_ClockSelectSet(cmuClock_LFA,cmuSelect_ULFRCO);
	}
	else
	{
		/*Using LFXO when in EM0, EM1, EM2*/
		CMU_OscillatorEnable(cmuOsc_LFXO, true, true);
		CMU_ClockSelectSet(cmuClock_LFA,cmuSelect_LFXO);
	}

	/*Provide clock for peripherals*/
	CMU_ClockEnable(cmuClock_GPIO, true);
	CMU_ClockEnable(cmuClock_LETIMER0, true);
	CMU_ClockEnable(cmuClock_HFLE,true);

	// $[Peripheral Clock enables]
	/* Disable clock for GPCRC */
	CMU_ClockEnable(cmuClock_GPCRC, false);

	/* Disable clock for LDMA */
	CMU_ClockEnable(cmuClock_LDMA, false);

	/* Disable clock for PRS */
	CMU_ClockEnable(cmuClock_PRS, false);

	/* Disable clock for RTCC */
	CMU_ClockEnable(cmuClock_RTCC, false);

//	CMU_ClockEnable(cmuClock_EXPORT , false);
//	CMU_ClockEnable(cmuClock_BUS , false);
//	CMU_ClockEnable(cmuClock_CRYPTO, false);
//	CMU_ClockEnable(cmuClock_HFPER , false);
//	CMU_ClockEnable(cmuClock_USART0 , false);
//	CMU_ClockEnable(cmuClock_USART1 , false);
//	CMU_ClockEnable(cmuClock_RTCC, false);
//	CMU_ClockEnable(cmuClock_TIMER0 , false);
//	CMU_ClockEnable(cmuClock_TIMER1 , false);
//	CMU_ClockEnable(cmuClock_CRYOTIMER , false);
//	CMU_ClockEnable(cmuClock_IDAC0  , false);
//	CMU_ClockEnable(cmuClock_I2C0  , false);
//	CMU_ClockEnable(cmuClock_LEUART0  , false);
}

