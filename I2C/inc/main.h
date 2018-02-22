/********************************************************************************
Description
This file includes the global variables and other included header files for main.c

Author
Tarun Singh

Date
13 Sep 2017
********************************************************************************/

/*Include guard */
#ifndef MAIN_H_
#define MAIN_H_

//***********************************************************************************
// Include files
//***********************************************************************************

#include <stdint.h>
#include <stdbool.h>
#include "em_core.h"
#include "em_cmu.h"
#include "em_gpio.h"
//#include "timer.h"
//#include "spi.h"

#define LETIMER_ENERGY_MODE 3
/* Please change on time in main.c, global variable called ontime.
 * Had to be a variable instead of #define because of mathematical operations being performed in ADC module
 * Egs: on_time = on_time + 0.5
 */
#define ON_TIME 			(0.2)
/*Define the TOTAL time here*/
#define TOTAL_TIME 			(2.5)
/*Define the maximum count for the 16 bit LETIMER0*/
#define LETIMER0_MAX_VAL 	65536
#define ULFRCO 				1000
#define LFXO	 			32768
#define DEFAULT_TEMPERATURE	15

typedef enum {
    EM0 = 0,
    EM1 = 1,
    EM2 = 2,
    EM3 = 3,
    EM4 = 4
} sleepstate_enum_e;

typedef enum
{
	OFF = 0,
	ON = 1
} led_status;

#endif
