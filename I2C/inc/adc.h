/********************************************************************************
Description
This file includes the prototypes for ADC related functions

Author
Tarun Singh

Date
13 Sep 2017
********************************************************************************/
#ifndef INC_ADC_H_
#define INC_ADC_H_

extern int comp1;
extern float ontime;
extern int previous_direction;

#include "em_adc.h"
#include "em_core.h"
#include "gpio.h"
#include "sleep.h"
#include "letimer.h"
#include "main.h"

#define ON_TIME_CHANGE 	0.5
#define ON_TIME_LIMIT 	2.8
#define ON_TIME_MIN 	0.2
#define UP_MAX 			3600
#define UP_MIN 			3400
#define DOWN_MAX 		2100
#define DOWN_MIN 		1900
#define RIGHT_MAX 		3200
#define RIGHT_MIN 		2950
#define LEFT_MAX 		2600
#define LEFT_MIN 		2300
#define RESET_MAX 		100
#define ADGT    		20
#define ADLT			3700

typedef enum
{
	UP,
	DOWN,
	RIGHT,
	LEFT,
	RESET
} direction_e;

void ADC_Setup();

void change_on_time(void);
void delay(void);

#endif /* INC_ADC_H_ */
