//***********************************************************************************
// Include files
//***********************************************************************************
#include "main.h"
#include "em_gpio.h"

//***********************************************************************************
// defined files
//***********************************************************************************

// LED0 pin Port F, Pin 4
/* PORT name	Port number
 * PORT A 		0
 * PORT B		1
 * PORT C		2
 * PORT D		3
 * PORT E		4
 * PORT F		5
 */
#define	LED0_port 5
#define LED0_pin 4
#define LED0_default	false 	// off
// LED1 pin PORT F, pin 5
#define LED1_port 5
#define LED1_pin 5
#define LED1_default	false	// off

//***********************************************************************************
// global variables
//***********************************************************************************


//***********************************************************************************
// function prototypes
//***********************************************************************************
void gpio_init(void);
