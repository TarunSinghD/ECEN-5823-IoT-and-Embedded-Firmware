/********************************************************************************
Description
This file has enum definitions and the prototypes for sleep functions

Author
Tarun Singh

Date
13 Sep 2017
********************************************************************************/
/*Include guard */
#ifndef SLEEP_H_
#define SLEEP_H_

/************************************************************************************
Name: sleep
Description : Based on the value of sleep block counter this function decides which
			  energy mode to sleep in
Inputs: N/A
Returns N/A
************************************************************************************/
void sleep(void);

/************************************************************************************
Name: blockSleepMode
Description : Blocks the sleep mode to a lowest possible energy mode where the
			  desired peripherals are still available
Inputs: The lowest possible energy mode where the desired peripheral is available
Returns N/A
************************************************************************************/
void blockSleepMode(sleepstate_enum_e minimumMode);

/************************************************************************************
Name: unblockSleepMode
Description : Unblocks the sleep mode so that system can go into deeper sleep modes
Inputs: The lowest possible energy mode where the system was blocked to allow deeper
		sleep modes
Returns N/A
************************************************************************************/
void unblockSleepMode(sleepstate_enum_e minimumMode);

#endif /* INC_SLEEP_H_ */
