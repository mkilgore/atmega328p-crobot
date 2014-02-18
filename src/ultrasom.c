/*
 * ultrasom.cpp
 *
 *  Created on: Jan 3, 2014
 *      Author: victor
 */
/********************************************************************

Example program to learn interfacing Ultra Sonic Range Finder Module
with AVR ATmega32 Microcontroller.

                                     NOTICE
                           --------
NO PART OF THIS WORK CAN BE COPIED, DISTRIBUTED OR PUBLISHED WITHOUT A
WRITTEN PERMISSION FROM EXTREME ELECTRONICS INDIA. THE LIBRARY, NOR ANY PART
OF IT CAN BE USED IN COMMERCIAL APPLICATIONS. IT IS INTENDED TO BE USED FOR
HOBBY, LEARNING AND EDUCATIONAL PURPOSE ONLY. IF YOU WANT TO USE THEM IN
COMMERCIAL APPLICATION PLEASE WRITE TO THE AUTHOR.

WRITTEN BY:
 *   AVINASH GUPTA
me@avinashgupta.com
 ********************************************************************/
#include "ultrasom.h"

/********************************************************************

This function measures the width of high pulse in micro second.

 ********************************************************************/
/*************************************************************************
Function: getPulseWidth
Purpose:  get the pulse width of the sonar
 **************************************************************************/
unsigned int getPulseWidth(void)
{
	long i = 0;
	unsigned int result = 0;

	//Wait for the rising edge
	for (i = 0; i < 600000; i++) {
		if (!(ULTRASOUND_SHIELD_PIN & (1 << ULTRASOUND1_ECHO))) {
			continue;
		} else {
			break;
		}
	}
	if (i == 600000) {
		return US_ERROR; 	//Indicates time out
	}
	//High Edge Found
	//Setup Timer1
	TCCR1A = 0X00;
	TIMER1_OC1A_OFF();
	TIMER1_OC1B_OFF();
	TCCR1B = (1 << CS11);   //Prescaler = Fcpu/8
	TCNT1 = 0x00; 			//Init counter

	//Now wait for the falling edge
	for (i = 0; i < 600000; i++) {
		if (ULTRASOUND_SHIELD_PIN & (1 << ULTRASOUND1_ECHO)) {
			if (TCNT1 > 60000) {
				break;
			} else {
				continue;
			}
		} else {
			break;
		}
	}

	if (i == 600000) {
		return US_ERROR; //Indicates time out
	}
	//Falling edge found
	result = TCNT1;
	//Stop Timer
	TCCR1B = 0x00;
	if (result > 60000) {
		return US_NO_OBSTACLE; //No obstacle
	} else {
		return (result >> 1);
	}
}

/*************************************************************************
Function: get_UltrasoundData
Purpose:  returns the Ultrasonic data sensor read in centimeters
Parameters: integer pointer to the variable you want to grab the distance in centimeters
 **************************************************************************/
//uint8_t get_UltrasoundData(void)
uint8_t get_UltrasoundData(int* distance_read)
{
	uint8_t distance_cm = 0;
	unsigned int resp;
	uint8_t obstaculo_sonar = 0;

	//Set Ultra Sonic Port as out
	//set trigger port as output
	ULTRASOUND_SHIELD_DDR |= _BV(ULTRASOUND1_TRIG);
	//turn trigger LOW
	ULTRASOUND_SHIELD_PORT &= ~_BV(ULTRASOUND1_TRIG);
	mydelay_us(2);
	//turn trigger HIGH
	ULTRASOUND_SHIELD_PORT |= _BV(ULTRASOUND1_TRIG);
	mydelay_us(10);
	//trigger pin turn back to LOW
	ULTRASOUND_SHIELD_PORT &= ~_BV(ULTRASOUND1_TRIG);

	//set echo as input
	ULTRASOUND_SHIELD_DDR &= ~_BV(ULTRASOUND1_ECHO);

	//Measure the width of pulse
	resp = getPulseWidth();

	//Handle Errors
	if (resp == US_ERROR) {
		//printf("ERROR. \n");
	} else if (resp == US_NO_OBSTACLE) {
		//printf("clear\r");
	} else {
		distance_cm = (resp / 58); //Convert to cm
		*distance_read = distance_cm;
		//printf("sonar_distance: %d\n", distance_cm);
	}
	/* distance value adjust by the user desire*/
	if (distance_cm >= 200 || distance_cm <= 0) {
		obstaculo_sonar = 2;
		//printf("out of distance\n");
	} else {
		if (distance_cm < 10) { /* 10 centimeters */
			obstaculo_sonar = 1;
			//printf("obstacle!\n");
		} else {
			obstaculo_sonar = 0;
		}
	}
	return obstaculo_sonar;
}

/*************************************************************************
Function: sweep_sonar
Purpose:  returns the Ultrasonic data sensor read in centimeters
Parameters: integer pointer to the variable you want to grab the distance in centimeters
 **************************************************************************/
uint8_t sweep_sonar(int* distance_read) {
	uint8_t i = 0;
	uint8_t obstaculo_sonar = 0;

	for (i=0; i <=25; i+=5) {
		obstaculo_sonar = sonar_rotate(5, 200, distance_read);
	}
	return obstaculo_sonar;
}
