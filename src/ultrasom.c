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
//#include "../header/LS_defines.h"
#include "LS_ATmega328.h"
#include <util/delay.h>
#include "ultrasom_stepper.h"
#include "myDelay.h"
#include "sensorIR.h"

/********************************************************************

Configuration Area.
UltraSonic (US) sensor connection.
in this example it is connected to PORTA bit 0
Adjust the following to connect is to different i/o
 ********************************************************************

This function measusers the width of high pulse in micro second.

 ********************************************************************/

//globals
//unsigned int resp;
//int distance;

//******************** Função utilizando a lib do Arduino *************************************

/* HC-SR04 Sensor
https://www.dealextreme.com/p/hc-sr04-ultrasonic-sensor-distance-measuring-module-133696
This sketch reads a HC-SR04 ultrasonic rangefinder and returns the
distance to the closest object in range. To do this, it sends a pulse
to the sensor to initiate a reading, then listens for a pulse
to return. The length of the returning pulse is proportional to
the distance of the object from the sensor.
The circuit:
 * VCC connection of the sensor attached to +5V
 * GND connection of the sensor attached to ground
 * TRIG connection of the sensor attached to digital pin PC2
 * ECHO connection of the sensor attached to digital pin PC1


Original code for Ping))) example was created by David A. Mellis
Adapted for HC-SR04 by Tautvidas Sipavicius

This example code is in the public domain.
 */

//*********** Função do sonar toda feita em C, sem a lib do Arduino ***************************

unsigned int getPulseWidth(void)
{
	unsigned int i = 0, result;

	//Wait for the rising edge
	for (i = 0; i < 600000; i++) {
		if (!(ULTRASOUND_SHIELD_PIN & (1 << ULTRASOUND1_ECHO))) {
			continue;
		} else {
			break;
		}
	}
	if (i == 600000) {
		return US_ERROR; //Indicates time out
	}
	//High Edge Found
	//Setup Timer1
	TCCR1A = 0X00;
	TIMER1_OC1A_OFF();
	TIMER1_OC1B_OFF();
	TCCR1B = (1 << CS11); //Prescaler = Fcpu/8
	TCNT1 = 0x00; //Init counter

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

unsigned int get_UltrasoundData(void)
{
	int distance_cm = 0;
	unsigned int resp;
	int obstaculo_sonar = 0;

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
		//fazer alguma logica aqui para retornar o erro ao prog principal
	} else if (resp == US_NO_OBSTACLE) {
		//printf("clear\r");
	} else {
		distance_cm = (resp / 58.0); //Convert to cm
		//printf("distancia sonar: %d\n", distance_cm);
	}
	/* valor de distancia ajustado de acordo com o desejo do usuario*/
	if (distance_cm >= 200 || distance_cm <= 0) {
		obstaculo_sonar = 2;
		//printf("out of distance\n");
	} else {
		if (distance_cm < 10) { /* 10 centimetros */
			obstaculo_sonar = 1;
			//printf("obstaculo!\n");
		} else {
			obstaculo_sonar = 0;
		}
	}
	return obstaculo_sonar;
}
//*********************************************************************************************

int sweep_sonar(void) {
	int i = 0;
	int obstaculo_sonar = 0;

	for (i=0; i <=25; i+=5) {
		obstaculo_sonar = sonar_rotate(5, .02);
	}
	return obstaculo_sonar;
}
