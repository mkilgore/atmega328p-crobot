/*
 * ultrasom_stepper.cpp
 *
 *  Created on: Jan 4, 2014
 *      Author: victor
 */
//////////////////////////////////////////////////////////////////
//©2011 bildr
//Released under the MIT License - Please reuse change and share
//Using the easy stepper with your arduino
//use rotate and/or rotateDeg to controll stepper motor
//speed is any number from .01 -> 1 with 1 being fastest -
//Slower Speed == Stronger movement
/////////////////////////////////////////////////////////////////


#include  "ultrasom_stepper.h"

/*************************************************************************
Function: set_stepper_zero
Purpose:  moves the motor to the zero_position, in this case, when it hits my switch
 **************************************************************************/
void set_stepper_zero(void) {
	uint8_t zero_position = HIGH;
	uint8_t i = 0;
	for (i=0; i<=50;i+=5) {
		zero_position = tst_bit(FIM_DE_CURSO_PIN, FIM_DE_CURSO);
		//printf("chave_fim_de_curso: %d\r", zero_position);
		if (zero_position == HIGH) {
			obstacleAlarm();
			break;
		} else {
			rotate(-5, .02); //reverse
			//mydelay_us(10);
		}
	}
}

/*************************************************************************
Function: rotate
Purpose:  rotates the motor with the desired number of steps and speed
 **************************************************************************/
void rotate(int steps, float speed){

	//rotate a specific number of microsteps (8 microsteps per step) - (negative for reverse movement)
	//speed is any number from .01 -> 1 with 1 being fastest - Slower is stronger
	if (steps > 0) {
		STEPPER_PORT |= _BV(STEPPER_DIRECTION);
	}
	else {
		STEPPER_PORT &= ~_BV(STEPPER_DIRECTION);
	}
	steps = abs(steps);

	//float usDelay = (1/speed) * 70;
	int usDelay = (1/speed) * 70;

	for(int i=0; i < steps; i++){

		STEPPER_PORT |= _BV(STEPPER_STEP);
		mydelay_us(usDelay);
		STEPPER_PORT &= ~_BV(STEPPER_STEP);
		mydelay_us(usDelay);
	}
}

/*************************************************************************
Function: sonar_rotate
Purpose:  used to track obstacles using the ultrasonic sensor, returns if I got obstacle or not
 **************************************************************************/
int sonar_rotate(int steps, float speed){

	uint8_t obstaculo = 0;
	//rotate a specific number of microsteps (8 microsteps per step) - (negative for reverse movement)
	//speed is any number from .01 -> 1 with 1 being fastest - Slower is stronger
	if (steps > 0) {
		STEPPER_PORT |= _BV(STEPPER_DIRECTION);
	}
	else {
		STEPPER_PORT &= ~_BV(STEPPER_DIRECTION);
	}
	steps = abs(steps);

	//float usDelay = (1/speed) * 70;
	int usDelay = (1/speed) * 70;

	for(int i=0; i < steps; i++){
		obstaculo = get_UltrasoundData();
		if (obstaculo == 1) {
			break;
		}
		STEPPER_PORT |= _BV(STEPPER_STEP);
		mydelay_us(usDelay);
		STEPPER_PORT &= ~_BV(STEPPER_STEP);
		mydelay_us(usDelay);
	}
	return obstaculo;
}


/*************************************************************************
Function: rotateDeg
Purpose:  the same as rotate, but with this you move the motor with the desired angle
 **************************************************************************/
void rotateDeg(float deg, float speed){
	//rotate a specific number of degrees (negitive for reverse movement)
	//speed is any number from .01 -> 1 with 1 being fastest - Slower is stronger
	if (deg > 0) {
		STEPPER_PORT |= _BV(STEPPER_DIRECTION);
	}
	else {
		STEPPER_PORT &= ~_BV(STEPPER_DIRECTION);
	}

	int steps = abs(deg)*(1/0.225);
	//float usDelay = (1/speed) * 70;
	int usDelay = (1/speed) * 70;

	for(int i=0; i < steps; i++){

		STEPPER_PORT |= _BV(STEPPER_STEP);
		mydelay_us(usDelay);

		STEPPER_PORT &= ~_BV(STEPPER_STEP);
		mydelay_us(usDelay);
	}
}

/*************************************************************************
Function: stepper_sweep
Purpose:  just a debug function to teste the stepper
 **************************************************************************/
void stepper_sweep(void) {
	rotate(25, .01);
	mydelay_ms(50);
	rotate(-25, .01); //reverse
	mydelay_ms(50);
}
