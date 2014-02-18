/*
 *  ATmega328p robot contolled by bluetooth
 *
 * 	main.c
 *
 *  Created on: Jan 23, 2014
 *      Author: victor
 */

#include "src/template.h"
#include "src/uart.h"
#include "src/sensorIR.h"
#include "src/motores.h"
#include "src/myDelay.h"
#include "src/ultrasom.h"
#include "src/ultrasom_stepper.h"
#include "src/LS_ATmega328.h"
#include "src/LS_defines.h"
#include "stdlib.h"
#include <stdio.h>
#include <stdbool.h>
#include <assert.h>
#include "avr/interrupt.h"

#define MORE_PWM 190
#define LESS_PWM 165
#define WITH_CARE 1
#define NO_CARE	  0

//only detected as obstacle if distance is below 25cm, below 10cm the sensor starts misreading
#define IR_OBSTACLE_THRESHOLD 25
#define IR_OBSTACLE_UPPER_THRESHOLD 35
#define IR_MISREADING 10
/* define CPU frequency in Mhz here if not defined in Makefile */
#ifndef F_CPU
#define F_CPU 16000000UL
#endif

/* 9600 baud */
#define UART_BAUD_RATE 9600

/********************************************************************************
Global Variables
 ********************************************************************************/
volatile uint8_t autonomous;

static FILE mystdout = FDEV_SETUP_STREAM(usart_putchar_printf, NULL, _FDEV_SETUP_WRITE);
/*************************************************************************
	functions
 *************************************************************************/

void goForward(char flag)
{
	MOTOR_SHIELD_PORT |= _BV(MOTOR_INB) | _BV(MOTOR_INA);
	if (flag == 1) {
		TIMER0_COMPARE_A_CONFIGURE(LESS_PWM);
		TIMER0_COMPARE_B_CONFIGURE(LESS_PWM);
	}
	else  {
		TIMER0_COMPARE_A_CONFIGURE(MORE_PWM);
		TIMER0_COMPARE_B_CONFIGURE(MORE_PWM);
	}
	_delay_ms(100);
}

void goBack(char flag)
{
	MOTOR_SHIELD_PORT &= ~(_BV(MOTOR_INB) | _BV(MOTOR_INA) );
	if (flag == 1) {
		TIMER0_COMPARE_A_CONFIGURE(LESS_PWM);
		TIMER0_COMPARE_B_CONFIGURE(LESS_PWM);
	}
	else {
		TIMER0_COMPARE_A_CONFIGURE(MORE_PWM);
		TIMER0_COMPARE_B_CONFIGURE(MORE_PWM);
	}
	_delay_ms(100);
}

void goRight(char flag)
{
	MOTOR_SHIELD_PORT |= _BV(MOTOR_INB) | _BV(MOTOR_INA);
	TIMER0_COMPARE_A_CONFIGURE(0);
	if (flag == 1)  TIMER0_COMPARE_B_CONFIGURE(LESS_PWM); else TIMER0_COMPARE_B_CONFIGURE(MORE_PWM);
	_delay_ms(100);
}

void goLeft(char flag)
{
	MOTOR_SHIELD_PORT |= _BV(MOTOR_INB) | _BV(MOTOR_INA);
	if (flag == 1)  TIMER0_COMPARE_A_CONFIGURE(LESS_PWM); else TIMER0_COMPARE_A_CONFIGURE(MORE_PWM);
	TIMER0_COMPARE_B_CONFIGURE(0);
	_delay_ms(100);
}

void stopMove(void)
{
	TIMER0_COMPARE_A_CONFIGURE(0);
	TIMER0_COMPARE_B_CONFIGURE(0);
}

void turn(void)
{
	MOTOR_SHIELD_PORT |= _BV(MOTOR_INB);
	MOTOR_SHIELD_PORT &= ~_BV(MOTOR_INA);
	TIMER0_COMPARE_A_CONFIGURE(LESS_PWM);
	TIMER0_COMPARE_B_CONFIGURE(LESS_PWM);
	_delay_ms(100);
}

/*************************************************************************
    						HDW INITS
 *************************************************************************/

/*************************************************************************
Function: uartInit
Purpose:  initialize uart in 9600 8-N-1 standard and sets serial to stdout
 **************************************************************************/
void uartInit(void) {  /* testado: OK */
	/*
	 *  Initialize UART library, pass baudrate and AVR cpu clock
	 *  with the macro
	 *  UART_BAUD_SELECT() (normal speed mode )
	 *  or
	 *  UART_BAUD_SELECT_DOUBLE_SPEED() ( double speed mode)
	 */
	uart_init( UART_BAUD_SELECT(UART_BAUD_RATE,F_CPU) );

	// setup our stdio stream
	stdout = &mystdout;
}

/*************************************************************************
Function: switch_init
Purpose:  initialize switch key to indicate the stepper's zero position
 **************************************************************************/
void switch_init(void) {
	FIM_DE_CURSO_DDR &= ~_BV(FIM_DE_CURSO);
}

/*************************************************************************
Function: stepper_init
Purpose:  initialize the stepper pinout to be use with the ultrasonic sensor
 **************************************************************************/
void stepper_init(void) {
	STEPPER_DDR |= _BV(STEPPER_DIRECTION) | _BV(STEPPER_STEP);
}

/*************************************************************************
Function: timer0_init
Purpose:  used to generate PWM to the motor wheels
 **************************************************************************/
void timer0_init(void) {
	MOTOR_SHIELD_DDR |= _BV(MOTOR_PWMA) | _BV(MOTOR_PWMB) | _BV(MOTOR_INA) | _BV(MOTOR_INB);
	TIMER0_PHASE_CORRECT_MAX_PWM_MODE();
	TIMER0_OC0A_CLEAR_ON_COMPARE();
	TIMER0_OC0B_CLEAR_ON_COMPARE();
	TIMER0_CLOCK_PRESCALER_OFF();
	TIMER0_COMPARE_A_CONFIGURE(0);
	TIMER0_COMPARE_B_CONFIGURE(0);
}

/*************************************************************************
Function: ADC_init
Purpose:  set up the ADC to be used in IR sensor
 **************************************************************************/
void ADC_init(void) {
	SENSOR_IR_DDR &= ~_BV(SENSOR_IR_BIT);
	//ADC in 10bits : used in IR sensor routine : without shift left or right
	ADC_REFERENCE_AREF(); 			//ADC reference in 5V
	ADC_CLOCK_PRESCALER_128();
	/* the original ADC frequency of this project was 125KHz (Prescaler = 128), thus, I changed it to sample faster, in 1MHz (Prescaler16)
	 * I have some loss in precision, working in 10 bits with a frequency bigger than 200KHz, but in this case this do not matters
	 * */
	ADC_ENABLE();
	ADC_SELECT_CHANNEL_0();
	ADC_DIGITAL_INPUT_0_DISABLE();
	ADC_START_CONVERSION();				 //I discard the first sample, which takes 25 clock cycles
	ADC_WAIT_CONVERSION_FINISH();
}

/*************************************************************************
Function: buzzer_init
Purpose:  configures the pin used by buzzer, to alarm obstacles
 **************************************************************************/
void buzzer_init(void){
	ALARM_DDR |= _BV(ALARM_BIT);
}

/*-------------------------------*/
/* Protocol received by UART: Bluetooth module <STX MSB-X LSB-X MSB-Y LSB-Y ETX> */

int main(void)
{
	uint8_t obstaculo_ir = 0;
	char received = 0;
	char obstacle_flag = 0;

	/* initialize hardware */
	uartInit();
	buzzer_init();
	timer0_init();
	ADC_init();
	buzzer_init();

	/* DYNAMIC_ULTRASOUND defined in ultrasom.h
	 * commenting the #undef  DYNAMIC_ULTRASOUND you enable the ultrasonic sensor sweep with the stepper motor together with it.
	 */
#ifdef DYNAMIC_ULTRASOUND
	switch_init();
	stepper_init();
	set_stepper_zero();
#endif

	sei();

	for(;;)
	{
		do {
			stopMove();
			obstaculo_ir = 0;
			obstacle_flag = 0;
			while (uart_available() == 0);
			received = uart_getc();
			obstaculo_ir = verificaObstaculoIR();
			if (obstaculo_ir > IR_OBSTACLE_THRESHOLD) 										obstacle_flag = 0;
			else if (obstaculo_ir > IR_MISREADING && obstaculo_ir < IR_OBSTACLE_THRESHOLD) 	obstacle_flag = 1;
			else if (obstaculo_ir <= IR_MISREADING ) 										obstacle_flag = 2;

			if (obstacle_flag == 0 || obstacle_flag == 1) {
				switch(received) {
					case 'u':
						goForward(obstacle_flag);
						break;
					case 'd':
						goBack(obstacle_flag);
						break;
					case 'l':
						goLeft(obstacle_flag);
						break;
					case 'r':
						goRight(obstacle_flag);
						break;
					default:
						stopMove();
						break;
				}
			} else {
				if (received == 'd') {
					goBack(WITH_CARE);
				} else {
					stopMove();
					obstacleAlarm();
				}
			}
		} while (autonomous == 0);  //defined inside UART0_RECEIVE_INTERRUPT interrupt on uart.c
		do {
			obstaculo_ir = 0;
			obstaculo_ir = verificaObstaculoIR();
			if (obstaculo_ir >= IR_OBSTACLE_THRESHOLD && obstaculo_ir < IR_OBSTACLE_UPPER_THRESHOLD) goForward(WITH_CARE);
			else if (obstaculo_ir >= IR_OBSTACLE_UPPER_THRESHOLD) goForward(NO_CARE);
			else {
				goBack(0);
				turn();
			}
		} while (autonomous == 1);

	}

	return 0;
}
