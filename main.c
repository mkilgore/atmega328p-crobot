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

/* define CPU frequency in Mhz here if not defined in Makefile */
#ifndef F_CPU
#define F_CPU 16000000UL
#endif

/* 9600 baud */
#define UART_BAUD_RATE 9600
#define    STX          0x0002		//start TX: inicio do protocolo de envio
#define    ETX          0x0003		//end TX:	fim do protocolo de envio

/********************************************************************************
Global Variables
 ********************************************************************************/

static FILE mystdout = FDEV_SETUP_STREAM(usart_putchar_printf, NULL, _FDEV_SETUP_WRITE);
//volatile unsigned long milliseconds;

union coordenada_un {
		struct {
				uint8_t hi;
				uint8_t lo;
		} fields;
		uint16_t data;
};	    //16 bits access: x.data ou y.data

union coordenada_un x;
union coordenada_un y;

/*************************************************************************
	functions
 *************************************************************************/

//ISR (TIMER2_COMPA_vect) { /* testado: OK */
//	++milliseconds;
//}
//
//unsigned long millis(void) /* testado: OK */
//{
//	return milliseconds;
//}


/*************************************************************************
Function: setJoystick_Int
Purpose:  formats the joystick data grabbed of the application, by serial BT
 **************************************************************************/
void setJoystick_Int(uint8_t eixo_x_hi, uint8_t eixo_x_lo, uint8_t eixo_y_hi,uint8_t eixo_y_lo, int8_t* eixoX, int8_t* eixoY)    {
	// Check that these are not pointing to NULL
	assert(eixoX);
	assert(eixoY);
	int joyX = (eixo_x_hi<<7) + eixo_x_lo;
	int joyY = (eixo_y_hi<<7) + eixo_y_lo;
	joyX = joyX - 200;               // Transmission offset = 110
	joyY = joyY - 200;               // to avoid negative numbers

	// Your code here ...
	*eixoX = joyX;
	*eixoY = joyY;
}

/*************************************************************************
Function: trata_Protocolo
Purpose:  grab the raw data from Android application in order to pass it formatted
		  to the other functions
 **************************************************************************/
void trata_Protocolo(int8_t* eixoX, int8_t* eixoY) {

	uint8_t i = 0;

	while (uart_available() == 0);
	while ((uart_getc() & 0x00ff) != STX) {
		while (uart_available() == 0);
	}
	while (uart_available() == 0);
	i++;
	x.fields.hi = uart_getc();
	while (uart_available() == 0);
	i++;
	x.fields.lo = uart_getc();
	while (uart_available() == 0);
	i++;
	y.fields.hi = uart_getc();
	while (uart_available() == 0);
	i++;
	y.fields.lo = uart_getc();
	while (uart_available() == 0);
	i++;
	uart_getc();  //just do discard ETX

	/* DEBUG: minicom to see the data comming from BT module */
	//		printf("x-hi:");
	//		printf("0x%04d", x.fields.hi);
	//		uart_putc(' ');
	//		printf("x-lo:");
	//		printf("0x%04d\t", x.fields.lo);
	//		printf("y-hi:");
	//		printf("0x%04d", y.fields.hi);
	//		uart_putc(' ');
	//		printf("y-lo:");
	//		printf("0x%04d\n", y.fields.lo);

	if (i == 5) {
		setJoystick_Int(x.fields.hi, x.fields.lo, y.fields.hi, y.fields.lo, eixoX, eixoY);  //i=5 says that I get the whole desired protocol
	}
	else {
		uart_putc('*');
	}

}

/*************************************************************************
Function: trata_Obstaculos
Purpose:  looks for obstacles around the robot with the sonar and forward with the IR
 **************************************************************************/
void trata_Obstaculos(uint8_t* obstaculo_ir, uint8_t* obstaculo_sonar) {

	//cli();
	*obstaculo_ir = verificaObstaculo();

#ifdef DYNAMIC_ULTRASOUND
	*obstaculo_sonar = sweep_sonar();
	set_stepper_zero();

#else
	*obstaculo_sonar = get_UltrasoundData();
#endif
	//sei();
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
Function: timer2_init
Purpose:  initialize timer2 to be used in millis function
 **************************************************************************/
//void timer2_init(void) {
//
//	OCR2A = 124;  //setup for 1ms no modo CTC OCR2A = [(F_CPU/2*PRESCALER)*desired_delay]-1
//	TCCR2A |= (1 << WGM21);
//	// Set to CTC Mode
//	TIMSK2 |= (1 << OCIE2A);
//	//Set interrupt on compare match
//	TCCR2B |= (1 << CS22);
//	// set prescaler to 64 and starts PWM
//	TIMER2_OC2A_OFF();
//	TIMER2_OC2B_OFF();
//}

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
	//TIMER0_FAST_PWM_MAX_MODE();
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
	ADC_CLOCK_PRESCALER_16();
	/* the original ADC frequency of this project was 125KHz (Prescaler = 128), thus, I changed it to sample faster, in 1MHz
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
	int8_t eixoX = 0;
	int8_t eixoY = 0;
	uint8_t obstaculo_ir = 0;
	uint8_t obstaculo_sonar = 0;

	/* initialize hardware */
	uartInit();
	buzzer_init();
	timer0_init();
	ADC_init();
	buzzer_init();
	//configura o timer2 para a função milis
	//timer2_init();

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
		if (uart_available() > 0) {
			obstaculo_ir = 0;
			obstaculo_sonar = 0;
			trata_Protocolo(&eixoX, &eixoY);
			if (eixoY >= 50) {
				trata_Obstaculos(&obstaculo_ir, &obstaculo_sonar);
			}
			if ((obstaculo_ir == 0 && obstaculo_sonar == 0) || (obstaculo_ir == 0 && obstaculo_sonar == 2)) {
				move_motores(eixoX, eixoY);
			} else {
				motores_stop();
				obstacleAlarm();
			}
		}
	}

	return 0;
}
