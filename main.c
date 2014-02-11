/*
 * main.c
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

const long timeStamp = 1000;             // interval to transmit Buttons status (milliseconds)
uint8_t dataByte = 0;                    // second Byte sent to Android device
uint8_t buttonStatus = 0;                // first Byte sent to Android device
volatile unsigned long milliseconds;

union coordenada_un {
		struct {
				uint8_t hi;
				uint8_t lo;
		} fields;
		uint16_t data;
};	    // para acessar como 16 bits: x.data ou y.data

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
Function: sendData
Purpose:  sends data of buttons to refresh the status on the Android apk.
 **************************************************************************/
//void sendData(uint8_t button, uint8_t data)    {
//	/*
//	 *  Transmit string to UART
//	 *  The string is buffered by the uart library in a circular buffer
//	 *  and one character at a time is transmitted to the UART using interrupts.
//	 *  uart_puts() blocks if it can not write the whole string to the circular
//	 *  buffer
//	 */
//	uart_putc(STX);
//	uart_putc((button<<2)+4);
//	uart_putc(data+4);
//	uart_putc(ETX); // +4: avoid 0x2 & 0x3
//}

/*************************************************************************
Function: setLED
Purpose:  reads the button status to be refreshed on Android application using
		  sendData.
 **************************************************************************/
//void setLED(int LEDstatus)  {
//	switch (LEDstatus) {
//		case '1':
//			buttonStatus |= 0b0001;    // Button_1: ON
//			// your code...
//
//			//flagAutonomo = 1;
//			break;
//		case '2':
//			buttonStatus &= 0b1110;    // Button_1: OFF
//			// your code...
//			//		flagAutonomo = 0;
//			break;
//		case '3':
//			buttonStatus |= 0b0010;    // Button_2: ON
//			// your code...
//			break;
//		case '4':
//			buttonStatus &= 0b1101;    // Button_2: OFF
//			// your code...
//			break;
//		case '5':                    // configured as momentary button
//			buttonStatus |= 0b0100;     // Button_3: ON
//			// your code...
//			break;
//		case '6':
//			buttonStatus &= 0b1011;     // Button_3: OFF
//			// your code...
//			break;
//		case '7':                    // configured as momentary button
//			//      buttonStatus |= B1000;     // Button_4: ON
//			// your code...
//			break;
//		case '8':
//			buttonStatus &= 0b0111;    // Button_4: OFF
//			// your code...
//			break;
//	}
//	sendData(buttonStatus, dataByte);
//}

/*************************************************************************
Function: setJoystick_Int
Purpose:  formats the joystick data grabbed of the application, by serial BT
 **************************************************************************/
void setJoystick_Int(uint8_t eixo_x_hi, uint8_t eixo_x_lo, uint8_t eixo_y_hi,uint8_t eixo_y_lo, int* eixoX, int* eixoY)    {
	// Demo
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
Function: getdataByte
Purpose:  refresh the receivement of data packets in Android application
 **************************************************************************/
//uint8_t GetdataByte()  {
//	// Demo
//	static uint8_t i=0;
//	i +=5;
//	if(i >100)    i = 0;
//	return i;
//
//	// Your code ...
//}

/*************************************************************************
Function: trata_Protocolo
Purpose:  grab the raw data from Android application in order to pass it formatted
		  to the other functions
 **************************************************************************/
void trata_Protocolo(int* eixoX, int* eixoY) {

	uint8_t i = 0;

	/* uart_getc();
	 * Get received character from ringbuffer
	 * uart_getc() returns in the lower byte the received character and
	 * in the higher byte (bitmask) the last receive error
	 * UART_NO_DATA is returned when no data is available.
	 */

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
	uart_getc();  //somente para consumir ETX

	/* DEBUG: abrir o minicom para monitorar, desta forma os dados não voltarão para o apk */
	//	printf("x-hi:");
	//	printf("0x%04d", x.fields.hi);
	//	uart_putc(' ');
	//	printf("x-lo:");
	//	printf("0x%04d\t", x.fields.lo);
	//	printf("y-hi:");
	//	printf("0x%04d", y.fields.hi);
	//	uart_putc(' ');
	//	printf("y-lo:");
	//	printf("0x%04d\n", y.fields.lo);

	//	switch(i) {
	//
	//		case 2:
	//			//setLED(cmd[1]);
	//			setLED(x.fields.hi);
	//			i = 0;
	//			break;
	//
	//		case 5:
	//			setJoystick_Int(x.fields.hi, x.fields.lo, y.fields.hi, y.fields.lo, eixoX, eixoY);  // 6 Bytes: recebeu todos os 6 bytes  passo o endereço do eixoX e eixoY para atualizar seus valores dentro da função setJoystick_Int
	//			i = 0;
	//			break;
	//
	//		default:
	//			uart_putc('*');
	//			i = 0;
	//			break;
	//	}
	if (i == 5) {
		setJoystick_Int(x.fields.hi, x.fields.lo, y.fields.hi, y.fields.lo, eixoX, eixoY);  // 6 Bytes: recebeu todos os 6 bytes  passo o endereço do eixoX e eixoY para atualizar seus valores dentro da função setJoystick_Int
		i = 0;
	}
	else {
		uart_putc('*');
		i = 0;
	}

}

/*************************************************************************
Function: trata_Obstaculos
Purpose:  looks for obstacles around the robot with the sonar and forward with the IR
 **************************************************************************/
void trata_Obstaculos(unsigned int* obstaculo_ir, unsigned int* obstaculo_sonar) {

	*obstaculo_ir = verificaObstaculo();

#ifdef DYNAMIC_ULTRASOUND //definido em ultrasom.h
	*obstaculo_sonar = sweep_sonar();		   //sweep dinamico
	set_stepper_zero();

#else
	*obstaculo_sonar = get_UltrasoundData();  //sweep estatico
#endif
}

/*************************************************************************
      inits
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
void switch_init(void) {  /* testado: OK */
	FIM_DE_CURSO_DDR &= ~_BV(FIM_DE_CURSO);
}


/*************************************************************************
Function: timer2_init
Purpose:  initialize timer2 to be used in millis function
 **************************************************************************/
//void timer2_init(void) { /* testado: OK */
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
void timer0_init(void) {  /* testado: OK */

	MOTOR_SHIELD_DDR |= _BV(MOTOR_PWMA) | _BV(MOTOR_PWMB) | _BV(MOTOR_INA) | _BV(MOTOR_INB);
	TIMER0_PHASE_CORRECT_MAX_PWM_MODE();
	//TIMER0_FAST_PWM_MAX_MODE();
	TIMER0_OC0A_CLEAR_ON_COMPARE();
	TIMER0_OC0B_CLEAR_ON_COMPARE();
	TIMER0_CLOCK_PRESCALER_OFF();
}

/*************************************************************************
Function: ADC_init
Purpose:  set up the ADC to be used in IR sensor
 **************************************************************************/
void ADC_init(void) {  /* testado: OK */
	SENSOR_IR_DDR &= ~_BV(SENSOR_IR_BIT);

	//ADC em 10bits : usado no sensoriamento IR : sem shift left ou right
	ADC_REFERENCE_AREF(); //referencia do ADC em 5V
	ADC_CLOCK_PRESCALER_16();
	/* a frequência original do ADC neste projeto era 125KHz (PS = 128), estou tentando amostrar mais rápido em 1MHz
	 existe perda de precisão quando trabalhamos em 10bits com uma frequencia de amostragem maior que 200KHz, mas para este caso
	acaba não sendo tão crucial assim.*/
	ADC_ENABLE();
	ADC_SELECT_CHANNEL_0();
	ADC_DIGITAL_INPUT_0_DISABLE();
	ADC_START_CONVERSION(); //descarto a primeira amostra, que leva 25 ciclos de clock
	ADC_WAIT_CONVERSION_FINISH();
}

/*************************************************************************
Function: buzzer_init
Purpose:  configures the pin used by buzzer
 **************************************************************************/
void buzzer_init(void){
	//configure buzzer de alarme para obstaculo
	ALARM_DDR |= _BV(ALARM_BIT);
}

/*-------------------------------*/
//protocolo de 6 bytes <STX MSB-X LSB-X MSB-Y LSB-Y ETX>

int main(void)
{

	int eixoX = 0, eixoY = 0;
	unsigned int obstaculo_ir = 0;
	unsigned int obstaculo_sonar = 0;
	//static long previousMillis = 0;              // will store last Buttons status was updated
	//unsigned long currentMillis = 0;
	//int temp = 0;


	uartInit();

	buzzer_init();

	/*
	 * now enable interrupt, since UART library is interrupt controlled
	 */
	//configura o PWM do motor
	timer0_init();

	//configura o ADC
	ADC_init();

	//configura o alarme de obstaculos
	buzzer_init();

	//configura o timer2 para a função milis
	//timer2_init();

#ifdef DYNAMIC_ULTRASOUND  //definido em ultrasom.h
	switch_init();
	stepper_init();
	set_stepper_zero();
#endif
	sei();
	for(;;)
	{
		if (uart_available() > 0) {

			trata_Protocolo(&eixoX, &eixoY);
			if (eixoY >= 50) {
				trata_Obstaculos(&obstaculo_ir, &obstaculo_sonar);
			}
			if ((obstaculo_ir == 0 && obstaculo_sonar == 0) || (obstaculo_ir == 0 && obstaculo_sonar == 2)) {
				//sem obstaculo
				move_motores(eixoX, eixoY);
				obstaculo_ir = 0;
				obstaculo_sonar = 0;
			} else {                                             //else if (obstaculo_ir == 1 || obstaculo_sonar == 1) {
				motores_stop();
				obstacleAlarm();
				obstaculo_ir = 0;
				obstaculo_sonar = 0;
			}
		}
		//		temp = sweep_sonar();
		//		set_stepper_zero();
		//		printf("obstaculo_sonar: %d\n", temp);
	}

	return 0;
}
