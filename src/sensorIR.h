/*
 * sensorIR.h
 *
 *  Created on: Dec 30, 2013
 *      Author: victor
 */

#ifndef SENSORIR_H_
#define SENSORIR_H_
#define SENSOR_IR_DDR	DDRC
#define SENSOR_IR_PORT	PORTC
#define SENSOR_IR_PIN	PINC
#define SENSOR_IR_BIT	PC0
#define ALARM_DDR 	DDRB
#define ALARM_PORT	PORTB
#define ALARM_BIT	PB0
#define ALARM_PIN	8  //pino arduino: 8 pino atmega:PB0
#define ALARM_FREQUENCY	2000 //3kHz
#define ALARM_DURATION 50 //duracao em ms

int verificaObstaculo(void);
void obstacleAlarm(void);
int ir_converToDistance(int adc_value);
#endif /* SENSORIR_H_ */
