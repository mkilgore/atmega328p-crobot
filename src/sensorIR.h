/*
 * sensorIR.h
 *
 *  Created on: Dec 30, 2013
 *      Author: victor
 */


#ifndef SENSORIR_H_
#define SENSORIR_H_

#include <util/delay.h>
#include "template.h"
#include "LS_ATmega328.h"
#include "myDelay.h"
#include "stdio.h"

#define SENSOR_IR_DDR	DDRC
#define SENSOR_IR_PORT	PORTC
#define SENSOR_IR_PIN	PINC
#define SENSOR_IR_BIT	PC0
#define ALARM_DDR 	DDRB
#define ALARM_PORT	PORTB
#define ALARM_BIT	PB0

uint8_t verificaObstaculo(void);
void obstacleAlarm(void);
int ir_convertToDistance(int adc_value);
#endif /* SENSORIR_H_ */
