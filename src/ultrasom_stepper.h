/*
 * ultrasom_stepper.h
 *
 *  Created on: Jan 4, 2014
 *      Author: victor
 */


#ifndef ULTRASOM_STEPPER_H_
#define ULTRASOM_STEPPER_H_

#include "sensorIR.h"
#include "LS_ATmega328.h"
#include "LS_defines.h"
#include "stdlib.h"
#include "myDelay.h"
#include "ultrasom.h"

#define STEPPER_DDR                     DDRB
#define STEPPER_PIN                     PINB
#define STEPPER_PORT                    PORTB
#define STEPPER_DIRECTION               PB4
#define STEPPER_STEP                    PB5
#define FIM_DE_CURSO_DDR                DDRB
#define FIM_DE_CURSO_PIN                PINB
#define FIM_DE_CURSO_PORT               PORTB
#define FIM_DE_CURSO		            PB3

void rotate(int steps, int speed);
//int sonar_rotate(int steps, float speed);
int sonar_rotate(int steps, int speed, int* distance_read);
void rotateDeg(float deg, float speed);
int read_switches(void);
void set_stepper_zero(void); //seta motor para posição inicial
void stepper_sweep(void); //faz a varredura do sonar
#endif /* ULTRASOM_STEPPER_H_ */
