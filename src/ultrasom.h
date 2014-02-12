/* 
 * File:   ultrasom.h
 * Author: victor
 *
 * Created on May 12, 2013, 2:49 AM
 */
#ifndef ULTRASOM_H
#define	ULTRASOM_H

#include "LS_ATmega328.h"
#include "LS_ATmega328.h"
#include <util/delay.h>
#include "ultrasom_stepper.h"
#include "myDelay.h"
#include "sensorIR.h"

//defines if static or dynamic ultrassound sensor (stepper)
#define DYNAMIC_ULTRASOUND
#undef  DYNAMIC_ULTRASOUND

#define ULTRASOUND_SHIELD_DDR           DDRC
#define ULTRASOUND_SHIELD_PORT          PORTC
#define ULTRASOUND_SHIELD_PIN           PINC
#define ULTRASOUND1_ECHO                PC1             //output
#define ULTRASOUND1_TRIG                PC2             //output

#define US_ERROR                        0xFFFF
#define  US_NO_OBSTACLE                 0xFFFE

unsigned int getPulseWidth(void);
uint8_t get_UltrasoundData(void);
uint8_t sweep_sonar(void);

#endif	/* ULTRASOM_H */


