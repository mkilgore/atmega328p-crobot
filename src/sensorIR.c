/*
 * sensorIR.cpp
 *
 *  Created on: Dec 30, 2013
 *      Author: victor
 */

/* O sensor IR utiliza o canal 0 do ADC PORTC0 como porta de leitura */

#include "sensorIR.h"
#include <util/delay.h>
#include "template.h"
#include "LS_ATmega328.h"
#include "myDelay.h"
#include "stdio.h"
//globals

void obstacleAlarm(void) {
	int i = 0;
	for(i=0; i <= 30; i++)
	{
		ALARM_PORT |= _BV(ALARM_BIT);
		mydelay_us(75);
		ALARM_PORT &= ~_BV(ALARM_BIT);
		mydelay_us(75);
	}
}


int ir_converToDistance(int adc_value) {
	int distance;
	distance = (6787/(adc_value -3)) - 4;
	return distance;
}

int verificaObstaculo(void)
{
	int result_tmp = 0;
	int result_final = 0;
	int result_media = 0;
	int i = 0;
	const int n_samples = 5;
	//int distance_cm = 0;

	result_tmp = 0;

	for (i = n_samples; i > 0; i--) {
		ADC_START_CONVERSION();
		ADC_WAIT_CONVERSION_FINISH();
		result_tmp += ADC; //lê os 10 bits do ADC
		mydelay_us(25); //tempo de cada amostragem utilizando a frequencia do ADC em 1MHz
		//printf("leitura_ADC: %d\n", ADC );
	}

	result_media = (result_tmp / n_samples); //faço uma média para evitar falsos positivos
	//printf("leitura_ADC: %d\n", result_media );

	//distance_cm = ir_converToDistance(result_media);

	if (result_media > 500) {
		result_final = 1;
	}

	if (result_media <= 500) {
		result_final = 0;
	}
	result_media = 0;

	return result_final;
	//return distance_cm;
}


