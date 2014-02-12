/*
 * motores.cpp
 * Trata o acionamento dos motores do robô
 *  Created on: Dec 30, 2013
 *      Author: victor
 */
/* Autor do algoritmo de controle das rodas: Jonatha Fogaça
 * Created on: 02/01/2014
 * */

#include "motores.h"

/*************************************************************************
Function: calcula_coordenadas
Purpose:  vetorial calculation of coordinates
 **************************************************************************/
void calcula_coordenadas(int joyX, int joyY, float* motor_esquerda, float* motor_direita) {

	float resultante = 0;
	float mystic_number = 78;
	float tmp = 0, tempX = 0, tempY = 0;

	tempY = pow(joyY,2);
	tempX = pow(joyX,2);
	tmp = tempX + tempY;
	resultante = sqrt(tmp);
	*motor_direita = sqrt(tempX + tempY - (200*joyX) + 10000);
	*motor_esquerda = sqrt(tempX + tempY + (200*joyX) + 10000);
	*motor_direita = (int)(*motor_direita*resultante)/mystic_number;
	*motor_esquerda = (int)(*motor_esquerda*resultante)/mystic_number;
	if (*motor_direita >= 255) *motor_direita = 255;
	if (*motor_esquerda >= 255) *motor_esquerda = 255;

}

/*************************************************************************
Function: move_motores
Purpose:  moves the motors according to the function calcula_coordenadas()
 **************************************************************************/
void move_motores(int joyX, int joyY)
{
	float motor_direita = 0;
	float motor_esquerda = 0;

	calcula_coordenadas(joyX, joyY, &motor_esquerda, &motor_direita);
	TIMER0_COMPARE_A_CONFIGURE((int) motor_direita);
	TIMER0_COMPARE_B_CONFIGURE((int) motor_esquerda);
	//two upper quadrants
	if (joyY > 0) {
		MOTOR_SHIELD_PORT |= _BV(MOTOR_INB) | _BV(MOTOR_INA);
	} else {
		MOTOR_SHIELD_PORT &= ~(_BV(MOTOR_INB) | _BV(MOTOR_INA) );
	}
}

/*************************************************************************
Function: motores_stop
Purpose:  stops the two motors
 **************************************************************************/
void motores_stop(void) {
	TIMER0_COMPARE_A_CONFIGURE(0);
	TIMER0_COMPARE_B_CONFIGURE(0);
}
