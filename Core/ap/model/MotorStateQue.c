/*
 * MotorStateQue.c
 *
 *  Created on: Apr 30, 2024
 *      Author: kccistc
 */

#include "MotorStateQue.h"


//que_t motorStateQue;

typedef struct{
	que_t state;
	int speed;
}motorState_t;

motorState_t motorStateQue;

void MotorState_init()
{
	que_init(&motorStateQue.state);
	motorStateQue.speed = 0;
}

void MotorState_setSpeed(int speed)
{
	motorStateQue.speed = speed;
}

int MotorState_getSpeed()
{
	return motorStateQue.speed;
}

void MotorState_enQue(uint8_t rxData)
{
	enQue(&motorStateQue.state, rxData);
}

uint8_t MotorState_deQue()
{
	return deQue(&motorStateQue.state);
}

void MotorState_setFlag(int flagState)
{
	setQueFlag(&motorStateQue.state, flagState);
}

int MotorState_getFlag()
{
	return getQueFlag(&motorStateQue.state);
}
