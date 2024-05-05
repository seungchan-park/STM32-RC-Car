/*
 * Controller.c
 *
 *  Created on: Apr 30, 2024
 *      Author: kccistc
 */

#include "Controller.h"

void Controller_init()
{
	MotorState_init();
}

void Controller_excuteTask()
{
	int mode = Mode_get();
	switch (mode) {
	case AUTO:
		Controller_AutoTask();
		break;
	case MANUAL:
		Controller_ManualTask();
		break;
	}
}

void Controller_ManualTask()
{
	int flag = BT_getFlag();

	if (!flag) return;

	uint8_t rxBuff[10];

	Controller_BTgetData(rxBuff);
	Controller_BTparsingData(rxBuff);
	MotorState_setFlag(MOTOR_SET);
	BT_setFlag(BT_RESET);
}

void Controller_AutoTask()
{
	int flag = UL_getFlag();

	if (!flag) return;

	uint8_t rxBuff[10];

	Controller_ULgetData(rxBuff);
	Controller_ULparsingData(rxBuff);
	MotorState_setFlag(MOTOR_SET);
	UL_setFlag(UL_RESET);
}

void Controller_BTgetData(uint8_t *buff)
{
	uint8_t rxData = 1;

	for (int i = 0; rxData; i++) { // NULL을 만날 때까지
		rxData = BT_deQue();
		buff[i] = rxData;
	}
}

void Controller_ULgetData(uint8_t *buff)
{
	uint8_t rxData = 1;

	for (int i = 0; rxData; i++) { // NULL을 만날 때까지
		rxData = UL_deQue();
		buff[i] = rxData;
	}
}

void Controller_BTparsingData(uint8_t *buff)
{
	if (buff[0] == 'g') { // go
		MotorState_enQue(GO);
	}
	else if (buff[0] == 'b') { // back
		MotorState_enQue(BACK);
	}
	else if (buff[0] == 'l') { // left
		MotorState_enQue(LEFT);
	}
	else if (buff[0] == 'r') { // right
		MotorState_enQue(RIGHT);
	}
	else if (buff[0] == 's') { // stop
		MotorState_enQue(STOP);
	}
	else if (buff[0] == 'p') { // p98'\0'
		MotorState_enQue(SPEED);
		int speed = atoi((char *)&buff[1]);
		MotorState_setSpeed(speed);
	}
}

void Controller_ULparsingData(uint8_t *buff)
{
	// buff[0] = front, buff[1] = left, buff[2] = right
	if (buff[0] < 30 && buff[1] < 50) {
		MotorState_enQue(RIGHT);
	}
	else if (buff[1] < 30) {
		MotorState_enQue(RIGHT);
	}
	else if (buff[0] < 30 && buff[2] < 50) {
		MotorState_enQue(LEFT);
	}
	else if (buff[2] < 30) {
		MotorState_enQue(LEFT);
	}
	else if (buff[0] < 30 && buff[1] < 30 && buff[2] < 30) {
		MotorState_enQue(STOP);
	}
	else {
		MotorState_enQue(GO);
	}
}
