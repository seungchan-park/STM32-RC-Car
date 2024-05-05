/*
 * BT_dataQue.c
 *
 *  Created on: Apr 30, 2024
 *      Author: kccistc
 */

#include "BT_dataQue.h"

que_t BTQue, ULQue;
char Mode = MANUAL;

void BT_init()
{
	que_init(&BTQue);
	que_init(&ULQue);
}

void BT_enQue(uint8_t rxData)
{
	enQue(&BTQue, rxData);
}

uint8_t BT_deQue()
{
	return deQue(&BTQue);
}

void BT_setFlag(int flagState)
{
	setQueFlag(&BTQue, flagState);
}

int BT_getFlag()
{
	return getQueFlag(&BTQue);
}

void Mode_set(int mode)
{
	Mode = mode;
}

int Mode_get()
{
	return Mode;
}

void UL_enQue(uint8_t rxData)
{
	enQue(&ULQue, rxData);
}

uint8_t UL_deQue()
{
	return deQue(&ULQue);
}

void UL_setFlag(int flagState)
{
	setQueFlag(&ULQue, flagState);
}

int UL_getFlag()
{
	return getQueFlag(&ULQue);
}
