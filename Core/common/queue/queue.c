/*
 * queue.c
 *
 *  Created on: Apr 26, 2024
 *      Author: kccistc
 */

#include "queue.h"


void que_init(que_t *que)
{
	que->head = 0;
	que->tail = 0;
	que->queCounter = 0;
	que->cmpltFlag = 0;
}

void setQueFlag(que_t *que, int flagState)
{
	que->cmpltFlag = flagState;
}

int getQueFlag(que_t *que)
{
	return que->cmpltFlag;
}

int queFull(que_t *que)
{
	//if (head == ((tail+1)%BUF_SIZE))
	if (que->queCounter == BUF_SIZE)
		return 1; // Full
	else
		return 0;
}

int queEmpty(que_t *que)
{
	//if (head == tail)
	if (que->queCounter == 0)
		return 1; // Empty
	else
		return 0;
}

void enQue(que_t *que, uint8_t data) // push
{
	if(queFull(que)) return;

	que->queBuff[que->tail] = data;
	que->tail = (que->tail+1)%BUF_SIZE;
	que->queCounter++;
}

uint8_t deQue(que_t *que) // pop
{
	if(queEmpty(que)) return -1;

	uint8_t temp = que->queBuff[que->head];
	que->head = (que->head+1)%BUF_SIZE;
	que->queCounter--;

	return temp;
}
