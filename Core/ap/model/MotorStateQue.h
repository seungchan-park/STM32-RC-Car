/*
 * MotorStateQue.h
 *
 *  Created on: Apr 30, 2024
 *      Author: kccistc
 */

#ifndef AP_MODEL_MOTORSTATEQUE_H_
#define AP_MODEL_MOTORSTATEQUE_H_

#include "../../common/queue/queue.h"
#include <stdint.h>

#define MOTOR_SET		1
#define MOTOR_RESET		0

typedef enum {STOP, GO, LEFT, RIGHT, BACK, SPEED}motorState_e;

void MotorState_init();
void MotorState_setSpeed(int speed);
int MotorState_getSpeed();
void MotorState_enQue(uint8_t rxData);
uint8_t MotorState_deQue();
void MotorState_setFlag(int flagState);
int MotorState_getFlag();

#endif /* AP_MODEL_MOTORSTATEQUE_H_ */
