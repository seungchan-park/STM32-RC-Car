/*
 * Ultrasonic.h
 *
 *  Created on: Apr 24, 2024
 *      Author: kccistc
 */

#ifndef DRIVER_ULTRASONIC_ULTRASONIC_H_
#define DRIVER_ULTRASONIC_ULTRASONIC_H_
#include "stm32f4xx_hal.h"

// trig port, trig pin, echo port, echo pin, timer, flag
typedef struct { // 다르게 쓰인 정보를 구조체로 표현
	TIM_HandleTypeDef *hTim;
	GPIO_TypeDef* GPIO_Trig;
	uint16_t GPIO_TrigPin;
	GPIO_TypeDef* GPIO_Echo;
	uint16_t GPIO_EchoPin;
	uint16_t timCounter;
	int echoFlag;
} ultraSonic_t;

void UltraSonic_init(
		ultraSonic_t *ultraSonic,
		TIM_HandleTypeDef *hTim,
		GPIO_TypeDef* GPIO_Trig,
		uint16_t GPIO_TrigPin,
		GPIO_TypeDef* GPIO_Echo,
		uint16_t GPIO_EchoPin);
int UltraSonic_getEchopinState(ultraSonic_t *ultraSonic);
void UltraSonic_clearTimer(ultraSonic_t *ultraSonic);
void UltraSonic_startTimer(ultraSonic_t *ultraSonic);
void UltraSonic_stopTimer(ultraSonic_t *ultraSonic);
uint16_t UltraSonic_getTimerCounter(ultraSonic_t *ultraSonic);
void UltraSonic_ISR_Process(ultraSonic_t *ultraSonic, uint16_t GPIO_Pin);
void UltraSonic_startTrig(ultraSonic_t *ultraSonic);
void UltraSonic_clearEchoFlag(ultraSonic_t *ultraSonic);
void UltraSonic_setEchoFlag(ultraSonic_t *ultraSonic);
int UltraSonic_isCmpltRecvEcho(ultraSonic_t *ultraSonic);
int UltraSonic_getDistance(ultraSonic_t *ultraSonic);

#endif /* DRIVER_ULTRASONIC_ULTRASONIC_H_ */
