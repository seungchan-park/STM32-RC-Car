/*
 * Ultrasonic.c
 *
 *  Created on: Apr 24, 2024
 *      Author: kccistc
 */
#include "Ultrasonic.h"
#include "../common/delayus/delayus.h"

void UltraSonic_init(
		ultraSonic_t *ultraSonic,
		TIM_HandleTypeDef *hTim,
		GPIO_TypeDef* GPIO_Trig,
		uint16_t GPIO_TrigPin,
		GPIO_TypeDef* GPIO_Echo,
		uint16_t GPIO_EchoPin)
{
	ultraSonic->hTim = hTim;
	ultraSonic->GPIO_Trig = GPIO_Trig;
	ultraSonic->GPIO_TrigPin = GPIO_TrigPin;
	ultraSonic->GPIO_Echo = GPIO_Echo;
	ultraSonic->GPIO_EchoPin = GPIO_EchoPin;
	ultraSonic->timCounter = 0;
	ultraSonic->echoFlag = 0;
}

int UltraSonic_getEchopinState(ultraSonic_t *ultraSonic)
{
	return HAL_GPIO_ReadPin(ultraSonic->GPIO_Echo, ultraSonic->GPIO_EchoPin);
}

void UltraSonic_clearTimer(ultraSonic_t *ultraSonic)
{
	__HAL_TIM_SET_COUNTER(ultraSonic->hTim, 0); // counter 0으로 초기화
}

void UltraSonic_startTimer(ultraSonic_t *ultraSonic)
{
	HAL_TIM_Base_Start(ultraSonic->hTim); // count 시작
}

void UltraSonic_stopTimer(ultraSonic_t *ultraSonic)
{
	HAL_TIM_Base_Stop(ultraSonic->hTim); // count 끝
}

uint16_t UltraSonic_getTimerCounter(ultraSonic_t *ultraSonic)
{
	return __HAL_TIM_GET_COUNTER(ultraSonic->hTim); // count값을 가져와 저장
}

void UltraSonic_ISR_Process(ultraSonic_t *ultraSonic, uint16_t GPIO_Pin)
{
	if ((GPIO_Pin == ultraSonic->GPIO_EchoPin)) {
		// echo pin high 유지시간 측정
		if (UltraSonic_getEchopinState(ultraSonic)) { // rising edge
			UltraSonic_clearTimer(ultraSonic);
			UltraSonic_startTimer(ultraSonic);
			UltraSonic_clearEchoFlag(ultraSonic);
		}
		else { // falling edge
			UltraSonic_stopTimer(ultraSonic);
			ultraSonic->timCounter = UltraSonic_getTimerCounter(ultraSonic);
			UltraSonic_setEchoFlag(ultraSonic);
		}
	}
}

void UltraSonic_startTrig(ultraSonic_t *ultraSonic)
{
	HAL_GPIO_WritePin(ultraSonic->GPIO_Trig, ultraSonic->GPIO_TrigPin, SET);
	DelayUS(15);
	HAL_GPIO_WritePin(ultraSonic->GPIO_Trig, ultraSonic->GPIO_TrigPin, RESET);
}

void UltraSonic_clearEchoFlag(ultraSonic_t *ultraSonic)
{
	ultraSonic->echoFlag = 0;
}

void UltraSonic_setEchoFlag(ultraSonic_t *ultraSonic)
{
	ultraSonic->echoFlag = 1;
}

int UltraSonic_isCmpltRecvEcho(ultraSonic_t *ultraSonic)
{
	return ultraSonic->echoFlag;
}

int UltraSonic_getDistance(ultraSonic_t *ultraSonic)
{
	int timeout = 0;

	UltraSonic_startTrig(ultraSonic);
	while(!UltraSonic_isCmpltRecvEcho(ultraSonic)){
		timeout++;
		if(timeout > 10) return 100;
		HAL_Delay(1);
	}
	UltraSonic_clearEchoFlag(ultraSonic);
	return ultraSonic->timCounter * 0.017; // cm distance
}
