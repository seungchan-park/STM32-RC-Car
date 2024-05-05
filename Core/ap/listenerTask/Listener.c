/*
 * Listener.c
 *
 *  Created on: Apr 30, 2024
 *      Author: kccistc
 */

#include "Listener.h"

ultraSonic_t US_front, US_left, US_right;
button_t btn1, btn2;


void Listener_BT_ISR_Process(uint8_t rxData)
{
	if (Mode_get() == AUTO) return;

	if (rxData == ';') {
		BT_enQue('\0'); // null문자는 0과 '\n' 똑같다.
		BT_setFlag(BT_SET);
	}
	else {
		BT_enQue(rxData);
		BT_setFlag(BT_RESET);
	}
}

void Listener_UlraSonic_ISR_Process(uint16_t GPIO_Pin)
{
	if (Mode_get() == MANUAL) return;

	UltraSonic_ISR_Process(&US_front, GPIO_Pin);
	UltraSonic_ISR_Process(&US_left, GPIO_Pin);
	UltraSonic_ISR_Process(&US_right, GPIO_Pin);
}

void Listener_init(TIM_HandleTypeDef *tim1, TIM_HandleTypeDef *tim2, TIM_HandleTypeDef *tim4)
{
	BT_init();
	button_init(&btn1, GPIOC, GPIO_PIN_10);
	button_init(&btn2, GPIOC, GPIO_PIN_11);
	UltraSonic_init(&US_front, tim1, FRONT_GPIO_TRIG, FRONT_GPIO_TRIG_PIN, FRONT_GPIO_ECHO, FRONT_GPIO_ECHO_PIN);
	UltraSonic_init(&US_left, tim2, LEFT_GPIO_TRIG, LEFT_GPIO_TRIG_PIN, LEFT_GPIO_ECHO, LEFT_GPIO_ECHO_PIN);
	UltraSonic_init(&US_right, tim4, RIGHT_GPIO_TRIG, RIGHT_GPIO_TRIG_PIN, RIGHT_GPIO_ECHO, RIGHT_GPIO_ECHO_PIN);
}

void Listener_eventCheck()
{
	if (button_getState(&btn1) == ACT_RELEASED) {
		Mode_set(AUTO);
		BT_init();
	}
	if (button_getState(&btn2) == ACT_RELEASED) {
		Mode_set(MANUAL);
		BT_init();
	}
}

void Listener_excuteTask()
{
	if (Mode_get() == MANUAL) return;

	if (UL_getFlag() == UL_SET) return;

	int distance = UltraSonic_getDistance(&US_front);
	UL_enQue(distance); // 인수와 매개변수의 자료형이 int랑 uint8_t로 다르다.
	distance = UltraSonic_getDistance(&US_left);
	UL_enQue(distance);
	distance = UltraSonic_getDistance(&US_right);
	UL_enQue(distance);
	UL_enQue('\0'); // distance : front, left, right, '\0'

	UL_setFlag(UL_SET);
}
