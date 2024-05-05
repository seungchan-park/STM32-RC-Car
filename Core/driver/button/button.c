/*
 * button.c
 *
 *  Created on: Apr 23, 2024
 *      Author: kccistc
 */
#include "button.h"

void button_init(button_t *button, GPIO_TypeDef* GPIOx, uint16_t GPIO_Pin)
{
	button->GPIOx = GPIOx;
	button->GPIO_Pin = GPIO_Pin;
	button->prevState = RELEASED;
}

button_state_t button_getState(button_t *button)
{
	int curState = HAL_GPIO_ReadPin(button->GPIOx, button->GPIO_Pin);
	if ((button->prevState == RELEASED) && (curState == PUSHED)) {
		//HAL_Delay(50);
		button->prevState = PUSHED;
		return ACT_PUSHED;
	}
	else if ((button->prevState == PUSHED) && (curState == RELEASED)) {
		//HAL_Delay(50);
		button->prevState = RELEASED;
		return ACT_RELEASED;
	}
	return NO_ACT;
}
