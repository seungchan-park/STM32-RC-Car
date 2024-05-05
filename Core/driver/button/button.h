/*
 * button.h
 *
 *  Created on: Apr 23, 2024
 *      Author: kccistc
 */

#ifndef DRIVER_BUTTON_BUTTON_H_
#define DRIVER_BUTTON_BUTTON_H_

#include "stm32f4xx_hal.h"

typedef enum {PUSHED=0, RELEASED, NO_ACT, ACT_PUSHED, ACT_RELEASED} button_state_t;

typedef struct {
	GPIO_TypeDef* GPIOx;
	uint16_t GPIO_Pin;
	int prevState;
}button_t;

void button_init(button_t *button, GPIO_TypeDef* GPIOx, uint16_t GPIO_Pin);
button_state_t button_getState(button_t *button);

#endif /* DRIVER_BUTTON_BUTTON_H_ */
