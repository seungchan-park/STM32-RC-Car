/*
 * led.c
 *
 *  Created on: Apr 23, 2024
 *      Author: kccistc
 */

#include "led.h"

void led_init(led_t *led, GPIO_TypeDef* GPIOx, uint16_t GPIO_Pin)
{
	led->GPIOx = GPIOx;
	led->GPIO_Pin = GPIO_Pin;
}

void led_on(led_t *led)
{
	HAL_GPIO_WritePin(led->GPIOx, led->GPIO_Pin, SET);
}

void led_off(led_t *led)
{
	HAL_GPIO_WritePin(led->GPIOx, led->GPIO_Pin, RESET);
}

void led_toggle(led_t *led)
{
	HAL_GPIO_TogglePin(led->GPIOx, led->GPIO_Pin);
}
