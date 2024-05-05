/*
 * led.h
 *
 *  Created on: Apr 23, 2024
 *      Author: kccistc
 */

#ifndef DRIVER_LED_LED_H_
#define DRIVER_LED_LED_H_

#include "stm32f4xx_hal.h"

typedef struct {
	GPIO_TypeDef* GPIOx;
	uint16_t GPIO_Pin;
}led_t;

void led_init(led_t *led, GPIO_TypeDef* GPIOx, uint16_t GPIO_Pin);
void led_on(led_t *led);
void led_off(led_t *led);
void led_toggle(led_t *led);

#endif /* DRIVER_LED_LED_H_ */
