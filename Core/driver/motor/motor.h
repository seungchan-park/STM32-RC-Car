/*
 * motor.h
 *
 *  Created on: Apr 25, 2024
 *      Author: kccistc
 */

#ifndef DRIVER_MOTOR_MOTOR_H_
#define DRIVER_MOTOR_MOTOR_H_
#include "stm32f4xx_hal.h"

typedef struct {
	TIM_HandleTypeDef *htim;
	uint32_t Channel;
	GPIO_TypeDef *dir1_GPIO;
	uint16_t dir1_GPIO_PIN;
	GPIO_TypeDef *dir2_GPIO;
	uint16_t dir2_GPIO_PIN;
}motor_t;

void Motor_init(motor_t *motor,
		TIM_HandleTypeDef *htim,
		uint32_t Channel,
		GPIO_TypeDef *dir1_GPIO,
		uint16_t dir1_GPIO_PIN,
		GPIO_TypeDef *dir2_GPIO,
		uint16_t dir2_GPIO_PIN);
void Motor_stop(motor_t *motor);
void Motor_forward(motor_t *motor);
void Motor_backward(motor_t *motor);
void Motor_setSpeed(motor_t *motor, int speedVal);

#endif /* DRIVER_MOTOR_MOTOR_H_ */
