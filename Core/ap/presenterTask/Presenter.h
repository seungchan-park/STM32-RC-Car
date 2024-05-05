/*
 * Presenter.h
 *
 *  Created on: Apr 30, 2024
 *      Author: kccistc
 */

#ifndef AP_PRESENTERTASK_PRESENTER_H_
#define AP_PRESENTERTASK_PRESENTER_H_

#include "../model/MotorStateQue.h"
#include "../../driver/LCD/LCD.h"
#include "../../driver/motor/motor.h"
#include <stdio.h>

#define LEFT_DIR1_GPIO			GPIOC
#define LEFT_DIR1_GPIO_PIN		GPIO_PIN_7
#define LEFT_DIR2_GPIO			GPIOB
#define LEFT_DIR2_GPIO_PIN		GPIO_PIN_6
#define RIGHT_DIR1_GPIO			GPIOA
#define RIGHT_DIR1_GPIO_PIN		GPIO_PIN_8
#define RIGHT_DIR2_GPIO			GPIOB
#define RIGHT_DIR2_GPIO_PIN		GPIO_PIN_10

void Presenter_init(I2C_HandleTypeDef *hi2c, TIM_HandleTypeDef *htim);

void Presenter_excuteTack();
void Presenter_setState();
void Presenter_CarGo();
void Presenter_CarStop();
void Presenter_CarLeft();
void Presenter_CarRight();
void Presenter_CarBack();
void Presenter_CarSpeed();

#endif /* AP_PRESENTERTASK_PRESENTER_H_ */
