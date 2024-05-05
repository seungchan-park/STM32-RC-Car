/*
 * Listener.h
 *
 *  Created on: Apr 30, 2024
 *      Author: kccistc
 */

#ifndef AP_LISTENERTASK_LISTENER_H_
#define AP_LISTENERTASK_LISTENER_H_

#include "../model/BT_dataQue.h"
#include "../../driver/button/button.h"
#include "../../driver/Ultrasonic/Ultrasonic.h"

#define FRONT_GPIO_TRIG			GPIOC
#define FRONT_GPIO_TRIG_PIN		GPIO_PIN_0
#define FRONT_GPIO_ECHO			GPIOC
#define FRONT_GPIO_ECHO_PIN		GPIO_PIN_1

#define LEFT_GPIO_TRIG			GPIOB
#define LEFT_GPIO_TRIG_PIN		GPIO_PIN_0
#define LEFT_GPIO_ECHO			GPIOA
#define LEFT_GPIO_ECHO_PIN		GPIO_PIN_4

#define RIGHT_GPIO_TRIG			GPIOC
#define RIGHT_GPIO_TRIG_PIN		GPIO_PIN_2
#define RIGHT_GPIO_ECHO			GPIOC
#define RIGHT_GPIO_ECHO_PIN		GPIO_PIN_3

void Listener_BT_ISR_Process(uint8_t rxData);
void Listener_UlraSonic_ISR_Process(uint16_t GPIO_Pin);
void Listener_init();
void Listener_eventCheck();
void Listener_excuteTask();

#endif /* AP_LISTENERTASK_LISTENER_H_ */
