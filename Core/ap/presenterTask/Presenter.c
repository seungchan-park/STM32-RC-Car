/*
 * Presenter.c
 *
 *  Created on: Apr 30, 2024
 *      Author: kccistc
 */

#include "Presenter.h"

motor_t hLeftMotor, hRightMotor;

void Presenter_init(I2C_HandleTypeDef *hi2c, TIM_HandleTypeDef *htim)
{
	LCD_init(hi2c);
	Motor_init(&hLeftMotor, htim, TIM_CHANNEL_1, LEFT_DIR1_GPIO, LEFT_DIR1_GPIO_PIN, LEFT_DIR2_GPIO, LEFT_DIR2_GPIO_PIN);
	Motor_init(&hRightMotor, htim, TIM_CHANNEL_2, RIGHT_DIR1_GPIO, RIGHT_DIR1_GPIO_PIN, RIGHT_DIR2_GPIO, RIGHT_DIR2_GPIO_PIN);
	Motor_setSpeed(&hLeftMotor, 800);
	Motor_setSpeed(&hRightMotor, 800);
}

void Presenter_excuteTack()
{
	int motorQueFlag = MotorState_getFlag();

	if (!motorQueFlag) return;

	Presenter_setState();
	MotorState_setFlag(MOTOR_RESET);
}

void Presenter_setState()
{
	uint8_t motorState = MotorState_deQue();
	switch (motorState) {
	case GO:
		Presenter_CarGo();
		break;
	case BACK:
		Presenter_CarBack();
		break;
	case LEFT:
		Presenter_CarLeft();
		break;
	case RIGHT:
		Presenter_CarRight();
		break;
	case STOP:
		Presenter_CarStop();
		break;
	case SPEED:
		Presenter_CarSpeed();
		break;
	default:
		Presenter_CarStop();
		break;
	}
}

void Presenter_CarGo()
{
	LCD_writeStringXY(0, 0, "CAR Go!");
	Motor_forward(&hLeftMotor);
	Motor_forward(&hRightMotor);
}

void Presenter_CarStop()
{
	LCD_writeStringXY(0, 0, "CAR Stop!");
	Motor_stop(&hLeftMotor);
	Motor_stop(&hRightMotor);
}

void Presenter_CarLeft()
{
	LCD_writeStringXY(0, 0, "CAR Left!");
	Motor_backward(&hLeftMotor);
	Motor_forward(&hRightMotor);
}

void Presenter_CarRight()
{
	LCD_writeStringXY(0, 0, "CAR Right!");
	Motor_forward(&hLeftMotor);
	Motor_backward(&hRightMotor);
}

void Presenter_CarBack()
{
	LCD_writeStringXY(0, 0, "CAR Back!");
	Motor_backward(&hLeftMotor);
	Motor_backward(&hRightMotor);
}

void Presenter_CarSpeed()
{
	char buff[30];
	sprintf(buff, "CAR Speed:%d", MotorState_getSpeed());
	LCD_writeStringXY(1, 0, buff);
	Motor_setSpeed(&hLeftMotor, MotorState_getSpeed());
	Motor_setSpeed(&hRightMotor, MotorState_getSpeed());
}
