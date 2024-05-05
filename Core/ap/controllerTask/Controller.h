/*
 * Controller.h
 *
 *  Created on: Apr 30, 2024
 *      Author: kccistc
 */

#ifndef AP_CONTROLLERTASK_CONTROLLER_H_
#define AP_CONTROLLERTASK_CONTROLLER_H_

#include "../model/BT_dataQue.h"
#include "../model/MotorStateQue.h"
#include <stdlib.h>

void Controller_init();
void Controller_excuteTask();
void Controller_AutoTask();
void Controller_ManualTask();
void Controller_BTgetData(uint8_t *buff);
void Controller_ULgetData(uint8_t *buff);
void Controller_BTparsingData(uint8_t *buff);
void Controller_ULparsingData(uint8_t *buff);

#endif /* AP_CONTROLLERTASK_CONTROLLER_H_ */
