/*
 * BT_dataQue.h
 *
 *  Created on: Apr 30, 2024
 *      Author: kccistc
 */

#ifndef AP_MODEL_BT_DATAQUE_H_
#define AP_MODEL_BT_DATAQUE_H_

#include "../../common/queue/queue.h"
#include <stdint.h>

#define BT_SET		1
#define BT_RESET	0
#define UL_SET		1
#define UL_RESET	0

#define MANUAL 0
#define AUTO 1

void BT_init();
void BT_enQue(uint8_t rxData);
uint8_t BT_deQue();
void BT_setFlag(int flagState);
int BT_getFlag();
void Mode_set(int mode);
int Mode_get();
void UL_enQue(uint8_t rxData);
uint8_t UL_deQue();
void UL_setFlag(int flagState);
int UL_getFlag();

#endif /* AP_MODEL_BT_DATAQUE_H_ */
