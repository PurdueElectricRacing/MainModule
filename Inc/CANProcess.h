/*
 * CANRXProcess.h
 *
 *  Created on: Dec 22, 2016
 *      Author: ben
 */

#ifndef CANPROCESS_H_
#define CANPROCESS_H_

//includes
#include "CAN_Bus.h"
#include "PedalBox.h"
//#include "WheelModule.h"

//wheel module defines
#define WM_SPEED_7_0_BYTE           2
#define WM_SPEED_11_8_BYTE          1
#define WM_SPEED_11_8_MASK          0x0F00


//void processPedalboxFrame(CanRxMsgTypeDef* rx);
void prchg_led_enbl(uint8_t val);

void processCalibrate(CanRxMsgTypeDef* rx);


#endif /* CANPROCESS_H_ */
