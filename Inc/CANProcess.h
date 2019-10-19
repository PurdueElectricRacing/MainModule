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

//void processPedalboxFrame(CanRxMsgTypeDef* rx);
void prchg_led_enbl(uint8_t val);
void add_to_CAN_queue(CAN_HandleTypeDef * hcan);


#endif /* CANPROCESS_H_ */
