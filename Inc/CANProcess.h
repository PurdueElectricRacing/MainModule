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

//IMU defines
#define IMU_16G											4
#define IMU_ACCEL										0
#define IMU_8G_VAL									0x3FFF
#define IMU_8G_NEG									0xC001 //negative 0x3FFF


//Wheel Speed Defines
typedef enum
{
	WHEEL_L_7_0_BYTE   = 3,
	WHEEL_L_15_8_BYTE  = 2,
	WHEEL_L_23_16_BYTE = 1,
	WHEEL_L_31_24_BYTE = 0,

	WHEEL_R_7_0_BYTE   = 7,
	WHEEL_R_15_8_BYTE  = 6,
	WHEEL_R_23_16_BYTE = 5,
	WHEEL_R_31_24_BYTE = 4,
}WHEEL_DATA_INDEXES;



void ISR_RXCAN();


void taskRXCANProcess(void * params);
void taskTX_DCAN(void * params);
void taskTX_VCAN(void * params);
void taskRXCAN(void * params);

void processPedalboxFrame(CanRxMsgTypeDef* rx);
void prchg_led_enbl(uint8_t val);
void process_IMU(CanRxMsgTypeDef* rx);

void processCalibrate(CanRxMsgTypeDef* rx);
void processCalibratePowerLimit(CanRxMsgTypeDef* rx);
int process_bms_frame(CanRxMsgTypeDef* rx);

void send_ack(uint16_t can_id, uint16_t response);

#endif /* CANPROCESS_H_ */
