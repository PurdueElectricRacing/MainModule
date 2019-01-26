/***************************************************************************
*
*     File Information
*
*     Name of File: main_module_tasks.c
*
*     Authors (Include Email):
*       1. Ben Ng				xbenng@gmail.com
*
*     File dependents: (header files, flow charts, referenced documentation)
*       1. motor_controller_functions.h
*       http://www.unitek-online.de/pdf/download/Antriebe-Drive/Servo-Digital/E-DS-CAN.pdf
*     	http://www.unitek-online.de/pdf/download/Antriebe-Drive/Servo-Digital/E-DS-NDrive.pdf
*
*
*     File Description:
*     	Functions to control the motor controller.

***************************************************************************/
#include "car.h"
#include "motor_controller_functions.h"
#include "CANProcess.h"

/***************************************************************************
*
*     Function Information
*
*     Name of Function: processPedalboxFrame
*
*     Programmer's Name: Raymond Dong, dong155@purdue.edu
*
*     Function Return Type: none
*
*     Parameters (list data type, name, and comment one per line):
*		1.
*
*     Function Description: TODO: update for Rinehart
*
*
***************************************************************************/
int taskProcessMotorControllerFrame() {
	//CanRxMsgTypeDef rx;

//	while (1) {
//		if(xQueueReceive(car.q_mc_frame, &rx, 100))
//		{
//			//received a message from motor controller.
//			/*
//			{actual torque}				{actual dc current}			{dc current limit}
//			{calculated torque limit}	{pedal torque}
//			{calculated torque limit} = {dc current limit}/{actual dc current} * {actual torque}
//			if {calculated torque limit} > {pedal torque}
//				send {calculated torque limit}
//			else
//				send {pedal torque}
//			*/
//			//todo add semaphores around the torque values that are being updated
//			if (rx.Data[0] == REGID_I_ACT)
//			{
//				actualTorque0700 = rx.Data[1];
//				actualTorque1508 = rx.Data[2];
//				actualTorque = actualTorque0700 | (actualTorque1508 << 8);
//				BCparam = 1;    //actual torque received
//			}
//			if (rx.Data[0] == REGID_SPEED_ACTUAL)
//			{
//				speedActual = (rx.Data[1] << 8 | rx.Data[2]);
//				BCparam = 2;    //speed actual received
//			}
//			if (rx.Data[0] == REGID_I_IST)
//			{
//				currentActual = (rx.Data[1] << 8 | rx.Data[2]);
//				BCparam = 3;    //current actual received
//			}
//			if (rx.Data[0] == REGID_I_SOLL)
//			{
//				commandCurrent = (rx.Data[1] << 8 | rx.Data[2]);
//				BCparam = 4;    //command current received
//			}
//			if (rx.Data[0] == REGID_DC_BUS)
//			{
//				dcBusVoltage = (rx.Data[1] << 8 | rx.Data[2]);
//				BCparam = 5;    //current actual received
//			}
//			if (rx.Data[0] == REGID_T_MOTOR)
//			{
//				motorTemperature = (rx.Data[1] << 8 | rx.Data[2]);
//				BCparam = 6;    //motor temperature received
//			}
//			if (rx.Data[0] == REGID_T_IGBT)
//			{
//				powerStageTemperature = (rx.Data[1] << 8 | rx.Data[2]);
//				BCparam = 7;    //power stage temperature received
//			}
//			if (rx.Data[0] == REGID_T_AIR)
//			{
//				airTemperature = (rx.Data[1] << 8 | rx.Data[2]);
//				BCparam = 8;    //air temperature received
//			}
//			if (rx.Data[0] == REGID_I_REDA)
//			{
//				actualCurrentLimit = (rx.Data[1] << 8 | rx.Data[2]);
//				BCparam = 9;    //actual current limit received
//			}
//			if (rx.Data[0] == REGID_ERR_BITMAP1)
//			{
//				errBitMap1 = (rx.Data[1] << 8 | rx.Data[2]);
//				BCparam = 10;    //errBitMap1 received
//			}
//		}
//	}
	return 0;
}

void mcCmdTorque(uint16_t torqueVal) {
	CanTxMsgTypeDef tx;
	tx.IDE = 			CAN_ID_STD;
	tx.StdId = 		ID_RINEHART_STATION_TX;
	tx.DLC = 			8;
	tx.RTR =			CAN_RTR_DATA;
	tx.Data[0] = 	(uint8_t) torqueVal;	//torque command byte 1 since enabling make it go zero speed don't want anyone dead...
	tx.Data[1] =	(uint8_t) (torqueVal >> 8);	//torque command byte 2
	tx.Data[2] = 	DONT_CARE;	//speed command byte 1 (irrelevant since using torque)
	tx.Data[3] =	DONT_CARE;	//speed command byte 2
	tx.Data[4] = 	RINEHART_FORWARD;	//Direction Command 0=back, 1=forward
	tx.Data[5] =	INVERTER_ENABLE;	//Enables inverter, discharge, speed mode
	tx.Data[6] = 	0x0;	//torque command byte 1 (if set to zero will default to EEPROM which is desired)
	tx.Data[7] =	0x0;	//torque command byte 2

	xQueueSendToFront(car.q_tx_vcan, &tx, 100); //higher priority than polling
}

void mcCmdTorqueFake(uint16_t torqueVal) {
	CanTxMsgTypeDef tx;
	tx.IDE = 			CAN_ID_STD;
	tx.StdId = 		0x490;
	tx.DLC = 			8;
	tx.RTR =			CAN_RTR_DATA;
	tx.Data[0] = 	(uint8_t) torqueVal;	//torque command byte 1 since enabling make it go zero speed don't want anyone dead...
	tx.Data[1] =	(uint8_t) (torqueVal >> 8);	//torque command byte 2
	tx.Data[2] = 	DONT_CARE;	//speed command byte 1 (irrelevant since using torque)
	tx.Data[3] =	DONT_CARE;	//speed command byte 2
	tx.Data[4] = 	RINEHART_FORWARD;	//Direction Command 0=back, 1=forward
	tx.Data[5] =	INVERTER_ENABLE;	//Enables inverter, discharge, speed mode
	tx.Data[6] = 	0x0;	//torque command byte 1 (if set to zero will default to EEPROM which is desired)
	tx.Data[7] =	0x0;	//torque command byte 2

	xQueueSendToBack(car.q_tx_dcan, &tx, 100);
}

/***************************************************************************
*
*     Function Information
*
*     Name of Function: configbroadcast
*
*     Programmer's Name: Tamim Noor
*     				     Enrico William
*     				     David Farrell
*     				     Elijah Peters
*
*     Function Return Type: void
*
*     Parameters (list data type, name, and comment one per line):
*       1. inputArray,
*
*      Global Dependents:
*      car.q_tx_dcan
*
*     Function Description:
*			broadcasts configurations pg 16 - 17 on RMS CAN Protocol
***************************************************************************/
void configbroadcast (uint8_t * inputArray) {
	CanTxMsgTypeDef tx;
	tx.IDE = 			CAN_ID_STD;
	tx.StdId = 		ID_RINEHART_STATION_TX;
	tx.DLC = 			8;
	tx.RTR =			CAN_RTR_DATA;
	tx.Data[0] = 	CONFIGURE_LOW; //parameter address
	tx.Data[1] =	CONFIGURE_HIGH;	//parameter address
	tx.Data[2] = 	WRITE;//R/W Command
	tx.Data[3] =	RESERVED;	//RESERVED
	tx.Data[4] = 	inputArray[0];	//First Data Command
	tx.Data[5] =	inputArray[1];	//Second Data Command
	tx.Data[6] = 	inputArray[2];	//Third Data Command
	tx.Data[7] =	inputArray[3];	//Fourth Data Command

	xQueueSendToBack(car.q_tx_dcan, &tx, 100);
}

//this funciton is used to start an auto-broadcast of torque being sent to motor
//we can implement something similar to this if necessary. TODO:what to do with this...
void mcCmdTransmissionRequestPermenant (uint8_t regid, uint8_t retransmitTimeMS) {
	//example 10, BAMOCAR CAN MANUAL
//	CanTxMsgTypeDef tx;
//	tx.IDE = 		CAN_ID_STD;
//	tx.StdId = 		ID_BAMOCAR_STATION_TX;
//	tx.DLC = 		DLC_CMD_REQUEST_DATA;
//	tx.Data[0] = 	REGID_CMD_REQUEST_DATA;
//	tx.Data[1] =	regid;
//	tx.Data[2] =	(uint8_t) retransmitTimeMS;
//	xQueueSendToBack(car.q_tx_vcan, &tx, 100);
	return;
}

//this funciton is used to start an onetime-broadcast of torque being sent to motor
//we can implement something similar to this if necessary. TODO:what to do with this...
void mcCmdTransmissionRequestSingle(uint8_t regid) {
	//example 10, BAMOCAR CAN MANUAL
//	CanTxMsgTypeDef tx;
//	tx.IDE = 		CAN_ID_STD;
//	tx.StdId = 		ID_BAMOCAR_STATION_TX;
//	tx.DLC = 		DLC_CMD_REQUEST_DATA;
//	tx.Data[0] = 	REGID_CMD_REQUEST_DATA;
//	tx.Data[1] =	regid;
//	tx.Data[2] =	RETRANSMISSION_SINGLE;
//	xQueueSendToBack(car.q_tx_vcan, &tx, 100);
	return;
}

//this funciton is used to end an auto-broadcast of torque being sent to motor
//we can implement something similar to this if necessary. TODO:what to do with this...
void mcCmdTransmissionAbortPermenant(uint8_t regid) {
	//example 10, BAMOCAR CAN MANUAL
//	CanTxMsgTypeDef tx;
//	tx.IDE = 		CAN_ID_STD;
//	tx.StdId = 		ID_BAMOCAR_STATION_TX;
//	tx.DLC = 		DLC_CMD_REQUEST_DATA;
//	tx.Data[0] = 	REGID_CMD_REQUEST_DATA;
//	tx.Data[1] =	regid;
//	tx.Data[2] =	RETRANSMISSION_ABORT;
//	xQueueSendToBack(car.q_tx_vcan, &tx, 100);
	return;
}

void disableMotorController()
/***************************************************************************
*
*     Function Information
*
*     Name of Function: disableMotor
*
*     Programmer's Name: Ben Ng
*
*     Function Return Type: void
*
*     Parameters (list data type, name, and comment one per line):
*       1.Pedalbox_msg_t msg
			brake_level from pedalbox potentiometer
*			throttle_level from pedalbox potentiometer
*			APPS_Implausible flag
*			EOR flag
*		2.
*
*      Global Dependents:
*
*     Function Description:
*			sends 0 torque, then disables RUN, and REF
***************************************************************************/
{
	CanTxMsgTypeDef tx;
	tx.IDE = 			CAN_ID_STD;
	tx.StdId = 		ID_RINEHART_STATION_TX;
	tx.DLC = 			8;
	tx.Data[0] = 	0x0;	//torque command byte 1 since enabling make it go zero speed don't want anyone dead...
	tx.Data[1] =	0x0;	//torque command byte 2
	tx.Data[2] = 	DONT_CARE;	//speed command byte 1 (irrelevant since using torque)
	tx.Data[3] =	DONT_CARE;	//speed command byte 2
	tx.Data[4] = 	RINEHART_FORWARD;	//Direction Command 0=back, 1=forward
	tx.Data[5] =	INVERTER_DISABLE;	//Enables inverter, discharge, speed mode
	tx.Data[6] = 	0x0;	//torque command byte 1 (if set to zero will default to EEPROM which is desired)
	tx.Data[7] =	0x0;	//torque command byte 2

	xQueueSendToBack(car.q_tx_vcan, &tx, 100);
}

void enableMotorController() {
/***************************************************************************
*
*     Function Information
*
*     Name of Function: initMotorController
*
*     Programmer's Name: Ben Ng
*
*     Function Return Type: void
*
*     Parameters (list data type, name, and comment one per line):
*       1.
*
*      Global Dependents:
*
*     Function Description: See Rinehart Can Manual or Rinehart Can Table in Google Drive
*		Initializes the motor controller
*
***************************************************************************/

	CanTxMsgTypeDef tx;
	tx.IDE = 			CAN_ID_STD;
	tx.StdId = 		ID_RINEHART_STATION_TX;
	tx.DLC = 			8;
	tx.Data[0] = 	0x0;	//torque command byte 1 since enabling make it go zero speed don't want anyone dead...
	tx.Data[1] =	0x0;	//torque command byte 2
	tx.Data[2] = 	DONT_CARE;	//speed command byte 1 (irrelevant since using torque)
	tx.Data[3] =	DONT_CARE;	//speed command byte 2
	tx.Data[4] = 	RINEHART_FORWARD;	//Direction Command 0=back, 1=forward
	tx.Data[5] =	INVERTER_ENABLE;	//Enables inverter, discharge, speed mode
	tx.Data[6] = 	0x0;	//torque command byte 1 (if set to zero will default to EEPROM which is desired)
	tx.Data[7] =	0x0;	//torque command byte 2

	xQueueSendToBack(car.q_tx_vcan, &tx, 100);
}


void taskMotorControllerPoll(void* param)
/***************************************************************************
*
*     Function Information
*
*     Name of Function: taskMotorControllerPoll
*
*     Programmer's Name: Ben Ng
*
*     Function Return Type: void
*
*     Parameters (list data type, name, and comment one per line):
*       1. none
*
*     Global Dependents:
*
*     Function Description: TODO: Adapt this to Rinehart if desired
*		This task requsts frames from the motor controller at a constant interval
*		it does this without verifying that the frames have been sent back
*		consider doing this with verification and a read requested flag
*
***************************************************************************/
{
//	while(1)
//		{
//			//0x03B
//			//length 5
//			//byte0 - Pack Current - 2 bytes - MSB First - Big Endian - 0.1A
//			//byte1 - IN USE - 1 bytes - MSB First
//			//byte2 - Pack Inst. Voltage - 2 bytes - MSB First - Big Endian - * (1/10) - 0.1V
//			//byte3 - IN USE - 1 bytes - MSB First
//			//byte4 - CRC Checksum - 1 bytes - MSB First  - +64
//			//0x03C
//			//length 2
//			//byte0 - Pack DCL - 2 bytes - MSB First - Big Endian - 1A
//			//byte1 - IN USE - 1bytes - MSB First
//			// Request Parameters
//			BCparam = 0;            // BCparam 0 - Nothing received
//			HAL_GPIO_TogglePin(LD6_GPIO_Port, LD6_Pin);
//			while(BCparam != 1) {
//				mcCmdTransmissionRequestSingle(REGID_I_ACT);
//				vTaskDelay(POLL_DELAY);
//			}    // BCparam 1 - actualTorque received
//			while(BCparam != 2) {
//				mcCmdTransmissionRequestSingle(REGID_SPEED_ACTUAL);
//				vTaskDelay(POLL_DELAY);
//			}    // BCparam 2 - speed actual received
//			while(BCparam != 3) {
//				mcCmdTransmissionRequestSingle(REGID_I_IST);
//				vTaskDelay(POLL_DELAY);
//			}    // BCparam 3 - current actual received
//			while(BCparam != 4) {
//				mcCmdTransmissionRequestSingle(REGID_I_SOLL);
//				vTaskDelay(POLL_DELAY);
//			}    // BCparam 4 - command current received
//			while(BCparam != 5) {
//				mcCmdTransmissionRequestSingle(REGID_DC_BUS);
//				vTaskDelay(POLL_DELAY);
//			}    // BCparam 5 - current actual received
//			while(BCparam != 6) {
//				mcCmdTransmissionRequestSingle(REGID_T_MOTOR);
//				vTaskDelay(POLL_DELAY);
//			}    // BCparam 6 - motor temperature received
//			while(BCparam != 7) {
//				mcCmdTransmissionRequestSingle(REGID_T_IGBT);
//				vTaskDelay(POLL_DELAY);
//			}    // BCparam 7 - power stage temperature received
//			while(BCparam != 8) {
//				mcCmdTransmissionRequestSingle(REGID_T_AIR);
//				vTaskDelay(POLL_DELAY);
//			}    // BCparam 8 - air temperature received
//			while(BCparam != 9) {
//				mcCmdTransmissionRequestSingle(REGID_I_REDA);
//				vTaskDelay(POLL_DELAY);
//			}    // BCparam 9 - actual current limit received
//			while(BCparam != 10) {
//				mcCmdTransmissionRequestSingle(REGID_ERR_BITMAP1);
//				vTaskDelay(POLL_DELAY);
//			}    // BCparam 10 - errBitMap1 received
//
//		}
	return;
}


