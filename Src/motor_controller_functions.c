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
