/*
 * motor_controller_functions.h
 *
 *  Created on: Dec 23, 2016
 *      Author: ben ng for purdue electric racing
 */

#ifndef MOTOR_CONTROLLER_FUNCTIONS_H_
#define MOTOR_CONTROLLER_FUNCTIONS_H_

#include "stm32f4xx_hal.h"
#include "cmsis_os.h"
#include "car.h"

//function prototypes
int taskProcessMotorControllerFrame();
void mcCmdTransmissionRequestPermenant (uint8_t regid, uint8_t retransmitTimeMS);
void mcCmdTransmissionRequestSingle(uint8_t regid);
void mcCmdTransmissionAbortPermenant(uint8_t regid);
void mcCmdTorque(uint16_t);
void mcCmdTorqueFake(uint16_t);
void disableMotorController();
void enableMotorController();

//CAN Defines
#define RINEHART_FORWARD 					0x1
#define RINEHART_BACKWARD					0x0
#define INVERTER_ENABLE 					0x1		//set this bit to enable the MC
#define INVERTER_DISABLE 					0x0		//clear this bit to disable the MC
#define INVERTER_DISCHARGE_ENABLE 0x2		//internal discharge resistor on
#define SPEED_MODE_ENABLE					0x4		//use speed instead of torque
#define CONFIGURE_HIGH      			0
#define CONFIGURE_LOW       			0x94
#define WRITE											0x1
#define RESERVED  								0


//Brodcast configuration bytes
//Data byte 4 commands
#define TEMP1               0x1
#define TEMP2								0x2
#define TEMP3								0x4
#define ANALOG_IV     			0x8
#define DIGITAL_IS          0x10
#define MOTOR_POS           0x20
#define CURRENT             0x40
#define VOLTAGE 		    		0x80

//Data byte 5 commands
#define FLUX                0x1
#define INTERNAL_VOLTAGE		0x2
#define INTERNAL_STATES	    0x4
#define FAULT_CODES     		0x8
#define TORQUE_TIMER        0x10

//Data byte 7 commands
#define PARAM_RESPONSE      0x20
#define PARAM_COMMAND       0x40
#define CAN_COMMAND 				0x80


#endif /* MOTOR_CONTROLLER_FUNCTIONS_H_ */
