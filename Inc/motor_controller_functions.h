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
#define RINEHART_FORWARD 0x1
#define RINEHART_BACKWARD	0x0
#define INVERTER_ENABLE 	0x1						//set this bit to enable the MC
#define INVERTER_DISABLE 	0x0						//clear this bit to disable the MC
#define INVERTER_DISCHARGE_ENABLE 0x2		//internal discharge resistor on
#define SPEED_MODE_ENABLE	0x4						//use speed instead of torque



#endif /* MOTOR_CONTROLLER_FUNCTIONS_H_ */
