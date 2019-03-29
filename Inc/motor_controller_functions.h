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
void mcCmdTorque(uint16_t);
void mcCmdTorqueFake(uint16_t);
void disableMotorController();
void enableMotorController();

//Broadcast ID's
#define ID_RINEHART_TEMP1						0x0A0
#define ID_RINEHART_TEMP2						0x0A1
#define ID_RINEHART_TEMP3						0x0A2
#define ID_RINEHART_AIN_VOLT				0x0A3
#define ID_RINEHART_DIGIN_STATUS		0x0A4
#define ID_RINEHART_MOTOR_POS				0x0A5
#define ID_RINEHART_I_INFO					0x0A6
#define ID_RINEHART_V_INFO					0x0A7
#define ID_RINEHART_FLUX_INFO				0x0A8
#define ID_RINEHART_INTERN_VOLT			0x0A9
#define ID_RINEHART_INTERN_STATE		0x0AA
#define ID_RINEHART_FAULT_CODE			0x0AB
#define ID_RINEHART_TORQUE_TIM			0x0AC
#define ID_RINEHART_MOD_FLUX				0x0AD
#define ID_RINEHART_FIRMWARE				0x0AE
#define ID_RINEHART_DIAGNOSTIC			0x0AF

//Param Messages
#define ID_RINEHART_PARAM_CMD				0x0C1
#define ID_RINEHART_PARAM_RESP			0x0C2

//CAN Defines
#define RINEHART_FORWARD          0x1
#define RINEHART_BACKWARD         0x0
#define INVERTER_ENABLE           0x1   //set this bit to enable the MC
#define INVERTER_DISABLE          0x0   //clear this bit to disable the MC
#define INVERTER_DISCHARGE_ENABLE 0x2   //internal discharge resistor on
#define SPEED_MODE_ENABLE         0x4   //use speed instead of torque
#define CONFIGURE_HIGH            0
#define CONFIGURE_LOW             0x94
#define WRITE                     0x1
#define RESERVED                  0


//Brodcast configuration bytes Pages 16-17 of the Rinehart CAN Manual
//Data byte 4 commands
#define MC_TEMP1            0x1
#define MC_TEMP2            0x2
#define MC_TEMP3            0x4
#define ANALOG_IV           0x8   //Analog Input Voltages
#define DIGITAL_IS          0x10  //Digital Input Status
#define MOTOR_POS           0x20
#define MC_CURRENT          0x40
#define MC_VOLTAGE          0x80

//Data byte 5 commands
#define MC_FLUX             0x1
#define INTERNAL_VOLTAGE    0x2
#define INTERNAL_STATES     0x4
#define FAULT_CODES         0x8
#define TORQUE_TIMER        0x10

//Data byte 6 is not used

//Data byte 7 commands
#define PARAM_RESPONSE      0x20
#define PARAM_COMMAND       0x40
#define CAN_COMMAND         0x80  //can use this as a heartbeat


#endif /* MOTOR_CONTROLLER_FUNCTIONS_H_ */
