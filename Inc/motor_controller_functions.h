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
void param_request(uint16_t param_addr, uint8_t rw_cmd, uint16_t data);


//Broadcast ID's
// @TODO REMOVE THE BELOW BULL CRAP
// #define ID_RINEHART_TEMP1						0x0A0
// #define ID_RINEHART_TEMP2						0x0A1
// #define ID_RINEHART_TEMP3						0x0A2
// #define ID_RINEHART_AIN_VOLT				0x0A3
// #define ID_RINEHART_DIGIN_STATUS		0x0A4
// #define ID_RINEHART_MOTOR_POS				0x0A5
// #define ID_RINEHART_I_INFO					0x0A6
// #define ID_RINEHART_V_INFO					0x0A7
// #define ID_RINEHART_FLUX_INFO				0x0A8
// #define ID_RINEHART_INTERN_VOLT			0x0A9
// #define ID_RINEHART_INTERN_STATE		0x0AA
// #define ID_RINEHART_FAULT_CODE			0x0AB
// #define ID_RINEHART_TORQUE_TIM			0x0AC
// #define ID_RINEHART_MOD_FLUX				0x0AD
// #define ID_RINEHART_FIRMWARE				0x0AE
// #define ID_RINEHART_DIAGNOSTIC			0x0AF

// //Param Messages
// #define ID_RINEHART_PARAM_CMD				0x0C1
// #define ID_RINEHART_PARAM_RESP			0x0C2

enum 
{
  //CAN Defines
  RINEHART_FORWARD  = 1,
  RINEHART_BACKWARD = 0,
  INVERTER_ENABLE   = 1,   //set this bit to ,enable the MC
  INVERTER_DISABLE = 0,  //clear this bit to ,disable the MC
  INVERTER_DISCHARGE_ENABLE = 2,   //internal discharge ,resistor on
  SPEED_MODE_ENABLE = 4,   //use speed instead of ,torque
  CONFIGURE_HIGH = 0,
  CONFIGURE_LOW  = 0x94,
  WRITE    = 1,
  RESERVED = 0,
  //Broadcast configuration bytes Pages 16-17 of the Rinehart CAN Manual
  //Data byte 4 commands
  MC_TEMP1 =  0x1,
  MC_TEMP2 =  0x2,
  MC_TEMP3 =  0x4,
  ANALOG_IV  = 0x8,   //Analog Input Voltages,
  DIGITAL_IS = 0x10,  //Digital Input Status,
  MOTOR_POS  = 0x20,
  MC_CURRENT = 0x40,
  MC_VOLTAGE = 0x80,
  //Data byte 5 commands
  MC_FLUX = 0x1,
  INTERNAL_VOLTAGE = 0x2,
  INTERNAL_STATES  = 0x4,
  FAULT_CODES  = 0x8,
  TORQUE_TIMER = 0x10,
  //Data byte 6 is not used
  //Data byte 7 commands
  PARAM_RESPONSE = 0x20,
  PARAM_COMMAND  = 0x40,
  CAN_COMMAND    = 0x80,  //can use this as a ,heartbeat
} RINEHART_CONFIG_BYTES_t;

#endif /* MOTOR_CONTROLLER_FUNCTIONS_H_ */
