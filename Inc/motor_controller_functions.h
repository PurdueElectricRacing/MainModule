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

#define MAX_REGEN_CURRENT 40.0f

//function prototypes
void mcCmdTorque(uint16_t);
void mcCmdTorqueFake(uint16_t);
void disableMotorController();
void enableMotorController();
void param_request(uint16_t param_addr, uint8_t rw_cmd, uint16_t data);
int16_t brake_pres_regen_torque(float brake_pres);
int16_t throttle_pos_regen_torque(float throttle_pos);

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
