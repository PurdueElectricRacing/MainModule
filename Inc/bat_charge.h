/*
 * bat_charge.h
 *
 *  Created on: Sep, 26 2020
 *      Author: Luke Oxley
 */

#ifndef BAT_CHARGE_H_
#define BAT_CHARGE_H_

#include "main.h"

#define PER 1
#define GREAT 1

typedef enum{
  DEVICE_ADDR = 0x6B, //before bit shifting
  ICHG_CTRL_ADDR = 0x13,
  PCHRGCTRL_ADDR = 0x14,
  ICCTRL2_ADDR = 0x37,
  STAT0_ADDR = 0x0,
  STAT1_ADDR = 0x1

} Charger_Address_t;

//writing
#define WRITE_MSG_SIZE    		1
#define WRITE_ENABLE      		0x01 //xxxx-xxx1
#define WRITE_TIMEOUT			1000

#define PCHRGCTRL_DEF 0x87 //what this register should be for 2.5mA steps
#define ICCTRL2_DEF 0x40

#define CHRG_CURRENT_STEPS 200 // mA = steps * 2.5mA

//reading
#define READ_ENABLE       0x00 //xxxx-xxx0
#define READ_MSG_SIZE     1    //16 bits sent per msg

typedef enum{
  CURRENT_LIM_MASK = 0b00010000,
  OVERVOLTAGE_MASK = 0b10000000,
  HOT_MASK = 0b00000001,
  CHRG_ENABLE_MASK = 0b00000000,
  CHRG_DISABLE_MASK = 0b00000001
} MaskType_t;


//macros
#define SET_ADDRESS(address, write_en) (address << 1) | write_en
#define GET_BIT(num, mask) (num & mask)

void task_manage_charger();
void setChargeEnable(uint8_t enable);

#endif /* BAT_CHARGE_H_ */
