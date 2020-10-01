/*
 * bat_charge.h
 *
 *  Created on: Sep, 26 2020
 *      Author: Luke Oxley
 */

#ifndef BAT_CHARGE_H_
#define BAT_CHARGE_H_

#include <math.h>

#define DEVICE_ADDR 0x6B //before bit shifting


//writing
#define WRITE_MSG_SIZE    		1
#define WRITE_ENABLE      		0x01 //xxxx-xxx1
#define WRITE_TIMEOUT			1000

#define ICHG_CTRL_ADDR 0x13
#define PCHRGCTRL_ADDR 0x14
#define PCHRGCTRL_DEF 0x87 //what this register should be for 2.5mA steps

#define CHRG_CURRENT_STEPS 200 // mA = steps * 2.5mA

#define ICCTRL2_ADDR 0x37
#define ICCTRL2_DEF 0x40
#define CHRG_ENABLE 1
#define CHRG_DISABLE 0

//reading
#define READ_ENABLE       0x00 //xxxx-xxx0
#define READ_MSG_SIZE     1    //16 bits sent per msg

#define STAT0_ADDR 0x0
#define CURRENT_LIM_BIT 4

#define STAT1_ADDR 0x1
#define OVERVOLTAGE_BIT 7
#define HOT_BIT 0




//macros
#define set_address(address, write_en) (address << 1) | write_en
#define get_bit(num, n) (num >> n) & 1;

void task_manage_charger();
void setChargeEnable(uint8_t enable);

#endif /* BAT_CHARGE_H_ */