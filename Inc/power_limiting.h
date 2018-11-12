/*
 * power_limiting.h
 *
 *  Created on: Nov 9, 2018
 *      Author: Matt Flanagan
 */

#ifndef POWER_LIMITING_H_
#define POWER_LIMITING_H_

#include "BMS.h"
#include "car.h"

#define VOLT_THRESH		32000 //.0001 volts
#define VOLT_SOFT_LIM	30000
#define VOLT_HARD_LIM	28000

#define POWER_THRESH	75000 //when throttling takes place
#define POWER_SOFT_LIM	78000
#define POWER_HARD_LIM	80000

#define TEMP_THRESH		50 //when throttling takes place
#define TEMP_SOFT_LIM	55
#define TEMP_HARD_LIM	58

uint8_t power_limit_watt(int16_t torque_req);
uint8_t power_limit_temp(int16_t torque_req);
uint8_t power_limit_volt(int16_t torque_req);
int16_t limit_torque(int16_t torque_req);

#endif /* POWER_LIMITING_H_ */
