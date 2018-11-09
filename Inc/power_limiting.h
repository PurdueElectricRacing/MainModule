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

float power_limit_watt(int16_t torque_req);
float power_limit_temp(int16_t torque_req);
float power_limit_volt(int16_t torque_req);
int16_t limit_torque(int16_t torque_req);

#endif /* POWER_LIMITING_H_ */
