/*
 * power_limiting.c
 *
 *  Created on: Nov 9, 2018
 *      Author: Matt Flanagan
 */
#include "power_limiting.h"

float power_limit_watt(int16_t torque_req) {
	// calculate
	calcTorqueLimit = (80000 / (actualDC * 10 * actualV * 10)); //(DCLimit / (actualDC * 10)) * actualTorque;
	if(torque_to_send/MAX_THROTTLE_LEVEL > calcTorqueLimit)
	{
		torque_to_send = calcTorqueLimit * torque_to_send;
	}
	return 0;
}

float power_limit_temp(int16_t torque_req) {

	return 0.0;
}

float power_limit_volt(int16_t torque_req) {

	return 0.0;
}

int16_t limit_torque(int16_t torque_req) {
	int16_t torque_limited = 0;


	return torque_limited;
}
