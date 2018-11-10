/*
 * power_limiting.c
 *
 *  Created on: Nov 9, 2018
 *      Author: Matt Flanagan
 */
#include "power_limiting.h"

int16_t power_limit_watt(int16_t torque_req) {
	// calculate
//	calcTorqueLimit = (80000 / (actualDC * 10 * actualV * 10)); //(DCLimit / (actualDC * 10)) * actualTorque;
//	if(torque_to_send/MAX_THROTTLE_LEVEL > calcTorqueLimit)
//	{
//		torque_to_send = calcTorqueLimit * torque_to_send;
//	}

	return 0;
}

int16_t power_limit_temp(int16_t torque_req) {

	return 0.0;
}

int8_t power_limit_volt(int16_t torque_req) {
	//only throttle if past the threshold
	if (bms.low_cell_volt > VOLT_THRESH) return 100;
	//if past the hard lim stop the driving
	if (bms.low_cell_volt < VOLT_HARD_LIM) {
		bms.battery_violation = 3;
		return 0;
	}

	//between thresh hold and hard limit


	return 0.0;
}

int16_t limit_torque(int16_t torque_req) {
	int16_t torque_limited = torque_req;
	int16_t watt_gain = 100; //100 means gain of 1.
	int16_t temp_gain = 100;
	int16_t volt_gain = 100;

	watt_gain = power_limit_watt(torque_req);
	temp_gain = power_limit_temp(torque_req);
	volt_gain = power_limit_volt(torque_req);

	if (watt_gain < temp_gain && watt_gain < volt_gain) {
		//power limited by rules
		torque_limited = (torque_limited * watt_gain) / 100;
	} else if (temp_gain < watt_gain && temp_gain < volt_gain) {
		//power limited by temp
		torque_limited = (torque_limited * temp_gain) / 100;
	} else {
		//power limited by voltage
		torque_limited = (torque_limited * volt_gain) / 100;
	}

	return torque_limited;
}
