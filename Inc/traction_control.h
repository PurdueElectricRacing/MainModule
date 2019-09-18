/*
 * traction_control.h
 *
 *  Created on: 29 March 2019
 *      Author: Jose Luis Tejada
 */

#include"car.h"

#ifndef TRACTION_CONTROL_H_
#define TRACTION_CONTROL_H_

typedef struct{
	float FL_rpm;
	float FR_rpm;
	float RR_rpm;
	float RL_rpm;
} wheel_speed_t;

#define MAX_TORQUE MAX_THROTTLE_LEVEL / 10 //Maximum Torque Allowed
#define MIN_TORQUE 0 //Minimum Torque Allowed
#define MAX_SLIP_RATIO 0.15 //Maximum Slip Ratio
#define MIN_SLIP_RATIO 0.05 //Minimum Slip Ratio
#define METHOD_USED 0 //1-Average Wheel Speed, 2-Maximize Slip Ratio.
#define WHEEL_RADIUS 0.4572 //Ideal radius of wheel in meters.
#define SAMPLE_TIME 100 //Milliseconds.

//Declarations
uint16_t TractionControl(uint32_t current_time, uint32_t * last_execution_time, uint16_t torque_pedal, uint16_t * integral_term, uint16_t * previous_torque);

#endif
