#include "traction_control.h"
#include "car.h"

/***************************************************************************
 *
 *     File Information
 *
 *     Name of File: car.c
 *
 *     Authors (Include Email):
 *       1. Jose Luis Tejada				tejada@purdue.edu
 *
 *     File dependents: (header files, flow charts, referenced documentation)
 *       1. traction_control.h
 *
 *     File Description:
 *     	Function to control torque modulation for improved traction
 *
 ***************************************************************************/


uint16_t TractionControl(uint32_t current_time, uint32_t * last_execution_time, uint16_t torque_pedal, uint16_t * integral_term, uint16_t * previous_torque)
{
	//Determine if sufficient time has passed for execution.
	uint32_t time_change = current_time - *last_execution_time;
	uint16_t desired_torque = 0;

	if (time_change >= SAMPLE_TIME)
	{
		//Wheel Speeds in m/s
		float wheel_speed_FL = car.wheel_rpm.FL_rpm * 60 * (2*M_PI*WHEEL_RADIUS);
		float wheel_speed_FR = car.wheel_rpm.FR_rpm * 60 * (2*M_PI*WHEEL_RADIUS);
		float wheel_speed_RL = car.wheel_rpm.RL_rpm * 60 * (2*M_PI*WHEEL_RADIUS);
		float wheel_speed_RR = car.wheel_rpm.RR_rpm * 60 * (2*M_PI*WHEEL_RADIUS);
		float slip_ratio = 0;

		// Processing Data
		// Preliminary setup: Using averaged from and back wheel speeds to generate slip ratio,
		// which is then used to modulate the torque commands appropriately. Could further
		// expand to then use to modulate the torque for left/right wheels independently.
		if (METHOD_USED == 1)
		{
			//Method 1: Front/back wheel average speed.
			float averaged_front = (wheel_speed_FR + wheel_speed_FL) / 2;
			float averaged_back = (wheel_speed_RR + wheel_speed_RL) / 2;
			if (averaged_back == 0) //Prevents Divide over 0 error
			{
				slip_ratio = 0;
			}
			else
			{
				slip_ratio = (averaged_back - averaged_front) / averaged_back;
			}

		}
		else
		{
			// Method 2: Front Minimum Speed, Back Maximum Speed, results in higher slip ratio,
			// greater torque reduction, also useful during turning as wheel speeds vary.
			float min_front = 0;
			float max_back = 0;

			//Find Front minimum speed.
			if (wheel_speed_FL > wheel_speed_FR)
			{
				min_front = wheel_speed_FR;
			}
			else
			{
				min_front = wheel_speed_FL;
			}

			//Find Back maximum speed.
			if (wheel_speed_RL > wheel_speed_RR)
			{
				max_back = wheel_speed_RL;
			}
			else
			{
				max_back = wheel_speed_RR;
			}

			if (max_back == 0) //Prevents Divide over 0 error
			{
				slip_ratio = 0;
			}
			else
			{
				slip_ratio = (max_back - min_front) / max_back;
			}
		}


		//Slip Ratio Logic
		//If the slip ratio is within acceptable limits, the desired torque to be
		//sent to the motor controller is the torque sent by the pedal box.
		//Otherwise, if the slip ratio exceeds limit, PID controller calculates
		//optimal torque output so that slip ratio is maintained within acceptable limits
		if ((slip_ratio > MIN_SLIP_RATIO) & (slip_ratio < MAX_SLIP_RATIO)) //Ideal Slip Ratio
		{
			*integral_term = 0;
			*previous_torque = 0;
			desired_torque = torque_pedal; //Sets the torque outputted to MC to be current torque.
		}
		else if((slip_ratio < MIN_SLIP_RATIO)) //Used for Debug Purposes (We CANNOT increase torque, rule EV.2.2.3)
		{
			desired_torque = torque_pedal; //Sets the torque outputted to MC to be current torque.
		}
		else
		{
			//Constants --> Still to be experimentally determined. Placeholder Values.
			float proportional_gain = 1000;
			float integral_gain = -10;
			float derivative_gain = 100;
			float target_slip_ratio = (MAX_SLIP_RATIO + MIN_SLIP_RATIO) / 2; //Average between boundaries of ideal range for slip ratio.
			float error_slip_ratio = target_slip_ratio - slip_ratio;

			//Calculation of PID terms
			//Proportional Term Calculations
			float proportional_term = proportional_gain * error_slip_ratio;

			//Integral Term Calculations
			*integral_term += (integral_gain * error_slip_ratio);
			if (*integral_term < MIN_TORQUE)
			{
				*integral_term = MIN_TORQUE;
			}
			else if (*integral_term > MAX_TORQUE)
			{
				*integral_term = MAX_TORQUE;
			}

			float derivative_term = (torque_pedal - *previous_torque);
			//Caution: Casting float to unsigned int.
			desired_torque = (uint16_t)((proportional_term) +  (*integral_term) + (derivative_gain *  derivative_term));


			if (desired_torque > torque_pedal) //Prevents torque output greater than pedal torque command.s
			{
				desired_torque = torque_pedal;
			}
			else if (desired_torque < MIN_TORQUE)
			{
				desired_torque = MIN_TORQUE;
			}

			//Post-execution tasks
			*last_execution_time = current_time; //Set Last Execution time to current time.
			*previous_torque = torque_pedal; //Set Previous Torque to the current torque (From Pedals)
		}
	}
	else
	{
		desired_torque = torque_pedal;
	}

	return(desired_torque);
}

