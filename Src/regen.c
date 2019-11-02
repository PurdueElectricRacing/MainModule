#include "regen.h"
#include <math.h>

// @author: Chris Fallon
// @brief: calculate regen torque based on brake pressure
int16_t brake_pres_regen_torque(float brake_pres, uint16_t soc)
{
  if (soc > REGEN_CUTOFF)
  {
    return 0;
  }
  
	double current = brake_pres * (float) MAX_REGEN_CURRENT;
	return calc_regen_torque(current);
}

// @author: Chris Fallon
// @brief: calculate regen torque based on throttle position
int16_t throttle_pos_regen_torque(float throttle_pos, uint16_t soc)
{
  if (soc > REGEN_CUTOFF)
  {
    return 0;
  }
	double current = 0;
	double prcnt = 1 - throttle_pos;
	current = (prcnt) * MAX_REGEN_CURRENT;
	return calc_regen_torque(current);
}


// @author: Chris Fallon
// @brief: generic function to calculate regen torque based on
//         desired current
// @return: torque value as signed int
//      torque = -5E-07(current)^3 + 7E-05(current)^2 + 0.5149(current) - 0.7709
int16_t calc_regen_torque(double current)
{
	int16_t torque = 0;
	double temp_torque = -5.0E-7 * pow(current, 3);
	temp_torque += 7.0E-5 * pow(current, 2);
	temp_torque += 0.5149 * current;
	temp_torque += -0.7709;

	torque = (int16_t) -1 * (temp_torque * 10);
	return torque;
}

