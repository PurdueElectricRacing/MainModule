/*
 * power_limiting.c
 *
 *  Created on: Nov 9, 2018
 *      Author: Matt Flanagan
 */
#include "power_limiting.h"
#include "car.h"

// TODO global BMS and power_limit pointers maybe

// private function prototypes
uint8_t power_limit_watt(volatile BMS_t * bms);
uint8_t power_limit_temp(volatile BMS_t * bms);
uint8_t power_limit_volt(volatile BMS_t * bms);

void init_power_limit(volatile power_limit_t *p_power_limit)
{
	p_power_limit->power_hard_lim = 80000; //rule level
	p_power_limit->power_soft_lim = (80000 * 95) / 100; //95%
	p_power_limit->power_thresh = (80000 * 90) / 100; //90%
	p_power_limit->enabled = false;
}

int16_t limit_torque(int16_t torque_req) {
	int torque_limited = torque_req;
	int16_t watt_gain = 100; //100 means gain of 1.
	int16_t temp_gain = 100;
	int16_t volt_gain = 100;

	watt_gain = power_limit_watt(&car.bms);
	temp_gain = power_limit_temp(&car.bms);
	volt_gain = power_limit_volt(&car.bms);

	if (watt_gain < temp_gain && watt_gain < volt_gain)
	{
		//power limited by rules
		torque_limited = (torque_limited * watt_gain) / 100;
	}
	else if (temp_gain < watt_gain && temp_gain < volt_gain)
	{
		//power limited by temp
		torque_limited = (torque_limited * temp_gain) / 100;
	}
	else
	{
		//power limited by voltage
		torque_limited = (torque_limited * volt_gain) / 100;
	}

	return (int16_t) torque_limited;
}

uint8_t power_limit_volt(volatile BMS_t * bms) {
  uint8_t gain = 0;
  float multiplier = 0;
  //only throttle if past the threshold
  if (car.bms.low_cell_volt > VOLT_THRESH) return 100;
  //if past the hard lim stop the driving
  if (bms->low_cell_volt < VOLT_HARD_LIM)
  {
    bms->fault = OVER_VOLT_FAULT;
    return 0;
  }

  //between thresh hold and hard limit
  if (bms->low_cell_volt > VOLT_SOFT_LIM)
  {
      //between threshold and soft limit
      //have linear decrease from 100% -> 50%
    multiplier = ((float) (VOLT_THRESH - bms->low_cell_volt) / (VOLT_THRESH - VOLT_SOFT_LIM));
      gain = 100 - 50 * (multiplier);
  }
  else
  {
    //between soft and hard lim
    //linear decrease from 50% -> 0%
    multiplier = ((float) (VOLT_SOFT_LIM - bms->low_cell_volt) / (VOLT_SOFT_LIM - VOLT_HARD_LIM));
      gain = 50 - 50 * (multiplier);
  }

  return gain;
}

uint8_t power_limit_temp(volatile BMS_t * bms) {
  uint8_t gain = 0;
  float multiplier = 0;
  //only throttle if past the threshold
  if (bms->high_temp < TEMP_THRESH) return 100;
  //if past the hard lim stop the driving
  if (bms->high_temp > TEMP_HARD_LIM)
  {
    bms->fault = OVER_TEMP_FAULT;
    return 0;
  }

  if (bms->high_temp < TEMP_SOFT_LIM)
  {
    //between threshold and soft limit
    //have linear decrease from 100% -> 50%
    multiplier = ((float) (bms->high_temp - TEMP_THRESH) / (TEMP_SOFT_LIM - TEMP_THRESH));
		gain = 100 - 50 * (multiplier);
  }
  else
  {
    //between soft and hard lim
    //linear decrease from 50% -> 0%
    multiplier = ((float) (bms->high_temp - TEMP_SOFT_LIM) / (TEMP_HARD_LIM - TEMP_SOFT_LIM));
    gain = 50 - 50 * (multiplier);
  }

  return gain;
}

uint8_t power_limit_watt(volatile BMS_t * bms) {
  uint8_t gain = 0;
  float multiplier = 0;
  int power_actual = 0;
  //only throttle if past the threshold
  power_actual = bms->pack_current * bms->pack_volt; //calculate the instantaneous power draw

  if (power_actual < car.power_limit.power_thresh) return 100;

  //if past the hard lim stop the driving
  if (power_actual > 80000)
  {
    bms->fault = OVER_POWER_FAULT;
    return 0;
  }

  if (power_actual < car.power_limit.power_soft_lim)
  {
    //between threshold and soft limit
    //have linear decrease from 100% -> 50%
    multiplier = ((float) (power_actual - car.power_limit.power_thresh) / (car.power_limit.power_soft_lim - car.power_limit.power_thresh));
    gain = 100 - 50 * (multiplier);
  }
  else
  {
    //between soft and hard lim
    //linear decrease from 50% -> 0%
    multiplier = ((float) (power_actual - car.power_limit.power_soft_lim) / (car.power_limit.power_hard_lim - car.power_limit.power_soft_lim));
      gain = 50 - 50 * (multiplier);
  }

  return gain;
}
