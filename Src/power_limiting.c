/*
 * power_limiting.c
 *
 *  Created on: Nov 9, 2018
 *      Author: Matt Flanagan
 */
#include "power_limiting.h"

void init_pow_lim() {
	car.pow_lim.power_hard_lim = 80000; //rule level
	car.pow_lim.power_soft_lim = (80000 * 95) / 100; //95%
	car.pow_lim.power_thresh = (80000 * 90) / 100; //90%
	car.pow_lim.power_lim_en = ASSERTED;
}

int16_t limit_torque(int16_t torque_req) {
	int torque_limited = torque_req;
	int16_t watt_gain = 100; //100 means gain of 1.
	int16_t temp_gain = 100;
	int16_t volt_gain = 100;

	watt_gain = power_limit_watt();
	temp_gain = power_limit_temp();
	volt_gain = power_limit_volt();

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

	return (int16_t) torque_limited;
}

int power_limit_volt() {
  uint8_t gain = 0;
  float multiplier = 0;
  //only throttle if past the threshold
  if (car.bms_params.low_cell_volt > VOLT_THRESH) return 100;
  //if past the hard lim stop the driving
  if (car.bms_params.low_cell_volt < VOLT_HARD_LIM) {
    car.bms_params.battery_violation = 0;
    return 0;
  }

  //between thresh hold and hard limit
  if (car.bms_params.low_cell_volt > VOLT_SOFT_LIM) {
      //between threshold and soft limit
      //have linear decrease from 100% -> 50%
    multiplier = ((float) (VOLT_THRESH - car.bms_params.low_cell_volt) / (VOLT_THRESH - VOLT_SOFT_LIM));
      gain = 100 - 50 * (multiplier);
  } else {
    //between soft and hard lim
    //linear decrease from 50% -> 0%
    multiplier = ((float) (VOLT_SOFT_LIM - car.bms_params.low_cell_volt) / (VOLT_SOFT_LIM - VOLT_HARD_LIM));
      gain = 50 - 50 * (multiplier);
  }

  return gain;
}

int power_limit_temp() {
  uint8_t gain = 0;
  float multiplier = 0;
  //only throttle if past the threshold
  if (car.bms_params.high_temp < TEMP_THRESH) return 100;
  //if past the hard lim stop the driving
  if (car.bms_params.high_temp > TEMP_HARD_LIM) {
    car.bms_params.battery_violation = 1;
    return 0;
  }

  if (car.bms_params.high_temp < TEMP_SOFT_LIM) {
    //between threshold and soft limit
    //have linear decrease from 100% -> 50%
    multiplier = ((float) (car.bms_params.high_temp - TEMP_THRESH) / (TEMP_SOFT_LIM - TEMP_THRESH));
      gain = 100 - 50 * (multiplier);
  } else {
    //between soft and hard lim
    //linear decrease from 50% -> 0%
    multiplier = ((float) (car.bms_params.high_temp - TEMP_SOFT_LIM) / (TEMP_HARD_LIM - TEMP_SOFT_LIM));
      gain = 50 - 50 * (multiplier);
  }

  return gain;
}

int power_limit_watt() {
  uint8_t gain = 0;
  float multiplier = 0;
  int power_actual = 0;
  //only throttle if past the threshold
  power_actual = car.bms_params.pack_current * car.bms_params.pack_volt; //calculate the instantaneous power draw

  if (power_actual < car.pow_lim.power_thresh) return 100;
  //if past the hard lim stop the driving
  if (power_actual > 80000) {
    car.bms_params.battery_violation = 2;
    return 0;
  }

  if (power_actual < car.pow_lim.power_soft_lim) {
    //between threshold and soft limit
    //have linear decrease from 100% -> 50%
    multiplier = ((float) (power_actual - car.pow_lim.power_thresh) / (car.pow_lim.power_soft_lim - car.pow_lim.power_thresh));
    gain = 100 - 50 * (multiplier);
  } else {
    //between soft and hard lim
    //linear decrease from 50% -> 0%
    multiplier = ((float) (power_actual - car.pow_lim.power_soft_lim) / (car.pow_lim.power_hard_lim - car.pow_lim.power_soft_lim));
      gain = 50 - 50 * (multiplier);
  }

  return gain;
}
