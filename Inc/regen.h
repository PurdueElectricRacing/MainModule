#ifndef REGEN_H_
#define REGEN_H_
#include "stm32f4xx_hal.h"
#include "main.h"

#define MAX_REGEN_CURRENT 40.0f


int16_t throttle_pos_regen_torque(float throttle_pos);
int16_t brake_pres_regen_torque(float brake_pres);
int16_t calc_regen_torque(double current);

#endif
