#ifndef REGEN_H_
#define REGEN_H_
#include "stm32f4xx_hal.h"
#include "main.h"

#define MAX_REGEN_CURRENT 40.0f
#define REGEN_CUTOFF  90


int16_t throttle_pos_regen_torque(float throttle_pos, uint16_t soc);
int16_t brake_pres_regen_torque(float brake_pres, uint16_t soc);
int16_t calc_regen_torque(double current);

#endif