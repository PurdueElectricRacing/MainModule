#ifndef WHEEL_MOD_H_
#define WHEEL_MOD_H_

#include "stm32f4xx_hal.h"
#include "CAN_Bus.h"

#define WHEEL_SPEED_TIMEOUT 200
typedef struct
{
	float FL_rpm;
	float FR_rpm;

	float RR_rpm;
	float RL_rpm;
  uint32_t last_rx;
} wheel_module_t;

void init_wheel_mod(volatile wheel_module_t * mod);
void calc_wheel_speed(volatile wheel_module_t * mod, uint32_t id, uint8_t * data);

#endif
