#ifndef WHEEL_MOD_H_
#define WHEEL_MOD_H_

#include "stm32f4xx_hal.h"
#include "CAN_Bus.h"

#define WHEEL_SPEED_TIMEOUT 200
typedef struct
{
	uint32_t FL_rpm;
	uint32_t FR_rpm;

	uint32_t RR_rpm;
	uint32_t RL_rpm;
  uint32_t last_rx;
} wheel_module_t;

void init_wheel_mod(wheel_module_t * mod);
void calc_wheel_speed(wheel_module_t * mod, uint32_t id, uint8_t * data);

#endif
