#include "wheel_mod.h"


void init_wheel_mod(volatile wheel_module_t * mod)
{
  mod->FL_rpm = 0.0;
  mod->FR_rpm = 0.0;
  mod->RL_rpm = 0.0;
  mod->RR_rpm = 0.0;
}

// @authors: Chris Fallon
// @brief: Calculate the wheel speed for the given ID
void calc_wheel_speed(volatile wheel_module_t * mod, uint32_t id, uint8_t * data)
{
	volatile float *left;
	volatile float *right;
	uint32_t left_raw;
	uint32_t right_raw;

  if (id == ID_WHEEL_FRONT)
  {
  	left = &mod->FL_rpm;
  	right = &mod->FR_rpm;
  }
  else
  {
  	left = &mod->RL_rpm;
  	right = &mod->RR_rpm;
  }

  left_raw = ((uint32_t) data[0]) << 24 | ((uint32_t) data[1] << 16) | ((uint32_t) data[2] << 8) | data[3];

  right_raw = ((uint32_t) data[4]) << 24 | ((uint32_t) data[5] << 16) | ((uint32_t) data[6] << 8) | data[7];

  // 10000 is the scalar from DAQ
  *left = left_raw / WHEEL_SPEED_SCALAR;
  *right = right_raw / WHEEL_SPEED_SCALAR;
}
