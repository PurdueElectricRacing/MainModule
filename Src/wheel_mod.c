#include "wheel_mod.h"
#include "cmsis_os.h"


void init_wheel_mod(wheel_module_t * mod)
{
  mod->FL_rpm = 0;
  mod->FR_rpm = 0;
  mod->RL_rpm = 0;
  mod->RR_rpm = 0;
}

// @authors: Chris Fallon
// @brief: Calculate the wheel speed for the given ID
void calc_wheel_speed(wheel_module_t * mod, uint32_t id, uint8_t * data)
{

	uint32_t *left;
	uint32_t *right;

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
  if ((xTaskGetTickCount() - (mod->last_rx)) > WHEEL_SPEED_TIMEOUT)
  {
  	*left = 0;
  	*right = 0;
  }

  *left = ((uint32_t) data[0]) << 24 | ((uint32_t) data[1] << 16) | ((uint32_t) data[2] << 8) | data[3];

  *right = ((uint32_t) data[4]) << 24 | ((uint32_t) data[5] << 16) | ((uint32_t) data[6] << 8) | data[7];

}
