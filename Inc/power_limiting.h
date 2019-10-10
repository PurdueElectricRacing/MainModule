/*
 * power_limiting.h
 *
 *  Created on: Nov 9, 2018
 *      Author: Matt Flanagan
 */

#ifndef POWER_LIMITING_H_
#define POWER_LIMITING_H_


//#include "car.h"
#include "stm32f4xx_hal.h"
#include "cmsis_os.h"
#include <stdbool.h>
#include "BMS.h"

#define VOLT_THRESH		32000 //.0001 volts
#define VOLT_SOFT_LIM	30000
#define VOLT_HARD_LIM	28000

#define TEMP_THRESH		50 //when throttling takes place
#define TEMP_SOFT_LIM	55
#define TEMP_HARD_LIM	58

typedef struct
{
  int power_thresh;
  int power_soft_lim;
  int power_hard_lim;
  bool enabled;
} power_limit_t;


int16_t limit_torque(int16_t torque_req, volatile BMS_t * bms);
void init_power_limit(volatile power_limit_t * p_pow_lim, bool enable);
void processCalibratePowerLimit(uint8_t * Data, volatile power_limit_t * power_limit);

#endif /* POWER_LIMITING_H_ */
