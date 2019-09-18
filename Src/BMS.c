/*
 * BMS.c
 *
 *  Created on: Nov 10, 2018
 *      Author: Matt Flanagan
 */
#include "BMS.h"

//init the semaphore and set all of the values to a default intial value
int init_bms_struct() {
  car.bms_params.high_temp = 0;
  car.bms_params.low_cell_volt = 4.2; //fully charged 18650
  car.bms_params.pack_current = 0;
  car.bms_params.pack_volt = 300; //full charge
  car.bms_params.pack_soc = 200; //full charge
  car.bms_params.battery_violation = 0; //no errors
	return 0;
}


