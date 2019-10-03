/*
 * BMS.c
 *
 *  Created on: Nov 10, 2018
 *      Author: Matt Flanagan
 */
#include "BMS.h"
#include "car.h"
#include "CAN_Bus.h"


//init the semaphore and set all of the values to a default intial value
int init_bms_struct(volatile BMS_t * bms)
{
  bms->high_temp = 0;
  bms->low_cell_volt = 4.2; //fully charged 18650
  bms->pack_current  = 0;
  bms->pack_volt     = 300; //full charge
  bms->pack_soc      = 200; //full charge
  bms->fault         = 0; //no errors
	return 0;
}


