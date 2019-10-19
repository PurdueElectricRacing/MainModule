/*
 * BMS.c
 *
 *  Created on: Nov 10, 2018
 *      Author: Matt Flanagan
 */
#include "BMS.h"
#include "CAN_Bus.h"


//init the semaphore and set all of the values to a default intial value
int init_bms_struct(BMS_t * bms)
{
  bms->high_temp = 0;
  bms->low_cell_volt = 4.2; //fully charged 18650
  bms->pack_current  = 0;
  bms->pack_volt     = PACK_FULL_VOLTAGE; //full charge
  bms->pack_soc      = 200; //full charge
  bms->fault         = NO_FAULT; //no errors
	return 0;
}


//called from rx_process frame and updates the variables used for power limiting
int process_bms_frame(uint8_t* Data, BMS_t * bms)
{
	//process the bms can frame
	//take the BMS semaphore
  bms->pack_current = (Data[0] << 8) | Data[1];
  bms->pack_volt = (Data[2] << 8) | Data[3];
  bms->pack_soc  = Data[4];
  bms->high_temp = Data[5];
  bms->low_cell_volt = (Data[6] << 8) | Data[7];
	return 0;
}

