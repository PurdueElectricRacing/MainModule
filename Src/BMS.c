/*
 * BMS.c
 *
 *  Created on: Nov 10, 2018
 *      Author: Matt Flanagan
 */
#include "BMS.h"

/*
typedef struct {
	SemaphoreHandle_t bms_params; 	//protects these with it's life
	float_t pack_current; 			//Most recent pack current from the BMS
	float_t pack_volt;				//Most recent pack voltage
	uint8_t pack_soc;				//pack SOC
	uint8_t  high_temp;			//the current highest temperature of a cell
	float_t low_cell_volt;			//the lowest cell voltage
}bms_data_t;
*/

//init the semaphore and set all of the values to a default intial value
int init_bms_struct() {
	bms.bms_params = xSemaphoreCreateBinary();
	if (xSemaphoreTake(bms.bms_params, 10) == pdTRUE) {
		bms.high_temp = 0;
		bms.low_cell_volt = 4.2; //fully charged 18650
		bms.pack_current = 0;
		bms.pack_volt = 300; //full charge
		bms.pack_soc = 200; //full charge
		bms.battery_violation = 0; //no errors
		xSemaphoreGive(bms.bms_params);
	} else {
		return HAL_ERROR;
	}
	return 0;
}

//called from rx_process frame and updates the variables used for power limiting
int process_bms_frame(CanRxMsgTypeDef* rx) {
	//process the bms can frame
	//take the BMS semaphore
	if (xSemaphoreTake(bms.bms_params, 10) == pdTRUE) {
		bms.pack_current = (rx->Data[0] << 8) | rx->Data[1];
		bms.pack_volt = (rx->Data[2] << 8) | rx->Data[3];
		bms.pack_soc = rx->Data[4];
		bms.high_temp = rx->Data[5];
		bms.low_cell_volt = (rx->Data[6] << 8) | rx->Data[7];
		xSemaphoreGive(bms.bms_params);
	} else {
		return HAL_ERROR;
	}
	return 0;
}
