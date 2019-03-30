
#ifndef BMS_H_
#define BMS_H_

#include "stm32f4xx_hal.h"
#include "cmsis_os.h"

#define ID_BMS_AVGCELLVOLTAGE   0x401
#define ID_BMS_HIGHCELLVOLTAGE  0x402
#define ID_BMS_LOWCELLVOLTAGE   0x403
#define ID_BMS_AVGTemperature    0x404
#define ID_BMS_HIGHTemperature   0x405
#define ID_BMS_LOWTemperature    0x406
#define ID_BMS_PACKCURRENT      0x407
#define ID_BMS_PACKINSTVOLTAGE  0x408


#define AVG_CELLVOLTAGE_BITS_0_7    0
#define AVG_CELLVOLTAGE_BITS_11_8   1
#define HIGH_CELLVOLTAGE_BITS_0_7   0
#define HIGH_CELLVOLTAGE_BITS_11_8  1
#define LOW_CELLVOLTAGE_BITS_0_7    0
#define LOW_CELLVOLTAGE_BITS_11_8   1
#define AVG_Temperature_BITS_0_7    0
#define AVG_Temperature_BITS_11_8   1
#define HIGH_Temperature_BITS_0_7   0
#define HIGH_Temperature_BITS_11_8  1
#define LOW_Temperature_BITS_0_7    0 
#define LOW_Temperature_BITS_11_8   1
#define PACK_CURRENT_BITS_0_7       0
#define PACK_CURRENT_BITS_11_8      1
#define PACK_INST_VOLTAGE_BITS_0_7  0
#define PACK_INST_VOTLAGE_BITS_11_8 1

typedef struct {
	SemaphoreHandle_t bms_params; 	//protects these with it's life
	uint16_t pack_current; 			//Most recent pack current from the BMS
	uint16_t pack_volt;				//Most recent pack voltage
	uint8_t pack_soc;				//pack SOC
	uint8_t  high_temp;			//the current highest temperature of a cell
	uint16_t low_cell_volt;			//the lowest cell voltage
	uint8_t battery_violation;		//flag that tells what limit was broken 1 -> Power, 2 -> Temp, 3 -> Volt
}bms_data_t;

int init_bms_struct(void);

extern bms_data_t bms;

#endif /* BMS_H_ */
