
#ifndef BMS_H_
#define BMS_H_

#include "stm32f4xx_hal.h"
#include "cmsis_os.h"


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

typedef enum 
{
  // TODO possibly make this be values ORed together
  OVER_POWER_FAULT = 0,
  OVER_TEMP_FAULT  = 1,
  OVER_VOLT_FAULT  = 2,
} BMS_Fault_t;


typedef struct
{
  BMS_Fault_t  fault;    //flag that tells what limit was broken 1 -> Power, 2 -> Temp, 3 -> Volt
  uint8_t  pack_soc;        //pack SOC
  uint8_t  high_temp;       //the current highest temperature of a cell
  uint16_t pack_current;    //Most recent pack current from the BMS
  uint16_t pack_volt;       //Most recent pack voltage
  uint16_t low_cell_volt;   //the lowest cell voltage
} BMS_t;

int init_bms_struct(volatile BMS_t * bms);
int process_bms_frame(uint8_t* Data, volatile BMS_t * bms);

#endif /* BMS_H_ */
