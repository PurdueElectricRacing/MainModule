#ifndef IMU_H_
#define IMU_H_

#include "CAN_Bus.h"
//IMU defines
#define IMU_16G											4
#define IMU_ACCEL										0
#define IMU_8G_VAL									0x3FFF
#define IMU_8G_NEG									0xC001 //negative 0x3FFF

// I am fully aware of how dumb this is.
// Carry on.
typedef enum
{
  REKT = 1,
  NOT_REKT = 0
} rekt_t;

rekt_t process_IMU(CanRxMsgTypeDef* rx);


#endif