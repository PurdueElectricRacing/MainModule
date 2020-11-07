#include "imu.h"


/***************************************************************************
*
*     Function Information
*
*     Name of Function: process_IMU
*
*     Programmer's Name: Matt Flanagan, matthewdavidflanagan@outlook.com
*
*     Function Return Type: none
*
*     Parameters (list data type, name, and comment one per line):
*			1. CanRxMsgTypeDef* rx, CAN frame of IMU
*      Global Dependents:
*	    1. None
*
*     Function Description:
*     	Looks IMU data and see's if any axis is over 8G (rules for inertia switch). If so then
*     	fault the SDC. ASSUMES THAT THE IMU DATA COMING IS 16G Resolution
*
***************************************************************************/
 rekt_t process_IMU(CanRxMsgTypeDef* rx) {
	int16_t accel_x = 0;
	int16_t accel_y = 0;
	int16_t accel_z = 0;

	if (rx->Data[7] == IMU_16G && rx->Data[0] == IMU_ACCEL) {
		//IMU is in 16g resolution and is an acceleration frame
		accel_x = (int16_t) (rx->Data[1] << 8) | rx->Data[2];
		accel_y = (int16_t) (rx->Data[3] << 8) | rx->Data[4];
		accel_z = (int16_t) (rx->Data[5] << 8) | rx->Data[6];

		if ((accel_x > IMU_8G_VAL/4 || accel_y > IMU_8G_VAL || accel_z > IMU_8G_VAL)
				&& (accel_x < IMU_8G_NEG || accel_y < IMU_8G_NEG || accel_z < IMU_8G_NEG)) {
			//car just got straight fucked
			//open the SDC
      return REKT;
		}
	}
  return NOT_REKT;
}