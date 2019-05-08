/*
 * CANRXProcess.h
 *
 *  Created on: Dec 22, 2016
 *      Author: ben
 */

#ifndef CANPROCESS_H_
#define CANPROCESS_H_

//includes
#include "motor_controller_functions.h"
#include "car.h"
#include "PedalBox.h"
//#include "WheelModule.h"

//defines for reading data from RxCanMsgTypeDef
#define ID_PEDALBOX1              0x500
#define ID_PEDALBOX2              0x501
#define ID_PEDALBOXCALIBRATE      0x503
#define ID_RINEHART_STATION_TX    0x0C0 //message recieved by MC

//#define ID_BMS_PACK_VOLTAGE           0x400
#define ID_WHEEL_FRONT						0x700
#define ID_WHEEL_REAR							0x701
#define ID_DASHBOARD              0x350
#define ID_DASHBOARD1             0x351
#define ID_DASHBOARD2             0x352
#define ID_DASHBOARD_ACK					0x360
#define ID_BMS_PACK_CUR_VOL       0x03B //current and voltage
#define ID_BMS_DCL                0x03C
#define ID_PEDALBOX_ERRORS        0x601

//wheel module defines
#define WM_SPEED_7_0_BYTE           2
#define WM_SPEED_11_8_BYTE          1
#define WM_SPEED_11_8_MASK          0x0F00

//IMU defines
#define IMU_16G											4
#define IMU_ACCEL										0
#define IMU_8G_VAL									0x3FFF
#define IMU_8G_NEG									0xC001 //negative 0x3FFF

//pedalbox defines //todo not sure if better to send whole frame or just pbmsg.
#define PEDALBOX1_FILTER                0 //filter number corresponding to the PEDALBOX1 message
#define PEDALBOX1_THROT1_7_0_BYTE       1
#define PEDALBOX1_THROT1_7_0_OFFSET     0
#define PEDALBOX1_THROT1_7_0_MASK       0b11111111
#define PEDALBOX1_THROT1_11_8_BYTE      0
#define PEDALBOX1_THROT1_11_8_OFFSET    0
#define PEDALBOX1_THROT1_11_8_MASK      0b00001111
#define PEDALBOX1_THROT2_7_0_BYTE       3
#define PEDALBOX1_THROT2_7_0_OFFSET     0
#define PEDALBOX1_THROT2_7_0_MASK       0b11111111
#define PEDALBOX1_THROT2_11_8_BYTE      2
#define PEDALBOX1_THROT2_11_8_OFFSET    0
#define PEDALBOX1_THROT2_11_8_MASK      0b00001111
//brake
#define PEDALBOX1_BRAKE1_7_0_BYTE       5
#define PEDALBOX1_BRAKE1_7_0_OFFSET     0
#define PEDALBOX1_BRAKE1_7_0_MASK       0b11111111
#define PEDALBOX1_BRAKE1_11_8_BYTE      4
#define PEDALBOX1_BRAKE1_11_8_OFFSET    0
#define PEDALBOX1_BRAKE1_11_8_MASK      0b00001111
#define PEDALBOX1_BRAKE2_7_0_BYTE       7
#define PEDALBOX1_BRAKE2_7_0_OFFSET     0
#define PEDALBOX1_BRAKE2_7_0_MASK       0b11111111
#define PEDALBOX1_BRAKE2_11_8_BYTE      6
#define PEDALBOX1_BRAKE2_11_8_OFFSET    0
#define PEDALBOX1_BRAKE2_11_8_MASK      0b00001111

#define PEDALBOX1_EOR_BYTE            3
#define PEDALBOX1_EOR_OFFSET          0
#define PEDALBOX1_EOR_MASK            0b00000001
#define PEDALBOX1_IMP_BYTE            3
#define PEDALBOX1_IMP_OFFSET          1
#define PEDALBOX1_IMP_MASK            0b00000010

//Power Limiting
#define ID_POWER_LIMIT                0x352
#define ID_BMS_MACRO                  0x6B1
//Wheel Speed Defines
//Front Left
#define WHEEL_FL_7_0_BYTE 3
#define WHEEL_FL_15_8_BYTE 2
#define WHEEL_FL_23_16_BYTE 1
#define WHEEL_FL_31_24_BYTE 0
//Front Right
#define WHEEL_FR_7_0_BYTE 7
#define WHEEL_FR_15_8_BYTE 6
#define WHEEL_FR_23_16_BYTE 5
#define WHEEL_FR_31_24_BYTE 4
//Rear Left
#define WHEEL_RL_7_0_BYTE 3
#define WHEEL_RL_15_8_BYTE 2
#define WHEEL_RL_23_16_BYTE 1
#define WHEEL_RL_31_24_BYTE 0
//Rear Right
#define WHEEL_RR_7_0_BYTE 7
#define WHEEL_RR_15_8_BYTE 6
#define WHEEL_RR_23_16_BYTE 5
#define WHEEL_RR_31_24_BYTE 4




/**
  * @brief  CAN Tx message structure definition
  */
typedef struct {
  uint32_t StdId;    /*!< Specifies the standard identifier.
                          This parameter must be a number between Min_Data = 0 and Max_Data = 0x7FF */

  uint32_t ExtId;    /*!< Specifies the extended identifier.
                          This parameter must be a number between Min_Data = 0 and Max_Data = 0x1FFFFFFF */

  uint32_t IDE;      /*!< Specifies the type of identifier for the message that will be transmitted.
                          This parameter can be a value of @ref CAN_Identifier_Type */

  uint32_t RTR;      /*!< Specifies the type of frame for the message that will be transmitted.
                          This parameter can be a value of @ref CAN_remote_transmission_request */

  uint32_t DLC;      /*!< Specifies the length of the frame that will be transmitted.
                          This parameter must be a number between Min_Data = 0 and Max_Data = 8 */

  uint8_t Data[8];   /*!< Contains the data to be transmitted.
                          This parameter must be a number between Min_Data = 0 and Max_Data = 0xFF */

} CanTxMsgTypeDef;

/**
  * @brief  CAN Rx message structure definition
  */
typedef struct {
  uint32_t StdId;       /*!< Specifies the standard identifier.
                             This parameter must be a number between Min_Data = 0 and Max_Data = 0x7FF */

  uint32_t ExtId;       /*!< Specifies the extended identifier.
                             This parameter must be a number between Min_Data = 0 and Max_Data = 0x1FFFFFFF */

  uint32_t IDE;         /*!< Specifies the type of identifier for the message that will be received.
                             This parameter can be a value of @ref CAN_Identifier_Type */

  uint32_t RTR;         /*!< Specifies the type of frame for the received message.
                             This parameter can be a value of @ref CAN_remote_transmission_request */

  uint32_t DLC;         /*!< Specifies the length of the frame that will be received.
                             This parameter must be a number between Min_Data = 0 and Max_Data = 8 */

  uint8_t Data[8];      /*!< Contains the data to be received.
                             This parameter must be a number between Min_Data = 0 and Max_Data = 0xFF */

  uint32_t FMI;         /*!< Specifies the index of the filter the message stored in the mailbox passes through.
                             This parameter must be a number between Min_Data = 0 and Max_Data = 0xFF */

  uint32_t FIFONumber;  /*!< Specifies the receive FIFO number.
                             This parameter can be CAN_FIFO0 or CAN_FIFO1 */

} CanRxMsgTypeDef;

void ISR_RXCAN();
void DCANFilterConfig();
void VCANFilterConfig();
void taskRXCANProcess();
void taskTX_DCAN();
void taskTX_VCAN();
void taskRXCAN();
void processBamoCar(CanRxMsgTypeDef* rx);
void processWheelModuleFrame(CanRxMsgTypeDef* rx);
void processPedalboxFrame(CanRxMsgTypeDef* rx);
void process_IMU(CanRxMsgTypeDef* rx);

void processCalibrate(CanRxMsgTypeDef* rx);
void processCalibratePowerLimit(CanRxMsgTypeDef* rx);
int process_bms_frame(CanRxMsgTypeDef* rx);

void send_ack(uint16_t can_id, uint16_t response);

#endif /* CANPROCESS_H_ */
