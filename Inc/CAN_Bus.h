#ifndef CAN_BUS_H
#define CAN_BUS_H

#include "stm32f4xx_hal.h"
#include "cmsis_os.h"

enum
{
  ID_RINEHART_TEMP1        = 0x0A0,
  ID_RINEHART_TEMP2        = 0x0A1,
  ID_RINEHART_TEMP3        = 0x0A2,
  ID_RINEHART_AIN_VOLT     = 0x0A3,
  ID_RINEHART_DIGIN_STATUS = 0x0A4,
  ID_RINEHART_MOTOR_POS    = 0x0A5,
  ID_RINEHART_I_INFO       = 0x0A6,
  ID_RINEHART_V_INFO       = 0x0A7,
  ID_RINEHART_FLUX_INFO    = 0x0A8,
  ID_RINEHART_INTERN_VOLT  = 0X0A9,
  ID_RINEHART_INTERN_STATE = 0X0AA,
  ID_RINEHART_FAULT_CODE   = 0X0AB,
  ID_RINEHART_TORQUE_TIM   = 0X0AC,
  ID_RINEHART_MOD_FLUX     = 0X0AD,
  ID_RINEHART_FIRMWARE     = 0X0AE,
  ID_RINEHART_DIAGNOSTIC   = 0X0AF,

  ID_RINEHART_STATION_TX = 0x0C0,
  ID_RINEHART_PARAM_CMD  = 0x0C1,
  ID_RINEHART_PARAM_RESP = 0x0C2,
  ID_BMS_PACK_CUR_VOL    = 0x03B,
  
  ID_BMS_DCL     = 0x03C,
  ID_MAIN        = 0x200,
  ID_DASHBOARD   = 0x350,
  ID_DASHBOARD1  = 0x351,
  ID_POWER_LIMIT = 0x352,
  ID_DASHBOARD_ACK = 0x360,
  MAIN_ACK_ID  = 0x360,
  // BMS IDs
  ID_BMS_AVGCELLVOLTAGE  = 0x401,
  ID_BMS_HIGHCELLVOLTAGE = 0x402,
  ID_BMS_LOWCELLVOLTAGE  = 0x403,
  ID_BMS_AVGTemperature  = 0x404,
  ID_BMS_HIGHTemperature = 0x405,
  ID_BMS_LOWTemperature  = 0x406,
  ID_BMS_PACKCURRENT     = 0x407,
  ID_BMS_PACKINSTVOLTAGE = 0x408,

  ID_PEDALBOX1 = 0x500,
  ID_PEDALBOX2 = 0x501,
  ID_PEDALBOXCALIBRATE = 0x503,
  ID_PEDALBOX_ERRORS   = 0x601,
	ID_BMS = 0x6B1,
  ID_WHEEL_FRONT = 0x700,
  ID_WHEEL_REAR  = 0x701,
  ID_F_TIRE_TEMP = 0x710,
  ID_R_TIRE_TEMP = 0x711,
  ID_R_COOLANT   = 0x720,
  ID_R_COOLANT_SPEED = 0x721,
  ID_F_SHOCKS = 0x730,
  ID_R_SHOCKS = 0x731,
  ID_F_DROP_LINKS = 0x740,
  ID_F_STEER_PUSH = 0x750,
  ID_R_TIE_PUSH   = 0x760,
  ID_F_LCA = 0x770,
  ID_R_LCA = 0x780,
  ID_F_UCA = 0x790,
  ID_R_UCA = 0x7A0,
  ID_F_ARB = 0x7B0,
  ID_R_ARB = 0x7B1,
  ID_F_IMU = 0x7D0,
  ID_ENABLE_DAQ  = 0x7E0,
  ID_FRONT_ERROR = 0x7F0,
  ID_REAR_ERROR  = 0x7F1,
  
} CAN_IDs_t;

typedef enum
{
	WHEEL_SPEED_SCALAR = 10000,
}scalars_t;

typedef struct
{
  uint32_t StdId; /*!< Specifies the standard identifier.
                          This parameter must be a number between Min_Data = 0 and Max_Data = 0x7FF */

  uint32_t ExtId; /*!< Specifies the extended identifier.
                          This parameter must be a number between Min_Data = 0 and Max_Data = 0x1FFFFFFF */

  uint32_t IDE; /*!< Specifies the type of identifier for the message that will be transmitted.
                          This parameter can be a value of @ref CAN_Identifier_Type */

  uint32_t RTR; /*!< Specifies the type of frame for the message that will be transmitted.
                          This parameter can be a value of @ref CAN_remote_transmission_request */

  uint32_t DLC; /*!< Specifies the length of the frame that will be transmitted.
                          This parameter must be a number between Min_Data = 0 and Max_Data = 8 */

  uint8_t Data[8]; /*!< Contains the data to be transmitted.
                          This parameter must be a number between Min_Data = 0 and Max_Data = 0xFF */

} CanTxMsgTypeDef;

/**
  * @brief  CAN Rx message structure definition
  */
typedef struct
{
  uint32_t StdId; /*!< Specifies the standard identifier.
                             This parameter must be a number between Min_Data = 0 and Max_Data = 0x7FF */

  uint32_t ExtId; /*!< Specifies the extended identifier.
                             This parameter must be a number between Min_Data = 0 and Max_Data = 0x1FFFFFFF */

  uint32_t IDE; /*!< Specifies the type of identifier for the message that will be received.
                             This parameter can be a value of @ref CAN_Identifier_Type */

  uint32_t RTR; /*!< Specifies the type of frame for the received message.
                             This parameter can be a value of @ref CAN_remote_transmission_request */

  uint32_t DLC; /*!< Specifies the length of the frame that will be received.
                             This parameter must be a number between Min_Data = 0 and Max_Data = 8 */

  uint8_t Data[8]; /*!< Contains the data to be received.
                             This parameter must be a number between Min_Data = 0 and Max_Data = 0xFF */

  uint32_t FMI; /*!< Specifies the index of the filter the message stored in the mailbox passes through.
                             This parameter must be a number between Min_Data = 0 and Max_Data = 0xFF */

  uint32_t FIFONumber; /*!< Specifies the receive FIFO number.
                             This parameter can be CAN_FIFO0 or CAN_FIFO1 */

} CanRxMsgTypeDef;


typedef enum
{
	WHEEL_L_7_0_BYTE   = 3,
	WHEEL_L_15_8_BYTE  = 2,
	WHEEL_L_23_16_BYTE = 1,
	WHEEL_L_31_24_BYTE = 0,

	WHEEL_R_7_0_BYTE   = 7,
	WHEEL_R_15_8_BYTE  = 6,
	WHEEL_R_23_16_BYTE = 5,
	WHEEL_R_31_24_BYTE = 4,
}WHEEL_DATA_INDEXES;

typedef struct
{
  CAN_HandleTypeDef *hcan;
  QueueHandle_t q_rx;
  QueueHandle_t q_tx;
} CAN_Bus_TypeDef;

void send_ack(uint16_t can_id, uint16_t response, CAN_Bus_TypeDef * hcan);

void taskTX_CAN(void * params);
void task_RX_CAN(void * params);
void init_can_bus(CAN_Bus_TypeDef * bus, CAN_HandleTypeDef * hcan, uint16_t rx_q_size, uint16_t tx_q_size);

void DCANFilterConfig(CAN_HandleTypeDef * hcan);
void VCANFilterConfig(CAN_HandleTypeDef * hcan);

HAL_StatusTypeDef broadcast_can_msg(CanTxMsgTypeDef * tx, CAN_HandleTypeDef * can);
#endif