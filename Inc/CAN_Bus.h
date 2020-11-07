#ifndef CAN_BUS_H
#define CAN_BUS_H

#include "stm32f4xx_hal.h"
#include "cmsis_os.h"
#include "main.h"

#define BEGIN_DATA_BYTE(x) (x * sizeof(uint8_t *)) // macro for returning the offset of a can data array

typedef enum
{
	ID_RINEHART_STATION_TX = 0x069,
	ID_RINEHART_PARAM_CMD = 0x005,
  ID_BMS_PACK_CUR_VOL  = 0x03B,

  ID_EMDRIVE_INIT = 0x700,
  ID_EMDRIVE_MASTER_SYNC = 0x080,
  ID_EMDRIVE_SLAVE_PDO_1 = 0x180,
  ID_EMDRIVE_SLAVE_PDO_2 = 0x280,
  ID_EMDRIVE_SLAVE_PDO_3 = 0x380,
  ID_EMDRIVE_SLAVE_PDO_4 = 0x480,
  ID_EMDRIVE_NMT_CONTROL = 0x000,
  ID_EMDRIVE_MASTER_PDO_1 = 0x200,


  ID_BMS_DCL     = 0x03C,
  ID_MAIN        = 0x420,
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

  ID_PEDALBOX = 0x501,
  ID_PEDALBOXCALIBRATE = 0x503,
  ID_PEDALBOX_ERRORS   = 0x601,
	ID_BMS = 0x6B1,
  ID_WHEEL_FRONT = 0x40E,
  ID_WHEEL_REAR  = 0x40F,
  ID_F_TIRE_TEMP = 0x410,
  ID_R_TIRE_TEMP = 0x411,
  ID_R_COOLANT   = 0x412,
  ID_R_COOLANT_SPEED = 0x413,
  ID_F_SHOCKS = 0x414,
  ID_R_SHOCKS = 0x415,
  ID_F_DROP_LINKS = 0x416,
  ID_F_STEER_PUSH = 0x417,
  ID_R_TIE_PUSH   = 0x418,
  ID_F_LCA = 0x419,
  ID_R_LCA = 0x41A,
  ID_F_UCA = 0x41B,
  ID_R_UCA = 0x41C,
  ID_F_ARB = 0x41D,
  ID_R_ARB = 0x41E,
  ID_F_IMU = 0x421,
  ID_ENABLE_DAQ  = 0x425,
  ID_FRONT_ERROR = 0x423,
  ID_REAR_ERROR  = 0x424,
  
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


uint16_t parse_from_big_16(uint8_t * data);
uint16_t parse_from_lil_16(uint8_t * data);
uint32_t parse_from_big_32(uint8_t * data);
uint32_t parse_from_lil_32(uint8_t * data);




HAL_StatusTypeDef broadcast_can_msg(CanTxMsgTypeDef * tx, CAN_HandleTypeDef * can);
#endif
