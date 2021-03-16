#ifndef EMDRIVE_H
#define EMDRIVE_H

#include "main.h"
#include "cmsis_os.h"
#include "stm32f4xx_hal.h"
#include <stdbool.h>
#include "imu.h"

#include "CAN_Bus.h"

#define NODE_ID 0x001

typedef enum 
{
  OK = 0,
  GENERIC_ERR     = 0x1000,
  OVERCURRENT_ERR = 0x2220,
  DC_LINK_OVER_VOLT_ERR = 0x3210,
  PHASE_A_CURRENT_MEAS_ERR = 0xFF01,
  PHASE_B_CURRENT_MEAS_ERR = 0xFF02 ,
  HIGH_SIDE_FET_SHORT_ERR = 0xFF03,
  LOW_SIDE_FET_SHORT_ERR = 0xFF05,
  LOW_SIDE_FET_PHS1_SHORT_ERR = 0xFF06,
  LOW_SIDE_FET_PHS2_SHORT_ERR = 0xFF04,
  LOW_SIDE_FET_PHS3_SHORT_ERR = 0xFF07,
  HIGH_SIDE_FET_PHS1_SHORT_ERR = 0xFF08,
  HIGH_SIDE_FET_PHS2_SHORT_ERR = 0xFF09,
  HIGH_SIDE_FET_PHS3_SHORT_ERR = 0xFF0A,
  MOTOR_FEEDBACK_ERR = 0xFF0B,
  DC_LINK_UNDER_VOLT_ERR = 0xFF0C ,
  PULS_MODE_FINISHED_ERR = 0xFF0D,
  EMRGNCY_BUTT_PRSD_ERR = 0xFF0E,
} emdrive_err_t;

typedef enum
{
  EMDRIVE_START  = 0x01,
  EMDRIVE_STOP   = 0x02,
  EMDRIVE_PRE_OP = 0x80,
  EMDRIVE_RESET  = 0x81,
  EMDRIVE_RESET_COMM = 0x82,
} emdrive_nmt_command_t;

typedef enum 
{
  READY_TO_SWITCH_ON    = 0,
  SWITCHED_ON           = 1,
  OPERATION_ENBL        = 2,
  FAULT                 = 3,
  VOLTAGE_ENBL          = 4,
  QUICK_STOP            = 5,
  SWITCH_ON_DSBL        = 6,
  WARNING               = 7,
  REMOTE                = 8,
  TARGET_REACHED        = 10,
  INTERNAL_LIMIT_ACTIVE = 11,
} status_word_masks_t;

typedef enum
{
    BYTES_4 = 0x23,
    BYTES_3 = 0x27,
    BYTES_2 = 0x2B,
    BYTES_1 = 0x2F,
} emdrive_sdo_size_t;

enum
{
    CONTROLWORD = 0x6040,
    TORQUE = 0x6071,
} emdrive_dict_obj;

typedef struct
{
  int32_t position_actual;
  int32_t electrical_power;
  int32_t velocity;

  uint16_t status_word;
  uint16_t error_codes;

  int16_t torque_actual;
  int16_t phase_b_current;
  
  uint8_t emdrive_temp;
  uint8_t motor_temp;
  uint8_t actual_current;
} emdrive_tpdo_t;

typedef enum
{
  PRE_OPERATION = 0,
  OPERATION = 1,
  CW6,
  DRIVE
} emdrive_state_t;

typedef struct 
{
  bool warning;
  emdrive_state_t state;
  emdrive_err_t err;
  emdrive_tpdo_t data;
} emdrive_t;

void emdrive_sync(CAN_Bus_TypeDef * can);
void emdrive_init(emdrive_t * drive);
// only PDO 1 matters, since we aren't setting position values.
void emdrive_control(emdrive_nmt_command_t action, emdrive_t * drive, CAN_Bus_TypeDef * can);
void emdrive_parse_pdo(CAN_IDs_t id, uint8_t * data, emdrive_t * drive);
void emdrive_move_the_car_yo(int16_t torque, CAN_Bus_TypeDef * can);
emdrive_err_t emdrive_check_statusword(emdrive_t * drive);

#endif
