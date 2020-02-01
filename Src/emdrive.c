#include "emdrive.h"
#include <string.h>

// @brief: Init function
// @author: Chris Fallon
void emdrive_init(emdrive_t * drive)
{
  memset(&drive->data, 0, sizeof(emdrive_tpdo_t)); // init all data points to 0
  drive->state = PRE_OPERATION;
  
  drive->err = OK;

  drive->warning = false;

}

// @brief: Function for syncing the motor controller
//         intended to be sent periodically by an RTOS task,
//         but can be used by itself
// @author: Chris Fallon
void emdrive_sync(CAN_Bus_TypeDef * can)
{
  CanTxMsgTypeDef tx;
  tx.IDE = CAN_ID_STD;
  tx.RTR = CAN_RTR_DATA;
  tx.StdId = ID_EMDRIVE_MASTER_SYNC;
  tx.DLC = 1;
  tx.Data[0] = 0;
  xQueueSendToBack(can->q_tx, &tx, 100);
}



// @brief: Function for sending config parameters to the emdrive
// @author: Chris Fallon
void emdrive_control(emdrive_nmt_command_t action, emdrive_t * drive, CAN_Bus_TypeDef * can)
{
 if (drive->state == PRE_OPERATION && action == EMDRIVE_STOP)
 {
   return;
 }
 else if (drive->state == PRE_OPERATION && action == EMDRIVE_START)
 {
   drive->state = OPERATION;
 }
 else if (drive->state == OPERATION && action != EMDRIVE_START)
 {
   drive->state = PRE_OPERATION;
 }

  CanTxMsgTypeDef tx;
  tx.IDE = CAN_ID_STD;
  tx.RTR = CAN_RTR_DATA;
  tx.StdId = ID_EMDRIVE_NMT_CONTROL;
  tx.DLC = 2;
  tx.Data[0] = action;  // testing start mc
  tx.Data[1] = 0x00;

  xQueueSendToBack(can->q_tx, &tx, 100);
}

// @brief: function for parsing the data returned from the emdrive
//         TPDO1 - TPDO3 values. These are sent after the sync function 
//         initiates the transaction.
void emdrive_parse_pdo(CAN_IDs_t id, uint8_t * data, emdrive_t * drive)
{
  emdrive_tpdo_t * pdo = &(drive->data);

  if (id == (ID_EMDRIVE_SLAVE_PDO_1 | NODE_ID))
  {
    pdo->torque_actual = parse_from_lil_16(data + BEGIN_DATA_BYTE(6));
    pdo->position_actual = parse_from_lil_32(data + BEGIN_DATA_BYTE(2));
    pdo->status_word = parse_from_lil_16(data);
    emdrive_check_statusword(drive);
  }
  else if (id == (ID_EMDRIVE_SLAVE_PDO_2 | NODE_ID))
  {
    pdo->electrical_power = parse_from_lil_16(data + BEGIN_DATA_BYTE(4));
    pdo->error_codes = parse_from_lil_32(data + BEGIN_DATA_BYTE(2));
    pdo->motor_temp = data[1];
    pdo->emdrive_temp = data[0];
  }
  else if (id == (ID_EMDRIVE_SLAVE_PDO_3 | NODE_ID))
  {
    pdo->phase_b_current = parse_from_lil_16(data + BEGIN_DATA_BYTE(6));
    pdo->velocity = parse_from_lil_32((data + BEGIN_DATA_BYTE(2)));
    pdo->actual_current = parse_from_lil_16(data);
  }

} 

// @brief: Function for commanding torque from motor controller
// @author: Chris Fallon
void emdrive_move_the_car_yo(emdrive_t * drive, int16_t torque, CAN_Bus_TypeDef * can)
{
  CanTxMsgTypeDef tx;
  tx.StdId = ID_EMDRIVE_MASTER_PDO_1 | NODE_ID; // and with the emdrive ID, set to 1 in the config tool when I wrote this.
  tx.RTR = CAN_RTR_DATA;
  tx.IDE = CAN_ID_STD;
  tx.DLC = 8;
  tx.Data[0] = 0x0F;
  tx.Data[1] = 0x00;
  // don't care, not using velocity control mode
  for (uint8_t i = 2; i < 6; i++)
  {
    tx.Data[i] = 0;
  }
  // Apparently this is little endian. that's dumb emdrive.
  tx.Data[6] = (uint8_t) torque & 0xFF;
  tx.Data[7] = (torque >> 8) & 0xFF;

  xQueueSendToBack(can->q_tx, &tx, 100);
}

// @brief: Function which checks the statusword 
// @return: 
uint16_t emdrive_check_statusword(emdrive_t * drive)
{
  uint16_t status = drive->data.status_word;
  if (status & (1 << WARNING))
  {
    drive->warning = true;
  }
  if (status & (1 << FAULT))
  {
    drive->err = drive->data.error_codes;
  }
  return 0;
}
