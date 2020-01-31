#include "emdrive.h"


// @brief: Init function
// @author: Chris Fallon
void emdrive_init(emdrive_t * drive)
{
  drive->data.position_actual = 0;
  drive->data.status_word = 0;
  drive->data.error_codes = 0;
  
  drive->state = PRE_OPERATION;
  
  drive->err = OK;

  drive->warning = false;
  drive->emdrive_init_rcvd = false;

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
  xSendToBack(can->q_tx, &tx, 100);
}



// @brief: Function for sending config parameters to the emdrive
// @author: Chris Fallon
void emdrive_control(emdrive_nmt_command_t action, emdrive_t * drive, CAN_Bus_TypeDef * can)
{
  if (!drive->emdrive_init_rcvd)
  {
    return;
  }

  CanTxMsgTypeDef tx;
  tx.StdId = ID_EMDRIVE_NMT_CONTROL;
  if (action == START)
  {

  }
}

// @brief: function for parsing the data returned from the emdrive
//         TPDO1 - TPDO3 values. These are sent after the sync function 
//         initiates the transaction.
void emdrive_parse_pdo(CAN_IDs_t id)
{

}