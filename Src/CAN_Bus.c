#include "CAN_Bus.h"


// @brief: Generic function  for broadcasting a CAN message
// @return: HAL Status
// @author: Chris Fallon
HAL_StatusTypeDef broadcast_can_msg(CanTxMsgTypeDef * tx, CAN_HandleTypeDef * can)
{
  CAN_TxHeaderTypeDef header;
  header.DLC = tx->DLC;
  header.IDE = tx->IDE;
  header.RTR = tx->RTR;
  header.StdId = tx->StdId;
  header.TransmitGlobalTime = DISABLE;
  uint32_t mailbox;
  while (!HAL_CAN_GetTxMailboxesFreeLevel(can)); // while mailboxes not free
  return HAL_CAN_AddTxMessage(can, &header, tx->Data, &mailbox);
}
