// File: CAN_Bus.c
// Author: Chris Fallon
// Purpose: A generic CAN source file to handle any of the non implementation specific setup



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


// @authors: Ben Ng
//           Chris Fallon
// @brief: Initialize which messages we want to accept on the can bus
//
// @NOTE: this is technically implementation specific, so change the
//        IDs to be which IDs need to be accepted. This is default for main module
void VCANFilterConfig(CAN_HandleTypeDef * hcan)
{
  CAN_FilterTypeDef FilterConf;

  FilterConf.FilterIdHigh =         ID_RINEHART_STATION_TX << 5; // left shift because the ID is in the high bits of the actual registers
  FilterConf.FilterIdLow =          ID_DASHBOARD << 5;
  FilterConf.FilterMaskIdHigh =     ID_PEDALBOX2 << 5;
  FilterConf.FilterMaskIdLow =      ID_WHEEL_REAR << 5;
  FilterConf.FilterFIFOAssignment = CAN_FilterFIFO0;
  FilterConf.FilterBank = 0;
  FilterConf.FilterMode = CAN_FILTERMODE_IDLIST;
  FilterConf.FilterScale = CAN_FILTERSCALE_16BIT;
  FilterConf.FilterActivation = ENABLE;
  HAL_CAN_ConfigFilter(hcan, &FilterConf);
}


// @authors: Ben Ng
//           Chris Fallon
// @brief: Initialize which messages we want to accept on the can bus
//
// @NOTE: this is technically implementation specific, so change the
//        IDs to be which IDs need to be accepted. This is default for main module
void DCANFilterConfig(CAN_HandleTypeDef * hcan)
{
  CAN_FilterTypeDef FilterConf;
  FilterConf.FilterIdHigh =         ID_WHEEL_FRONT << 5;
  FilterConf.FilterIdLow =          ID_WHEEL_REAR << 5;
  FilterConf.FilterMaskIdHigh =     0x7FF;
  FilterConf.FilterMaskIdLow =      0x7FF;
  FilterConf.FilterFIFOAssignment = CAN_FilterFIFO1;
  FilterConf.FilterBank = 1;
  FilterConf.FilterMode = CAN_FILTERMODE_IDLIST;
  FilterConf.FilterScale = CAN_FILTERSCALE_16BIT;
  FilterConf.FilterActivation = ENABLE;
  HAL_CAN_ConfigFilter(hcan, &FilterConf);
}


// @Task_Name: taskTX_CAN
// @brief: broadcast CAN messages
// @author: Ben Ng ; email: undefined
// @edited: Chris Fallon
void taskTX_CAN(void * params) {
  CanTxMsgTypeDef tx;
  CAN_Bus_TypeDef * can = (CAN_Bus_TypeDef *) params;

  for (;;) 
  {
    //check if this task is triggered
    if (xQueuePeek(can->q_tx, &tx, portMAX_DELAY) == pdTRUE) 
    {
      xQueueReceive(can->q_tx, &tx, portMAX_DELAY);  //actually take item out of queue
      broadcast_can_msg(&tx, can->hcan);
    }
  }
  vTaskDelete(NULL);
}