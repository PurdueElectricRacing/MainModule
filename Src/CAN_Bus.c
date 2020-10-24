// File: CAN_Bus.c
// Author: Chris Fallon
// Purpose: A generic CAN source file to handle any of the non implementation specific setup



#include "CAN_Bus.h"
#include "CANProcess.h"
#include "main.h"



// @author: Chris Fallon
// @brief: generic function to initialize a CAN Bus struct
//         creates queues and assigns hcan pointers
void init_can_bus(CAN_Bus_TypeDef * bus, CAN_HandleTypeDef * hcan, uint16_t rx_q_size, uint16_t tx_q_size)
{
  bus->hcan = hcan;
  bus->q_rx = xQueueCreate(rx_q_size, sizeof(CanRxMsgTypeDef));
  bus->q_tx = xQueueCreate(tx_q_size, sizeof(CanTxMsgTypeDef));
}


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
//  HAL_CAN_Transmit_IT(can);
}

extern CAN_HandleTypeDef hcan1;
extern CAN_HandleTypeDef hcan2;


// @authors: Ben Ng
//           Chris Fallon
// @brief: Initialize which messages we want to accept on the can bus
//
// @NOTE: this is technically implementation specific, so change the
//        IDs to be which IDs need to be accepted. This is default for main module
void VCANFilterConfig(CAN_HandleTypeDef * hcan)
{
  CAN_FilterTypeDef FilterConf;

  FilterConf.FilterIdHigh =         300 << 5; // left shift because the ID is in the high bits of the actual registers
  FilterConf.FilterIdLow =          ID_DASHBOARD << 5;
  FilterConf.FilterMaskIdHigh =     ID_PEDALBOX << 5;
  FilterConf.FilterMaskIdLow =      ID_WHEEL_REAR << 5;
  FilterConf.FilterFIFOAssignment = CAN_FilterFIFO0;
  FilterConf.FilterBank = 0;
  FilterConf.FilterMode = CAN_FILTERMODE_IDLIST;
  FilterConf.FilterScale = CAN_FILTERSCALE_16BIT;
  FilterConf.FilterActivation = ENABLE;
  HAL_CAN_ConfigFilter(&hcan1, &FilterConf);
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
  FilterConf.FilterMaskIdHigh =     0;
  FilterConf.FilterMaskIdLow =      0;
  FilterConf.FilterFIFOAssignment = CAN_FilterFIFO1;
  FilterConf.FilterBank = 1;
  FilterConf.FilterMode = CAN_FILTERMODE_IDLIST;
  FilterConf.FilterScale = CAN_FILTERSCALE_16BIT;
  FilterConf.FilterActivation = ENABLE;
  HAL_CAN_ConfigFilter(&hcan2, &FilterConf);
}


// @Task_Name: taskTX_CAN
// @brief: broadcast CAN messages
// @author: Ben Ng ; email: undefined
// @edited: Chris Fallon
void taskTX_CAN(void * params) {
  CanTxMsgTypeDef tx;
  CAN_Bus_TypeDef * can = (CAN_Bus_TypeDef *) params;
  TickType_t timeout = 0;
  TickType_t last_tick;
  for (;;) 
  {
  	last_tick = xTaskGetTickCount();
    //check if this task is triggered
    if (xQueueReceive(can->q_tx, &tx, timeout) == pdTRUE)
    {
      // flash orange LED on
      // may not appear to work if the error is not persistent.
      HAL_StatusTypeDef err = broadcast_can_msg(&tx, can->hcan);
    }
    HAL_GPIO_TogglePin(GPIOD, LD3_Pin);
    vTaskDelayUntil(&last_tick, pdMS_TO_TICKS(1));
  }
  vTaskDelete(NULL);
}

void send_ack(uint16_t can_id, uint16_t response, CAN_Bus_TypeDef * can) {
  CanTxMsgTypeDef tx;
  tx.IDE = CAN_ID_STD;
  tx.RTR = CAN_RTR_DATA;
  tx.StdId = can_id;
  tx.DLC = 1;
  tx.Data[0] = response;
  xQueueSendToBack(can->q_tx, &tx, 100);
}



// @brief: Parsing function so I don't have to type the data array out each time.
// @param: uint8_t * data: address of the first byte of the data array
//         i.e. if first byte is data[3], pass (&data) + 3 * sizeof(uint8_t *)
 uint32_t parse_from_lil_32(uint8_t * data)
{
  return ((uint32_t) *(data + (3 * sizeof(uint8_t *)))) << 24 
  | ((uint32_t) *(data + (2 * sizeof(uint8_t *)))) << 16 
  | ((uint16_t) *(data + (sizeof(uint8_t *)))) << 8 
  | *(data);
}
// @brief: Parsing function so I don't have to type the data array out each time.
// @param: uint8_t * data: address of the first byte of the data array
//         i.e. if first byte is data[3], pass (&data) + 3 * sizeof(uint8_t *)
 uint32_t parse_from_big_32(uint8_t * data)
{
  return ((uint32_t) *(data)) << 24 
  | ((uint32_t) *(data + (sizeof(uint8_t *)))) << 16 
  | ((uint16_t) *(data + (2 * sizeof(uint8_t *)))) << 8 
  | *(data + (3 * sizeof(uint8_t *)));
}
// @brief: Parsing function so I don't have to type the data array out each time.
// @param: uint8_t * data: address of the first byte of the data array
//         i.e. if first byte is data[3], pass (&data) + 3 * sizeof(uint8_t *)
 uint16_t parse_from_lil_16(uint8_t * data)
{
  return ((uint16_t) *(data + sizeof(uint8_t *)) << 8) | *data;
}
// @brief: Parsing function so I don't have to type the data array out each time.
// @param: uint8_t * data: address of the first byte of the data array
//         i.e. if first byte is data[3], pass (&data) + 3 * sizeof(uint8_t *)
 uint16_t parse_from_big_16(uint8_t * data)
{
  return ((uint16_t) *data << 8) | *(data + sizeof(uint8_t *));
}


