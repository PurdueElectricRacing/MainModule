
#include <CANProcess.h>
#include "CAN_Bus.h"
#include "car.h"
#include "PedalBox.h"
#include "BMS.h"
#include "imu.h"

#include <string.h>


// Required ISRs for when a CAN message is received
void HAL_CAN_RxFifo0MsgPendingCallback(CAN_HandleTypeDef *hcan)
{
	CanRxMsgTypeDef rx;
	CAN_RxHeaderTypeDef header;
	HAL_CAN_GetRxMessage(hcan, 0, &header, rx.Data);
	rx.DLC = header.DLC;
	rx.StdId = header.StdId;
	xQueueSendFromISR(car.dcan.q_rx, &rx, NULL);
}

void HAL_CAN_RxFifo1MsgPendingCallback(CAN_HandleTypeDef *hcan)
{
	CanRxMsgTypeDef rx;
	CAN_RxHeaderTypeDef header;
	HAL_CAN_GetRxMessage(hcan, 1, &header, rx.Data);
	rx.DLC = header.DLC;
	rx.StdId = header.StdId;
	xQueueSendFromISR(car.vcan.q_rx, &rx, NULL);
}


// @authors: Ben Ng
//           Chris Fallon
// @brief: Task function to process received CAN Messages.
//         CanRxMsgTypeDef are sent here to the q_rxcan queue to be processed
//         from the CAN RX interrupt handler.
//         The data is process and handled according to what kind of message is received
void task_RX_CAN(void * params) {

  CanRxMsgTypeDef rx;  //CanRxMsgTypeDef to be received on the queue
  TickType_t last_tick;
  while (1) 
  {
    last_tick = xTaskGetTickCount();
    HAL_GPIO_TogglePin(LD4_GPIO_Port, LD4_Pin);
    //if there is a CanRxMsgTypeDef in the queue, pop it, and store in rx

    if (xQueueReceive(car.vcan.q_rx, &rx, (TickType_t) 5) == pdTRUE)
    {
      //A CAN message has been received
      //check what kind of message we received
      switch (rx.StdId) {
        case ID_PEDALBOX2:
        { //if pedalbox1 message
          processPedalboxFrame(rx.Data, &car.pedalbox);
          break;
        }
        case ID_DASHBOARD:
        {
        	switch(rx.Data[0])
        	{
        	  case 0:
        	  case 1:
        	    ISR_StartButtonPressed();
              break;
        	  case 2:
        	    //traction toggle
        	    car.tract_cont_en = !car.tract_cont_en;
        	    send_ack(ID_DASHBOARD_ACK, 3, &car.vcan);
        	    break;
        	  case 3:
        	    car.power_limit.enabled = !car.power_limit.enabled;
        	    send_ack(ID_DASHBOARD_ACK, 4, &car.vcan);
        	    break;
        	}

          break;
        }
      }
    }
    
    if (xQueueReceive(car.dcan.q_rx, &rx, portMAX_DELAY) == pdTRUE) {
    	switch (rx.StdId)
    	{
        case ID_POWER_LIMIT:
        {
          processCalibratePowerLimit(rx.Data, &car.power_limit);
          break;
        }
        case ID_BMS:
        {
          process_bms_frame(rx.Data, &car.bms);
          break;
        }
        case ID_WHEEL_REAR:
        {
          // TODO move this to can ISR
        	calc_wheel_speed(&car.wheels, rx.StdId, rx.Data);
        	break;
        }
        case ID_F_IMU:
        {
//          if (process_IMU(&rx) == REKT)
//          {
//            car.state = CAR_STATE_INIT;
//            HAL_GPIO_WritePin(SDC_CTRL_GPIO_Port, SDC_CTRL_Pin, 1);
//          }
          break;
        }
      }
    }

    vTaskDelayUntil(&last_tick, 10);
  }
}








