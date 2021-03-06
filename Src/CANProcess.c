/***************************************************************************
*
*     File Information
*
*     Name of File: CANRXProcess.c
*
*     Authors (Include Email):
*       1. Ben Ng,       xbenng@gmail.com
*
*     File dependents: (header files, flow charts, referenced documentation)
*       1. FreeRTOS.h
*     2. stm32f7xx_hal_can.h
*     3. CANRXProcess.h
*
*     File Description: Used for interpreting incoming CAN messages on
*           main module
*
***************************************************************************/
#include <CANProcess.h>

#include "car.h"
#include "PedalBox.h"


void HAL_CAN_RxFifo0MsgPendingCallback(CAN_HandleTypeDef *hcan)
{
	//HAL_GPIO_TogglePin(LD4_GPIO_Port, LD4_Pin);

	CanRxMsgTypeDef rx;
	CAN_RxHeaderTypeDef header;
	HAL_CAN_GetRxMessage(hcan, 0, &header, rx.Data);
	rx.DLC = header.DLC;
	rx.StdId = header.StdId;
	xQueueSendFromISR(car.q_rx_dcan, &rx, NULL);
}

void HAL_CAN_RxFifo1MsgPendingCallback(CAN_HandleTypeDef *hcan)
{
	CanRxMsgTypeDef rx;
	CAN_RxHeaderTypeDef header;
	HAL_CAN_GetRxMessage(hcan, 1, &header, rx.Data);
	rx.DLC = header.DLC;
	rx.StdId = header.StdId;
	xQueueSendFromISR(car.q_rx_vcan, &rx, NULL);
}


void DCANFilterConfig() {
  CAN_FilterTypeDef FilterConf;

  FilterConf.FilterIdHigh =         ID_RINEHART_STATION_TX << 5; // 2 num
  FilterConf.FilterIdLow =          ID_PEDALBOX2 << 5; // 0
  FilterConf.FilterMaskIdHigh =     ID_DASHBOARD << 5;       // 3
  FilterConf.FilterMaskIdLow =      0x7fe;       // 1
  FilterConf.FilterFIFOAssignment = CAN_FilterFIFO0;
  FilterConf.FilterBank = 0;
  FilterConf.FilterMode = CAN_FILTERMODE_IDLIST;
  FilterConf.FilterScale = CAN_FILTERSCALE_16BIT;
  FilterConf.FilterActivation = ENABLE;
  HAL_CAN_ConfigFilter(car.phdcan, &FilterConf);
}

void VCANFilterConfig() {
  CAN_FilterTypeDef FilterConf;
  FilterConf.FilterIdHigh =         ID_WHEEL_FRONT << 5; // 2 num
  FilterConf.FilterIdLow =          ID_WHEEL_REAR << 5; // 0
  FilterConf.FilterMaskIdHigh =     0x000;       // 3
  FilterConf.FilterMaskIdLow =      0x000;       // 1
  FilterConf.FilterFIFOAssignment = CAN_FilterFIFO1;
  FilterConf.FilterBank = 1;
  FilterConf.FilterMode = CAN_FILTERMODE_IDLIST;
  FilterConf.FilterScale = CAN_FILTERSCALE_16BIT;
  FilterConf.FilterActivation = ENABLE;
  HAL_CAN_ConfigFilter(car.phvcan, &FilterConf);
}


/***************************************************************************
*
*     Function Information
*
*     Name of Function: taskTX_DCAN
*
*     Programmer's Name: Ben Ng, xbenng@gmail.com
*
*     Function Return Type: none
*
*     Parameters (list data type, name, and comment one per line):
*
*      Global Dependents:
*    1.
*
*     Function Description:
*       Task function to send CAN messages using the CAN peripheral
*
***************************************************************************/
void taskTX_DCAN() {
  CanTxMsgTypeDef tx;
  
  for (;;) {
    //check if this task is triggered
    if (xQueuePeek(car.q_tx_dcan, &tx, portMAX_DELAY) == pdTRUE)
    {
      xQueueReceive(car.q_tx_dcan, &tx, portMAX_DELAY);  //actually take item out of queue
      CAN_TxHeaderTypeDef header;
      header.DLC = tx.DLC;
      header.IDE = tx.IDE;
      header.RTR = tx.RTR;
      header.StdId = tx.StdId;
      header.TransmitGlobalTime = DISABLE;
      uint32_t mailbox;
      while (!HAL_CAN_GetTxMailboxesFreeLevel(car.phdcan)); // while mailboxes not free
      HAL_CAN_AddTxMessage(car.phdcan, &header, tx.Data, &mailbox);
    }
  }
}

/***************************************************************************
*
*     Function Information
*
*     Name of Function: taskTX_VCAN
*
*     Programmer's Name: Ben Ng, xbenng@gmail.com
*
*     Function Return Type: none
*
*     Parameters (list data type, name, and comment one per line):
*
*      Global Dependents:
*    1.
*
*     Function Description:
*       Task function to send CAN messages using the CAN peripheral
*
***************************************************************************/
void taskTX_VCAN() {
  CanTxMsgTypeDef tx;
  
  for (;;) {
    //check if this task is triggered
    if (xQueuePeek(car.q_tx_vcan, &tx, portMAX_DELAY) == pdTRUE) {
      xQueueReceive(car.q_tx_vcan, &tx, portMAX_DELAY);  //actually take item out of queue
      CAN_TxHeaderTypeDef header;
      header.DLC = tx.DLC;
      header.IDE = tx.IDE;
      header.RTR = tx.RTR;
      header.StdId = tx.StdId;
      header.TransmitGlobalTime = DISABLE;
      uint32_t mailbox;
      while (!HAL_CAN_GetTxMailboxesFreeLevel(car.phvcan)); // while mailboxes not free
      HAL_CAN_AddTxMessage(car.phvcan, &header, tx.Data, &mailbox);
    }
  }
}

/***************************************************************************
*
*     Function Information
*
*     Name of Function: taskRXCANprocess
*
*     Programmer's Name: Ben Ng, xbenng@gmail.com
*
*     Function Return Type: none
*
*     Parameters (list data type, name, and comment one per line):
*
*      Global Dependents:
*    1.-
*
*     Function Description:
*       Task function to process received CAN Messages.
*       CanRxMsgTypeDef are sent here to the q_rxcan queue to be processed
*       from the CAN RX interrupt handler.
*       The data is process and handled according to what kind of message is received
*
***************************************************************************/
void taskRXCANProcess() {

  CanRxMsgTypeDef rx;  //CanRxMsgTypeDef to be received on the queue
  while (1) {
    HAL_GPIO_TogglePin(LD4_GPIO_Port, LD4_Pin);
    //if there is a CanRxMsgTypeDef in the queue, pop it, and store in rx
    BaseType_t peek = xQueuePeek(car.q_rx_dcan, &rx, (TickType_t) 5);

    if (peek == pdTRUE)
    {
      //A CAN message has been received
      //check what kind of message we received
    	xQueueReceive(car.q_rx_dcan, &rx, (TickType_t) 5);

      switch (rx.StdId) {
        case ID_PEDALBOX2: { //if pedalbox1 message
          processPedalboxFrame(&rx);
          break;
        }
        case ID_PEDALBOXCALIBRATE: {
          processCalibrate(&rx);
          break;
        }
        case  ID_DASHBOARD: {
        	if (rx.Data[0] == 1)
        	{
        		ISR_StartButtonPressed();
        	}
        	else
        	{
        		//process other button functionality
        	}
          break;
        }
        case  ID_DASHBOARD1: {
          if (car.state == CAR_STATE_READY2DRIVE) {
            car.state = CAR_STATE_RECOVER;
          }
          break;
        }
        case  ID_DASHBOARD2: {
          HAL_GPIO_TogglePin(PUMP_GPIO_Port, PUMP_Pin);
          break;
        }
      }
    }
    
//    if (xQueueReceive(car.phdcan, &rx, (TickType_t) 5) == pdTRUE) {
//    	switch (rx.StdId) {
//    	case ID_WHEEL_FRONT:
//    		//TODO
//    		break;
//    	case ID_WHEEL_REAR:
//    		//TODO
//    		break;
//    	}
//    }
    vTaskDelay(10);

  }
}

/***************************************************************************
*
*     Function Information
*
*     Name of Function: processPedalboxFrame
*
*     Programmer's Name: Ben Ng, xbenng@gmail.com
*
*     Function Return Type: none
*
*     Parameters (list data type, name, and comment one per line):
*   1. CanRxMsgTypeDef* rx, CAN frame to be converted into a pedalbox message
*      Global Dependents:
*     1.
*
*     Function Description:
*       Converts CAN frame into a pedalbox message and sends it to pedalboxmsg handler
*
***************************************************************************/
void processPedalboxFrame(CanRxMsgTypeDef* rx) {
  if (car.pb_mode == PEDALBOX_MODE_DIGITAL) { //
    Pedalbox_msg_t pedalboxmsg;
    
    ///////////SCRUB DATA the from the CAN frame//////////////
    //mask then shift the throttle value data
    uint8_t throttle1_7_0   =
      rx->Data[PEDALBOX1_THROT1_7_0_BYTE]  >> PEDALBOX1_THROT1_7_0_OFFSET;  //Throttle 1 Value (7:0) [7:0]
    uint8_t throttle1_11_8  =
      (rx->Data[PEDALBOX1_THROT1_11_8_BYTE] & PEDALBOX1_THROT1_11_8_MASK) >>
      PEDALBOX1_THROT1_11_8_OFFSET;  //Throttle 1 Value (11:8) [3:0]
    uint8_t throttle2_7_0 =
      rx->Data[PEDALBOX1_THROT2_7_0_BYTE]  >> PEDALBOX1_THROT2_7_0_OFFSET;  //Throttle 2 Value (7:0) [7:0]
    uint8_t throttle2_11_8  =
      (rx->Data[PEDALBOX1_THROT2_11_8_BYTE] & PEDALBOX1_THROT2_11_8_MASK) >>
      PEDALBOX1_THROT2_11_8_OFFSET;  //Throttle 2 Value (11:8) [3:0]
      
    //mask then shift the brake value data
    uint8_t brake1_7_0  =
      rx->Data[PEDALBOX1_BRAKE1_7_0_BYTE]  >> PEDALBOX1_BRAKE1_7_0_OFFSET;  //brake 1 Value (7:0) [7:0]
    uint8_t brake1_11_8 =
      (rx->Data[PEDALBOX1_BRAKE1_11_8_BYTE] & PEDALBOX1_BRAKE1_11_8_MASK) >>
      PEDALBOX1_BRAKE1_11_8_OFFSET;  //brake 1 Value (11:8) [3:0]
    uint8_t brake2_7_0  =
      rx->Data[PEDALBOX1_BRAKE2_7_0_BYTE]  >> PEDALBOX1_BRAKE2_7_0_OFFSET;  //brake 2 Value (7:0) [7:0]
    uint8_t brake2_11_8 =
      (rx->Data[PEDALBOX1_BRAKE2_11_8_BYTE] & PEDALBOX1_BRAKE2_11_8_MASK) >>
      PEDALBOX1_BRAKE2_11_8_OFFSET;  //brake 2 Value (11:8) [3:0]
      
      
    //build the data
    pedalboxmsg.throttle1_raw = 0;
    pedalboxmsg.throttle1_raw |= throttle1_7_0 << 0;
    pedalboxmsg.throttle1_raw |= throttle1_11_8 << 8;
    pedalboxmsg.throttle2_raw = 0;
    pedalboxmsg.throttle2_raw |= throttle2_7_0 << 0;
    pedalboxmsg.throttle2_raw |= throttle2_11_8 << 8;
    pedalboxmsg.brake1_raw = 0;
    pedalboxmsg.brake1_raw |= brake1_7_0 << 0;
    pedalboxmsg.brake1_raw |= brake1_11_8 << 8;
    pedalboxmsg.brake2_raw = 0;
    pedalboxmsg.brake2_raw |= brake2_7_0 << 0;
    pedalboxmsg.brake2_raw |= brake2_11_8 << 8;
    
    
    //send to pedalboxmsg to queue
    xQueueSendToBack(car.q_pedalboxmsg, &pedalboxmsg, 100);
  }
}

void processCalibrate(CanRxMsgTypeDef* rx) {
  //set the calibration flag, so calibration values are updated upon reception of new pedalboxmsg
  if      (rx->Data[0] == 0x01) {
    car.calibrate_flag = CALIBRATE_THROTTLE_MAX; //calibrate high
  } else if (rx->Data[0] == 0x02) {
    car.calibrate_flag = CALIBRATE_THROTTLE_MIN; //calibrate low
  } else if (rx->Data[0] == 0x03) {
    car.calibrate_flag = CALIBRATE_BRAKE_MAX; //calibrate high
  } else if (rx->Data[0] == 0x04) {
    car.calibrate_flag = CALIBRATE_BRAKE_MIN; //calibrate low
  } else {
    car.calibrate_flag = CALIBRATE_NONE;
  }
  
}
