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

void HAL_CAN_RxFifo0MsgPendingCallback(CAN_HandleTypeDef *hcan)
{
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


void VCANFilterConfig() {
  CAN_FilterTypeDef FilterConf;

  FilterConf.FilterIdHigh =         ID_RINEHART_STATION_TX << 5; // 2 num
  FilterConf.FilterIdLow =          ID_PEDALBOX2 << 5; // 0
  FilterConf.FilterMaskIdHigh =     ID_DASHBOARD << 5;       // 3
  FilterConf.FilterMaskIdLow =      ID_WHEEL_FRONT << 5;       // 1
  FilterConf.FilterFIFOAssignment = CAN_FilterFIFO0;
  FilterConf.FilterBank = 0;
  FilterConf.FilterMode = CAN_FILTERMODE_IDLIST;
  FilterConf.FilterScale = CAN_FILTERSCALE_16BIT;
  FilterConf.FilterActivation = ENABLE;
  HAL_CAN_ConfigFilter(car.phvcan, &FilterConf);

  FilterConf.FilterIdHigh =         ID_WHEEL_REAR << 5; // 2 num
  FilterConf.FilterIdLow =          0 << 5; // 0
  FilterConf.FilterMaskIdHigh =     0 << 5;       // 3
  FilterConf.FilterMaskIdLow =      0 << 5;       // 1
  FilterConf.FilterFIFOAssignment = CAN_FilterFIFO0;
  FilterConf.FilterBank = 1;
  FilterConf.FilterMode = CAN_FILTERMODE_IDLIST;
  FilterConf.FilterScale = CAN_FILTERSCALE_16BIT;
  FilterConf.FilterActivation = ENABLE;
  HAL_CAN_ConfigFilter(car.phvcan, &FilterConf);
}

void DCANFilterConfig() {
  CAN_FilterTypeDef FilterConf;
  FilterConf.FilterIdHigh =         ID_BMS_MACRO << 5; // 2 num
  FilterConf.FilterIdLow =          0 << 5; // 0
  FilterConf.FilterMaskIdHigh =     0x000;       // 3
  FilterConf.FilterMaskIdLow =      0x000;       // 1
  FilterConf.FilterFIFOAssignment = CAN_FilterFIFO1;
  FilterConf.FilterBank = 1;
  FilterConf.FilterMode = CAN_FILTERMODE_IDLIST;
  FilterConf.FilterScale = CAN_FILTERSCALE_16BIT;
  FilterConf.FilterActivation = ENABLE;
  HAL_CAN_ConfigFilter(car.phdcan, &FilterConf);
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

    if (xQueueReceive(car.q_rx_vcan, &rx, (TickType_t) 5) == pdTRUE)
    {
      //A CAN message has been received
      //check what kind of message we received
      switch (rx.StdId) {
        case ID_PEDALBOX2: { //if pedalbox1 message
          processPedalboxFrame(&rx);
          break;
        }
        case  ID_DASHBOARD: {
        	switch(rx.Data[0]) {
        	  case 0:
        	  case 1:
        	    ISR_StartButtonPressed();
              break;
        	  case 2:
        	    //traction toggle
        	    car.traction_en = !car.traction_en;
        	    break;
        	  case 3:
        	    car.pow_lim.power_lim_en = !car.pow_lim.power_lim_en;
        	    break;
        	}

          break;
        }
        case ID_WHEEL_FRONT:
        case ID_WHEEL_REAR:
          processWheelModuleFrame(&rx);
          break;
      }
    }
    
    if (xQueueReceive(car.q_rx_dcan, &rx, portMAX_DELAY) == pdTRUE) {
    	switch (rx.StdId) {
        case ID_POWER_LIMIT:
        {
          processCalibratePowerLimit(&rx);
          break;
        }
        case ID_BMS_MACRO:
        {
          process_bms_frame(&rx);
          break;
        }
    	}
    }

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

//called from rx_process frame and updates the variables used for power limiting
int process_bms_frame(CanRxMsgTypeDef* rx) {
	//process the bms can frame
	//take the BMS semaphore
  car.bms_params.pack_current = (rx->Data[0] << 8) | rx->Data[1];
  car.bms_params.pack_volt = (rx->Data[2] << 8) | rx->Data[3];
  car.bms_params.pack_soc = rx->Data[4];
  car.bms_params.high_temp = rx->Data[5];
  car.bms_params.low_cell_volt = (rx->Data[6] << 8) | rx->Data[7];
	return 0;
}


void processCalibratePowerLimit(CanRxMsgTypeDef* rx) {
	car.pow_lim.power_hard_lim = rx->Data[0] << 24 | rx->Data[1] << 16 | rx->Data[2] << 8 | rx->Data[3];
	car.pow_lim.power_soft_lim = (car.pow_lim.power_hard_lim * 97) / 100; //97%
	car.pow_lim.power_thresh = (car.pow_lim.power_hard_lim * 90) / 100; //90%
}

/***************************************************************************
*
*     Function Information
*
*     Name of Function: process_IMU
*
*     Programmer's Name: Matt Flanagan, matthewdavidflanagan@outlook.com
*
*     Function Return Type: none
*
*     Parameters (list data type, name, and comment one per line):
*			1. CanRxMsgTypeDef* rx, CAN frame of IMU
*      Global Dependents:
*	    1. None
*
*     Function Description:
*     	Looks IMU data and see's if any axis is over 8G (rules for inertia switch). If so then
*     	fault the SDC. ASSUMES THAT THE IMU DATA COMING IS 16G Resolution
*
***************************************************************************/
void process_IMU(CanRxMsgTypeDef* rx) {
	int16_t accel_x = 0;
	int16_t accel_y = 0;
	int16_t accel_z = 0;

	if (rx->Data[7] == IMU_16G && rx->Data[0] == IMU_ACCEL) {
		//IMU is in 16g resolution and is an acceleration frame
		accel_x = (int16_t) (rx->Data[1] << 8) | rx->Data[2];
		accel_y = (int16_t) (rx->Data[3] << 8) | rx->Data[4];
		accel_z = (int16_t) (rx->Data[5] << 8) | rx->Data[6];

		if ((accel_x > IMU_8G_VAL/4 || accel_y > IMU_8G_VAL || accel_z > IMU_8G_VAL)
				&& (accel_x < IMU_8G_NEG || accel_y < IMU_8G_NEG || accel_z < IMU_8G_NEG)) {
			//car just got straight fucked
			//open the SDC
			HAL_GPIO_WritePin(SDC_CTRL_GPIO_Port, SDC_CTRL_Pin, GPIO_PIN_RESET);
			car.state = CAR_STATE_RESET;
		}
	}
}

/***************************************************************************
*
*     Function Information
*
*     Name of Function: processWheelModuleFrame
*
*     Programmer's Name: Jose Luis Tejada
*
*     Function Return Type: none
*
*     Parameters (list data type, name, and comment one per line):
*		1. CanRxMsgTypeDef* rx, CAN frame to be converted into a wheel speed rpm value
*      Global Dependents:
*	    1.
*
*     Function Description:
*     	Converts CAN frame into a rpm value and sends it to pedalboxmsg handler
*
***************************************************************************/
void processWheelModuleFrame(CanRxMsgTypeDef* rx){
	wheel_speed_t temp_ws;

	//Parse CAN message according to id
	switch (rx->StdId)
	{
		case 	0x700: //Todo, replace with defined constants
		{
			uint32_t wheel_FL = (uint8_t)rx->Data[WHEEL_FL_31_24_BYTE];
			wheel_FL = (wheel_FL << 8) | (uint8_t)rx->Data[WHEEL_FL_23_16_BYTE];
			wheel_FL = (wheel_FL << 8) | (uint8_t)rx->Data[WHEEL_FL_15_8_BYTE];
			wheel_FL = (wheel_FL << 8) | (uint8_t)rx->Data[WHEEL_FL_7_0_BYTE];

			uint32_t wheel_FR = (uint8_t)rx->Data[WHEEL_FR_31_24_BYTE];
			wheel_FR = (wheel_FR << 8) | (uint8_t)rx->Data[WHEEL_FR_23_16_BYTE];
			wheel_FR = (wheel_FR << 8) | (uint8_t)rx->Data[WHEEL_FR_15_8_BYTE];
			wheel_FR = (wheel_FR << 8) | (uint8_t)rx->Data[WHEEL_FR_7_0_BYTE];

			float rpm_FL = (float)wheel_FL / 10000.0;
			float rpm_FR = (float)wheel_FR / 10000.0;
			temp_ws.FL_rpm = rpm_FL;
			temp_ws.FR_rpm = rpm_FR;
			break;
		}
		case	0x701:
		{
			uint32_t wheel_RL = (uint8_t)rx->Data[WHEEL_RL_31_24_BYTE];
			wheel_RL = (wheel_RL << 8) | (uint8_t)rx->Data[WHEEL_RL_23_16_BYTE];
			wheel_RL = (wheel_RL << 8) | (uint8_t)rx->Data[WHEEL_RL_15_8_BYTE];
			wheel_RL = (wheel_RL << 8) | (uint8_t)rx->Data[WHEEL_RL_7_0_BYTE];

			uint32_t wheel_RR = (uint8_t)rx->Data[WHEEL_RR_31_24_BYTE];
			wheel_RR = (wheel_RR << 8) | (uint8_t)rx->Data[WHEEL_RR_23_16_BYTE];
			wheel_RR = (wheel_RR << 8) | (uint8_t)rx->Data[WHEEL_RR_15_8_BYTE];
			wheel_RR = (wheel_RR << 8) | (uint8_t)rx->Data[WHEEL_RR_7_0_BYTE];

			float rpm_RL = (float) wheel_RL / 10000.0;
			float rpm_RR = (float) wheel_RR / 10000.0;
			temp_ws.RL_rpm = rpm_RL;
			temp_ws.RR_rpm = rpm_RR;
			break;
		}
	}

	car.wheel_rpm = temp_ws;
}
