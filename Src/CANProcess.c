
#include <CANProcess.h>
#include "CAN_Bus.h"
#include "car.h"
#include "PedalBox.h"
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
void taskRXCANProcess(void * params) {

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
          processPedalboxFrame(&rx);
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
        	    send_ack(ID_DASHBOARD_ACK, 3);
        	    break;
        	  case 3:
        	    car.power_limit.enabled = !car.power_limit.enabled;
        	    send_ack(ID_DASHBOARD_ACK, 4);
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
          processCalibratePowerLimit(&rx);
          break;
        }
        case ID_BMS:
        {
          process_bms_frame(&rx);
          break;
        }
        case ID_WHEEL_REAR:
        {
          // TODO move this to can ISR
        	calc_wheel_speed(rx.StdId, rx.Data);
        	break;
        }
      }
    }

    vTaskDelayUntil(&last_tick, 10);
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
	Pedalbox_msg_t pedalboxmsg;

	///////////SCRUB DATA the from the CAN frame//////////////
	//mask then shift the throttle value data
	uint8_t throttle1_7_0  = rx->Data[PEDALBOX1_THROT1_7_0_BYTE] >> PEDALBOX1_THROT1_7_0_OFFSET;  //Throttle 1 Value (7:0) [7:0]
	uint8_t throttle1_11_8 =
		(rx->Data[PEDALBOX1_THROT1_11_8_BYTE] & PEDALBOX1_THROT1_11_8_MASK) >>
		PEDALBOX1_THROT1_11_8_OFFSET;  //Throttle 1 Value (11:8) [3:0]

	uint8_t throttle2_7_0  = rx->Data[PEDALBOX1_THROT2_7_0_BYTE] >> PEDALBOX1_THROT2_7_0_OFFSET;  //Throttle 2 Value (7:0) [7:0]
	uint8_t throttle2_11_8 =
		(rx->Data[PEDALBOX1_THROT2_11_8_BYTE] & PEDALBOX1_THROT2_11_8_MASK) >>
		PEDALBOX1_THROT2_11_8_OFFSET;  //Throttle 2 Value (11:8) [3:0]

	//mask then shift the brake value data
	uint8_t brake1_7_0  = rx->Data[PEDALBOX1_BRAKE1_7_0_BYTE] >> PEDALBOX1_BRAKE1_7_0_OFFSET;  //brake 1 Value (7:0) [7:0]
	uint8_t brake1_11_8 =
		(rx->Data[PEDALBOX1_BRAKE1_11_8_BYTE] & PEDALBOX1_BRAKE1_11_8_MASK) >>
		PEDALBOX1_BRAKE1_11_8_OFFSET;  //brake 1 Value (11:8) [3:0]

	uint8_t brake2_7_0  = rx->Data[PEDALBOX1_BRAKE2_7_0_BYTE] >> PEDALBOX1_BRAKE2_7_0_OFFSET;  //brake 2 Value (7:0) [7:0]
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
	xQueueSendToBack(car.pedalbox.pb_msg_q, &pedalboxmsg, 100);
}

//void processCalibrate(CanRxMsgTypeDef* rx) {
//  //set the calibration flag, so calibration values are updated upon reception of new pedalboxmsg
//  if      (rx->Data[0] == 0x01) {
//    car.calibrate_flag = CALIBRATE_THROTTLE_MAX; //calibrate high
//  } else if (rx->Data[0] == 0x02) {
//    car.calibrate_flag = CALIBRATE_THROTTLE_MIN; //calibrate low
//  } else if (rx->Data[0] == 0x03) {
//    car.calibrate_flag = CALIBRATE_BRAKE_MAX; //calibrate high
//  } else if (rx->Data[0] == 0x04) {
//    car.calibrate_flag = CALIBRATE_BRAKE_MIN; //calibrate low
//  } else {
//    car.calibrate_flag = CALIBRATE_NONE;
//  }
//
//}

//called from rx_process frame and updates the variables used for power limiting
int process_bms_frame(CanRxMsgTypeDef* rx) {
	//process the bms can frame
	//take the BMS semaphore
  car.bms.pack_current = (rx->Data[0] << 8) | rx->Data[1];
  car.bms.pack_volt = (rx->Data[2] << 8) | rx->Data[3];
  car.bms.pack_soc = rx->Data[4];
  car.bms.high_temp = rx->Data[5];
  car.bms.low_cell_volt = (rx->Data[6] << 8) | rx->Data[7];
	return 0;
}


void processCalibratePowerLimit(CanRxMsgTypeDef* rx) {
	car.power_limit.power_hard_lim = rx->Data[0] << 24 | rx->Data[1] << 16 | rx->Data[2] << 8 | rx->Data[3];
	car.power_limit.power_soft_lim = (car.power_limit.power_hard_lim * 97) / 100; //97%
	car.power_limit.power_thresh = (car.power_limit.power_hard_lim * 90) / 100; //90%
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


void send_ack(uint16_t can_id, uint16_t response) {
  CanTxMsgTypeDef tx;
  tx.IDE = CAN_ID_STD;
  tx.RTR = CAN_RTR_DATA;
  tx.StdId = can_id;
  tx.DLC = 1;
  tx.Data[0] = response;
  xQueueSendToBack(car.vcan.q_tx, &tx, 100);
}


// @authors: Chris Fallon
// @brief: Calculate the wheel speed for the given ID
void calc_wheel_speed(uint32_t id, uint8_t * data)
{
	volatile float *left;
	volatile float *right;
	uint32_t left_raw;
	uint32_t right_raw;

  if (id == ID_WHEEL_FRONT)
  {
  	left = &car.wheel_rpms.FL_rpm;
  	right = &car.wheel_rpms.FR_rpm;
  }
  else
  {
  	left = &car.wheel_rpms.RL_rpm;
  	right = &car.wheel_rpms.RR_rpm;
  }

  left_raw = ((uint32_t) data[0]) << 24
  		| ((uint32_t) data[1] << 16)
			| ((uint32_t) data[2] << 8)
			| data[3];

  right_raw = ((uint32_t) data[4]) << 24
  		| ((uint32_t) data[5] << 16)
			| ((uint32_t) data[6] << 8)
			| data[7];

  // 10000 is the scalar from DAQ
  *left = left_raw / DAQ_SCALAR;
  *right = right_raw / DAQ_SCALAR;
}
