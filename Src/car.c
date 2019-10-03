/***************************************************************************
*
*     File Information
*
*     Name of File: car.c
*
*     Authors (Include Email):
*       1. Ben Ng       xbenng@gmail.com
*
*     File dependents: (header files, flow charts, referenced documentation)
*       1. car.h
*
*     File Description:
*       Functions to control the physical car
*
***************************************************************************/

#include "car.h"
#include <math.h>

#include "motor_controller_functions.h"

// @author: Ben Ng
//          Chris Fallon
// @brief: set brake light on or off
void carSetBrakeLight(Brake_light_status_t status)
{
  HAL_GPIO_WritePin(BRAKE_LIGHT_GPIO_Port, BRAKE_LIGHT_Pin, status);
}


void carInit() {
   // set dcdc pin high, active low logic
  HAL_GPIO_WritePin(DCDC_ENABLE_GPIO_Port, DCDC_ENABLE_Pin, GPIO_PIN_SET);

  car.state = CAR_STATE_INIT;
  car.throttle_acc = 0;
  car.brake = 0;

  car.vcan.hcan = &hcan1;
  car.dcan.hcan = &hcan2;

  car.pb_msg_rx_time = UINT32_MAX;
  car.tract_cont_en = false; //default traction control to off

  // set accelerator pedal position sensor errors to no errors
  pedalbox_init(&car.pedalbox);
  init_bms_struct(&car.bms); //setup the bms data
	init_pow_lim(&car.power_limit); //setup the power limiting
}

void ISR_StartButtonPressed() {
  if (car.state == CAR_STATE_INIT)
  {
    if (car.brake >= BRAKE_PRESSED_THRESHOLD//check if brake is pressed before starting car
        && HAL_GPIO_ReadPin(P_AIR_STATUS_GPIO_Port, P_AIR_STATUS_Pin) == (GPIO_PinState) PC_COMPLETE) //check if precharge has finished
  	{
  		car.state = CAR_STATE_PREREADY2DRIVE;
      //send acknowledge message to dashboard
			send_ack(ID_DASHBOARD_ACK, 1);
  	}
  }
  else {
    car.state = CAR_STATE_RESET;
    send_ack(ID_DASHBOARD_ACK, 2);
  }
}

// @authors: Kai Strubel
//           Ben Ng
//           Chris Fallon
// @brief: function to communicte Main is alive
//         also handles enabling charging of LV batteries
void taskHeartbeat(void * params) {
  
  TickType_t last_wake;

  while (1) 
  {
    last_wake = xTaskGetTickCount();

    HAL_GPIO_TogglePin(SDC_CTRL_GPIO_Port, SDC_CTRL_Pin);
    int hv_active_status = HAL_GPIO_ReadPin(P_AIR_STATUS_GPIO_Port, P_AIR_STATUS_Pin);
    // if HV is on, enable LV charging circuit
    if(hv_active_status == GPIO_PIN_SET)
    {
      HAL_GPIO_WritePin(LV_BATT_CHARGER_ENABLE_GPIO_Port, LV_BATT_CHARGER_ENABLE_Pin, GPIO_PIN_SET);
    }
    else
    {
      HAL_GPIO_WritePin(LV_BATT_CHARGER_ENABLE_GPIO_Port, LV_BATT_CHARGER_ENABLE_Pin, GPIO_PIN_RESET);
      vTaskDelay(500);
    }
    // blink LED to show main is alive
    HAL_GPIO_TogglePin(LD3_GPIO_Port, LD3_Pin);
    // create Main status message and add to queue
    CanTxMsgTypeDef tx;
    tx.IDE = CAN_ID_STD;
    tx.RTR = CAN_RTR_DATA;
    tx.StdId = ID_MAIN;
    tx.DLC = 3;
    tx.Data[0] = car.state;
    tx.Data[1] = (car.pedalbox.apps_state_imp
    		| car.pedalbox.apps_state_brake_plaus
				| car.pedalbox.apps_state_eor
				| car.pedalbox.apps_state_timeout);
    tx.Data[2] = 0;

    if (HAL_GPIO_ReadPin(P_AIR_STATUS_GPIO_Port, P_AIR_STATUS_Pin) != (GPIO_PinState) PC_COMPLETE) {
      HAL_GPIO_TogglePin(LD5_GPIO_Port, LD5_Pin);
      tx.Data[2] = 1;
    }
    xQueueSendToBack(car.vcan.q_tx, &tx, 100);


    vTaskDelayUntil(&last_wake, HEARTBEAT_PERIOD);
  }
  // if returns kill task
  vTaskDelete(NULL);
}


// @authors: Ben Ng
//           Chris Fallon
// @brief: initialize all queues and tasks
void initRTOSObjects() {
  /* Create Queues */
  // TODO create wheel speed queue
  car.dcan.q_rx =       xQueueCreate(QUEUE_SIZE_RXCAN_1, sizeof(CanRxMsgTypeDef));
  car.dcan.q_tx =       xQueueCreate(QUEUE_SIZE_TXCAN_1, sizeof(CanTxMsgTypeDef));
  car.vcan.q_rx =       xQueueCreate(QUEUE_SIZE_RXCAN_2, sizeof(CanRxMsgTypeDef));
  car.dcan.q_tx =       xQueueCreate(QUEUE_SIZE_TXCAN_2, sizeof(CanTxMsgTypeDef));
  car.pedalbox.pb_msg_q =   xQueueCreate(QUEUE_SIZE_PEDALBOXMSG, sizeof(Pedalbox_msg_t));

  /* Create Tasks */
  //todo optimize stack depths http://www.freertos.org/FAQMem.html#StackSize
  xTaskCreate(taskPedalBoxMsgHandler, "PedalBoxMsgHandler", 256, NULL, 1, NULL);
  xTaskCreate(taskCarMainRoutine, "CarMain", 256, NULL, 1, NULL);
  xTaskCreate(taskTX_DCAN, "TX CAN DCAN", 256, NULL, 1, NULL);
  xTaskCreate(taskTX_VCAN, "TX CAN VCAN", 256, NULL, 1, NULL);
  xTaskCreate(taskRXCANProcess, "RX CAN", 256, NULL, 1, NULL);
  xTaskCreate(taskHeartbeat, "Heartbeat", 256, NULL, 1, NULL);
}


// @funcname soundBuzzer
// @return none
void soundBuzzer(int time_ms) 
{
	HAL_GPIO_WritePin(BUZZER_GPIO_Port, BUZZER_Pin, GPIO_PIN_SET); //turn on buzzer
	vTaskDelay((uint32_t) time_ms / portTICK_RATE_MS);
	HAL_GPIO_WritePin(BUZZER_GPIO_Port, BUZZER_Pin, GPIO_PIN_RESET); //turn off buzzer
}


void prchg_led_enbl(uint8_t val)
{
	CanTxMsgTypeDef tx;
	tx.RTR = CAN_RTR_DATA;
	tx.IDE = CAN_ID_STD;
	tx.StdId = MAIN_ACK_ID;
	tx.DLC = 1;
	tx.Data[0] = val;

	xQueueSendToBack(car.dcan.q_tx, &tx, 100);
}


void taskCarMainRoutine()
{
  //Initializations for Traction Control
  uint32_t last_time_tc = 0;
  uint16_t int_term_tc = 0;
  uint16_t prev_trq_tc = 0;
  TickType_t current_tick_time;

  while (1)
  {
      current_tick_time = xTaskGetTickCount();
      uint32_t current_time_ms = current_tick_time / portTICK_PERIOD_MS;
      int16_t torque_to_send = 0;
      uint8_t pc_low = 0;
      //do this no matter what state.
      //get current time in ms
      HAL_GPIO_TogglePin(LD6_GPIO_Port, LD6_Pin);

			//check that precharge is on, send CAN to dash precharge led
//		  if (HAL_GPIO_ReadPin(P_AIR_STATUS_GPIO_Port, P_AIR_STATUS_Pin) != (GPIO_PinState) PC_COMPLETE)
//		  {
//		  	pchg_led_enbl(0);
//		  }
//		  else
//		  {
//		  	pchg_led_enbl(1);
//		  }

      //always active block
      //Brake
      //check if brake level is greater than the threshold level
      if (car.brake >= BRAKE_PRESSED_THRESHOLD)
      {
        //brake is presssed
        carSetBrakeLight(BRAKE_LIGHT_ON);  //turn on brake light
        //EV 2.5, check if the throttle level is greater than 25% while brakes are on
        if (car.throttle_acc > APPS_BP_PLAUS_THRESHOLD)
        {
          //set apps-brake pedal plausibility error
          car.pedalbox.apps_state_brake_plaus = PEDALBOX_STATUS_ERROR;
        }
      }
      else
      {
        //brake is not pressed
        carSetBrakeLight(BRAKE_LIGHT_OFF);  //turn off brake light
      }

      //state dependent block
      if (car.state == CAR_STATE_INIT)
      {
        disableMotorController();
        //assert these pins always
        HAL_GPIO_WritePin(SDC_CTRL_GPIO_Port, SDC_CTRL_Pin, GPIO_PIN_SET); //close SDC
      }
      else if (car.state == CAR_STATE_PREREADY2DRIVE)
      {

        HAL_GPIO_WritePin(DCDC_ENABLE_GPIO_Port, DCDC_ENABLE_Pin, GPIO_PIN_RESET); //enable the DCDC's
        vTaskDelay(500); //account for the DCDC delay turn on

        HAL_GPIO_WritePin(PUMP_GPIO_Port, PUMP_Pin, GPIO_PIN_SET); //turn on pump
        enableMotorController();
        car.state = CAR_STATE_READY2DRIVE;  //car is started
//        HAL_GPIO_WritePin(BATT_FAN_GPIO_Port, BATT_FAN_Pin, GPIO_PIN_SET);
        soundBuzzer(BUZZER_DELAY); //turn buzzer on for 2 seconds

      }
      else if (car.state == CAR_STATE_READY2DRIVE)
      {
        //confirm the PC is not broken
        if (HAL_GPIO_ReadPin(P_AIR_STATUS_GPIO_Port, P_AIR_STATUS_Pin) != (GPIO_PinState) PC_COMPLETE
        		&& !pc_low)
        {
        	pc_low = 1;
        }
        else if (HAL_GPIO_ReadPin(P_AIR_STATUS_GPIO_Port, P_AIR_STATUS_Pin) != (GPIO_PinState) PC_COMPLETE
        		&& pc_low)
        {
					car.state = CAR_STATE_RESET;  //car is started
        }
        else
        {
        	pc_low = 0;
          //assert these pins during r2d
          //check if the age of the pedalbox message is greater than the timeout
          //T.6.2.10 b
          if (current_time_ms - car.pb_msg_rx_time > PEDALBOX_TIMEOUT)
          {
            torque_to_send = 0;
            car.pedalbox.apps_state_timeout = PEDALBOX_STATUS_ERROR;
          }
          else
          {
            car.pedalbox.apps_state_timeout = PEDALBOX_STATUS_NO_ERROR;
          }

          if (car.pedalbox.apps_state_brake_plaus == PEDALBOX_STATUS_NO_ERROR &&
              car.pedalbox.apps_state_eor == PEDALBOX_STATUS_NO_ERROR &&
              car.pedalbox.apps_state_imp == PEDALBOX_STATUS_NO_ERROR &&
              car.pedalbox.apps_state_timeout == PEDALBOX_STATUS_NO_ERROR)
          {
            torque_to_send = car.throttle_acc; //gets average
          }
          else if (car.pedalbox.apps_state_brake_plaus == PEDALBOX_STATUS_ERROR)
          {
            //nothing
          }

          if (car.power_limit.enabled == true) {
            torque_to_send = limit_torque(torque_to_send);
          }

          if (car.tract_cont_en == true) {
            torque_to_send = TractionControl(current_time_ms, &last_time_tc, torque_to_send, &int_term_tc, &prev_trq_tc);
          }

          mcCmdTorqueFake(torque_to_send);
          mcCmdTorque(torque_to_send);  //command the MC to move the motor
        }
      }
      else if (car.state == CAR_STATE_RESET)
      {
        HAL_GPIO_WritePin(DCDC_ENABLE_GPIO_Port, DCDC_ENABLE_Pin, GPIO_PIN_SET); //disable the DCDC's
        HAL_GPIO_WritePin(PUMP_GPIO_Port, PUMP_Pin, GPIO_PIN_RESET);
        disableMotorController();
        HAL_GPIO_WritePin(BATT_FAN_GPIO_Port, BATT_FAN_Pin, GPIO_PIN_RESET);
        car.state = CAR_STATE_INIT;
      }
      else if (car.state == CAR_STATE_RECOVER)
      {
        //TODO:this state will need to be looked at since RINEHART is a little different
        disableMotorController();
        vTaskDelay((uint32_t) 500 / portTICK_RATE_MS);
        enableMotorController();
        car.state = CAR_STATE_READY2DRIVE;
      }
      //wait until Constant 50 Hz rate
      vTaskDelayUntil(&current_tick_time, PERIOD_TORQUE_SEND);
    }
}


// TODO replace constants with enums
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
