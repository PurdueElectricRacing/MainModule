/***************************************************************************
*
*     File Information
*
*     Name of File: car.c
*
*     Authors (Include Email):
*       1. Ben Ng				xbenng@gmail.com
*
*     File dependents: (header files, flow charts, referenced documentation)
*       1. car.h
*
*     File Description:
*     	Functions to control the physical car
*
***************************************************************************/

#include "car.h"

void carSetBrakeLight(Brake_light_status_t status)
/***************************************************************************
*
*     Function Information
*
*     Name of Function: setBrakeLight
*
*     Programmer's Name: Ben Ng xbenng@gmail.com
*
*     Function Return Type: void
*
*     Parameters (list data type, name, and comment one per line):
*       1. Brake_light_status_t status, value to write to GPIO pin
*
*      Global Dependents:
*
*     Function Description:
*			turns brakelight on or off
***************************************************************************/
{
	HAL_GPIO_WritePin(BRAKE_LIGHT_GPIO_Port, BRAKE_LIGHT_Pin, status);
}


void carInit() {
	car.state = CAR_STATE_INIT;
	car.pb_mode = PEDALBOX_MODE_DIGITAL;
	car.throttle_acc = 0;
	car.brake = 0;
	car.phdcan = &hcan1;
	car.phvcan = &hcan2;
	car.calibrate_flag = CALIBRATE_NONE;
	car.throttle1_min = 0x0f90;
	car.throttle1_max = 0x07e0;
	car.throttle2_min = 0x0ed0;
	car.throttle2_max = 0x06c0;
	car.brake1_min = 0x027c;
	car.brake1_max = 0x0900;
	car.brake2_min = 0x026f;
	car.brake2_max = 0x0900;
	car.pb_msg_rx_time = 4294967295;
	car.apps_state_bp_plaus = PEDALBOX_STATUS_NO_ERROR;
	car.apps_state_eor = PEDALBOX_STATUS_NO_ERROR;
	car.apps_state_imp = PEDALBOX_STATUS_NO_ERROR;
	car.apps_state_timeout = PEDALBOX_STATUS_NO_ERROR;

}

void ISR_StartButtonPressed() {
	if (car.state == CAR_STATE_INIT)
	{
		if (car.brake >= BRAKE_PRESSED_THRESHOLD//check if brake is pressed before starting car
			&& HAL_GPIO_ReadPin(P_AIR_STATUS_GPIO_Port, P_AIR_STATUS_Pin) == (GPIO_PinState) PC_COMPLETE) //check if precharge has finished
		car.state = CAR_STATE_PREREADY2DRIVE;
	} else {
		car.state = CAR_STATE_RESET;
	}
}

//TODO Potential MC ping function
//TODO BMS functions

int mainModuleWatchdogTask() {
/***************************************************************************
*
*     Function Information
*
*     Name of Function: mainModuleTimeCheckIdle
*
*     Programmer's Name: Kai Strubel
*     					 Ben Ng			xbenng@gmail.com
*
*     Function Return Type: int
*
*     Parameters (list data type, name, and comment one per line):
*       1.
*
*      Global Dependents:
*	    1.bool launchControl
*		2.float MMPB_TIME time pedal box message handler function was last run
*		3.float MMWM_TIME time wheel module handler function was last run
*		4.float torque
*		5.float currentTime
*
*     Function Description:
*		Checks if wheel module and pedal box are still communicating
*
***************************************************************************/
	while (1) {
		/*
		//check how old the wheel module data is, if its too old, then turn off LC
		if (current_time_ms - MMWM_TIME > LC_THRESHOLD) {
			launchControl = 0;
			//error
		}*/
		vTaskDelay(500);
	}
}

int taskHeartbeat() {
/***************************************************************************
*.
*     Function Information
*
*     Name of Function: heartbeatIdle
*
*     Programmer's Name: Kai Strubel
*
*     Function Return Type: int
*
*     Parameters (list data type, name, and comment one per line):
*       1.
*
*      Global Dependents:
*
*     Function Description:
*		Heart beat to communicate that main module is alive
*
***************************************************************************/
	// write to GPIO
	while (1) {
		HAL_GPIO_TogglePin(SDC_CTRL_GPIO_Port, SDC_CTRL_Pin);
		vTaskDelay(HEARTBEAT_PERIOD);
	}
}

void initRTOSObjects() {
/***************************************************************************
*
*     Function Information
*
*     Name of Function: startTasks
*
*     Programmer's Name: Ben Ng
*
*     Function Return Type: int
*
*     Parameters (list data type, name, and comment one per line):
*       1.
*
*     Global Dependents:
*
*     Function Description:
*		all xTaskCreate calls
*		all xQueueCreate calls
*
***************************************************************************/

	/* Create Queues */

	car.q_rx_dcan = 			xQueueCreate(QUEUE_SIZE_RXCAN_1, sizeof(CanRxMsgTypeDef));
	car.q_tx_dcan = 			xQueueCreate(QUEUE_SIZE_TXCAN_1, sizeof(CanTxMsgTypeDef));
	car.q_rx_vcan = 			xQueueCreate(QUEUE_SIZE_RXCAN_2, sizeof(CanRxMsgTypeDef));
	car.q_tx_vcan = 			xQueueCreate(QUEUE_SIZE_TXCAN_2, sizeof(CanTxMsgTypeDef));
	car.q_pedalboxmsg = 	xQueueCreate(QUEUE_SIZE_PEDALBOXMSG, sizeof(Pedalbox_msg_t));
	car.q_mc_frame = 		xQueueCreate(QUEUE_SIZE_MCFRAME, sizeof(CanRxMsgTypeDef));

	/* Create Tasks */

	//todo optimize stack depths http://www.freertos.org/FAQMem.html#StackSize
	xTaskCreate(taskPedalBoxMsgHandler, "PedalBoxMsgHandler", 256, NULL, 1, NULL);
	xTaskCreate(taskCarMainRoutine, "CarMain", 256 , NULL, 1, NULL);
	xTaskCreate(taskTX_DCAN, "TX CAN DCAN", 256, NULL, 1, NULL);
	xTaskCreate(taskTX_VCAN, "TX CAN VCAN", 256, NULL, 1, NULL);
	xTaskCreate(taskRXCANProcess, "RX CAN", 256, NULL, 1, NULL);
	xTaskCreate(taskBlink, "blink", 256, NULL, 1, NULL);
 }

//extern uint8_t variable;
void taskBlink(void* can)
{
	//vTaskDelay(5000); //TESTING1
	while (1)
	{
		//HAL_GPIO_TogglePin(FRG_RUN_CTRL_GPIO_Port, FRG_RUN_CTRL_Pin);
		HAL_GPIO_TogglePin(LD3_GPIO_Port, LD3_Pin);

		CanTxMsgTypeDef tx;

		tx.IDE = CAN_ID_STD;
		tx.RTR = CAN_RTR_DATA;
		tx.StdId = 0x200;
		tx.DLC = 1;
		tx.Data[0] = 0;
		switch (car.state)
		{
		case CAR_STATE_INIT :
			tx.Data[0] |= 0b00000000;
			break;
		case CAR_STATE_PREREADY2DRIVE:
			tx.Data[0] |= 0b00000001;
			break;
		case CAR_STATE_READY2DRIVE :
			tx.Data[0] |= 0b00000010;
			break;
		case CAR_STATE_RESET :
			tx.Data[0] |= 0b00000011;
			break;
		case CAR_STATE_ERROR :
			tx.Data[0] |=  0b00000100;
			break;
		case CAR_STATE_RECOVER :
			tx.Data[0] |= 0b00000101;
		}
		if (car.apps_state_imp == PEDALBOX_STATUS_ERROR)
		{
			tx.Data[0] |= 0b00010000;
		}
		if (car.apps_state_bp_plaus == PEDALBOX_STATUS_ERROR)
		{
			tx.Data[0] |= 0b00100000;
		}
		if (car.apps_state_eor == PEDALBOX_STATUS_ERROR)
		{
			tx.Data[0] |= 0b01000000;
		}
		if (car.apps_state_timeout == PEDALBOX_STATUS_ERROR)
		{
			tx.Data[0] |= 0b10000000;
		}
		if(!HAL_GPIO_ReadPin(P_AIR_STATUS_GPIO_Port,P_AIR_STATUS_Pin))
		{
			HAL_GPIO_TogglePin(LD5_GPIO_Port, LD5_Pin);
			tx.Data[0] |= 0b00001000;
		}

		xQueueSendToBack(car.q_tx_dcan, &tx, 100);

		//		//req regid 40
		//mcCmdTransmissionRequestSingle(0x40);
		//HAL_CAN_Receive_IT(&hcan1, 0);
		vTaskDelay(250 / portTICK_RATE_MS);
	}
}


void taskSoundBuzzer(int* time_ms) {
/***************************************************************************
*
*     Function Information
*
*     Name of Function: taskSoundBuzzer
*
*     Programmer's Name: Ben Ng
*
*     Function Return Type: void
*
*     Parameters (list data type, name, and comment one per line):
*       1.
*
*     Global Dependents:
*
*     Function Description:
*		ready to drive sound task
*
***************************************************************************/
	while (1) {
		HAL_GPIO_WritePin(BUZZER_GPIO_Port, BUZZER_Pin, GPIO_PIN_SET); //turn on buzzer
		//enable FRG/RUN 0.5s after RFE.
		vTaskDelay((uint32_t) time_ms / portTICK_RATE_MS);
		HAL_GPIO_WritePin(BUZZER_GPIO_Port, BUZZER_Pin, GPIO_PIN_RESET); //turn off buzzer
		vTaskDelete(NULL);
	}
}




void taskCarMainRoutine() {
	//Initializations for Traction Control
	uint32_t last_time_tc = 0;
	uint16_t int_term_tc = 0;
	uint16_t prev_trq_tc = 0;

	while (1)
	{
		//do this no matter what state.
		//get current time in ms
		TickType_t current_tick_time = xTaskGetTickCount();
		uint32_t current_time_ms = current_tick_time / portTICK_PERIOD_MS;

		uint16_t torque_to_send = 0;
		//always active block
		//Brake
		//check if brake level is greater than the threshold level
		if (car.brake >= BRAKE_PRESSED_THRESHOLD) {
			//brake is presssed
			carSetBrakeLight(BRAKE_LIGHT_ON);  //turn on brake light
		} else {
			//brake is not pressed
			carSetBrakeLight(BRAKE_LIGHT_OFF);  //turn off brake light
		}

		if (HAL_GPIO_ReadPin(P_AIR_STATUS_GPIO_Port, P_AIR_STATUS_Pin) == GPIO_PIN_SET && car.state == CAR_STATE_READY2DRIVE)
		{
			car.state = CAR_STATE_RESET;
		}


			//debugging only not required (covered in taskBlink)
//		CanTxMsgTypeDef tx;
//		tx.StdId = ID_PEDALBOX_ERRORS;
//		tx.Data[0] = car.apps_state_bp_plaus;
//		tx.Data[1] = car.apps_state_eor;
//		tx.Data[2] = car.apps_state_imp;
//		tx.Data[3] = car.apps_state_timeout;
//		tx.DLC = 4;
//		tx.IDE = CAN_ID_STD;
//		tx.RTR = CAN_RTR_DATA;
//		xQueueSendToBack(car.q_tx_dcan, &tx, 100);

		//state dependent block
		if (car.state == CAR_STATE_INIT)
		{
			disableMotor();

			//assert these pins always
			HAL_GPIO_WritePin(SDC_CTRL_GPIO_Port, SDC_CTRL_Pin, GPIO_PIN_SET); //close SDC
		}
		else if (car.state == CAR_STATE_PREREADY2DRIVE)
		{

			HAL_GPIO_WritePin(PUMP_GPIO_Port, PUMP_Pin, GPIO_PIN_SET); //turn on pump
			enableMotorController();
			//turn on buzzer
			HAL_GPIO_WritePin(BUZZER_GPIO_Port, BUZZER_Pin, GPIO_PIN_SET); //turn on buzzer

			vTaskDelay((uint32_t) 2000 / portTICK_RATE_MS);
			HAL_GPIO_WritePin(BUZZER_GPIO_Port, BUZZER_Pin, GPIO_PIN_RESET); //turn off buzzer			car.state = CAR_STATE_READY2DRIVE;  //car is started
			HAL_GPIO_WritePin(BATT_FAN_GPIO_Port, BATT_FAN_Pin, GPIO_PIN_SET);
			car.state = CAR_STATE_READY2DRIVE;

		}
		else if (car.state == CAR_STATE_READY2DRIVE)
		{
			//confirm the PC is not broken
			HAL_GPIO_TogglePin(LD6_GPIO_Port, LD6_Pin);
			if (HAL_GPIO_ReadPin(P_AIR_STATUS_GPIO_Port, P_AIR_STATUS_Pin) != (GPIO_PinState) PC_COMPLETE)
			{
				car.state = CAR_STATE_RESET;  //car is started
			}
			else
			{
				//assert these pins during r2d

				//check if the age of the pedalbox message is greater than the timeout
				if (current_time_ms - car.pb_msg_rx_time > PEDALBOX_TIMEOUT)
				{
					torque_to_send = 0;
					car.apps_state_timeout = PEDALBOX_STATUS_ERROR;
					//todo send a CAN message to dash?
				}
				else
				{
					car.apps_state_timeout = PEDALBOX_STATUS_NO_ERROR;
				}
				if (car.apps_state_bp_plaus == PEDALBOX_STATUS_NO_ERROR &&
					car.apps_state_eor == PEDALBOX_STATUS_NO_ERROR &&
					car.apps_state_imp == PEDALBOX_STATUS_NO_ERROR &&
					car.apps_state_timeout == PEDALBOX_STATUS_NO_ERROR)
				{
					torque_to_send = car.throttle_acc; //gets average
				} else if (car.apps_state_bp_plaus == PEDALBOX_STATUS_ERROR) {

				}
			}
		}
		else if (car.state == CAR_STATE_ERROR)
		{
			//disableMotor();
		}
		else if (car.state == CAR_STATE_RESET)
		{
			HAL_GPIO_WritePin(PUMP_GPIO_Port,PUMP_Pin,GPIO_PIN_RESET);
			HAL_GPIO_WritePin(RFE_GPIO_Port,RFE_Pin,GPIO_PIN_RESET);
			HAL_GPIO_WritePin(FRG_RUN_GPIO_Port,FRG_RUN_Pin,GPIO_PIN_RESET);
			HAL_GPIO_WritePin(BATT_FAN_GPIO_Port,BATT_FAN_Pin,GPIO_PIN_RESET);
			car.state = CAR_STATE_INIT;

		}
		else if (car.state == CAR_STATE_RECOVER)
		{
			HAL_GPIO_WritePin(RFE_GPIO_Port,RFE_Pin,GPIO_PIN_RESET);
			HAL_GPIO_WritePin(FRG_RUN_GPIO_Port,FRG_RUN_Pin,GPIO_PIN_RESET);
			vTaskDelay((uint32_t) 500 / portTICK_RATE_MS);
			enableMotorController();
			car.state = CAR_STATE_READY2DRIVE;
		}
		// calculate
//			calcTorqueLimit = (80000 / (actualDC * 10 * actualV * 10)); //(DCLimit / (actualDC * 10)) * actualTorque;
//			if(torque_to_send/MAX_THROTTLE_LEVEL > calcTorqueLimit)
//			{
//				torque_to_send = calcTorqueLimit * torque_to_send;
//			}

		//Traction_Control
		//Assume toggle for traction control
		uint8_t toggle_tc = 1;

		if(toggle_tc == ENABLED){
			torque_to_send = TractionControl(current_time_ms, &last_time_tc, torque_to_send, &int_term_tc, &prev_trq_tc);
		}

		//mcCmdTorqueFake(car.throttle_acc);
		mcCmdTorque(torque_to_send);  //command the MC to move the motor


		//wait until Constant 10 Hz rate
		vTaskDelayUntil(&current_tick_time, PERIOD_TORQUE_SEND);

	}

}
