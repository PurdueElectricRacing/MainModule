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
#include "regen.h"


extern volatile Car_t car;

// @author:
// @brief:  Initialize the car struct and begin rtos tasks

void carInit(CAN_HandleTypeDef * vcan, CAN_HandleTypeDef * dcan)
{
	setDCDCEnabled(0);

	car.state = CAR_STATE_INIT;
	car.throttle_acc = 0;
	car.brake = 0;

	car.tract_cont_en = false;

	init_wheel_mod((wheel_module_t *) &car.wheels);

	// Cast away volatility of car struct
	init_can_bus((CAN_Bus_TypeDef *) &car.vcan, vcan, QUEUE_SIZE_RXCAN_1, QUEUE_SIZE_TXCAN_1);
	init_can_bus((CAN_Bus_TypeDef *) &car.dcan, dcan, QUEUE_SIZE_RXCAN_2, QUEUE_SIZE_TXCAN_2);

	// Cast away volatility of car struct
	VCANFilterConfig((CAN_HandleTypeDef *) car.vcan.hcan);
	DCANFilterConfig((CAN_HandleTypeDef *) car.dcan.hcan);

	// Initialize all peripherals
	pedalbox_init((PedalBox_t *) &car.pedalbox, QUEUE_SIZE_PEDALBOXMSG);
	init_bms_struct((BMS_t *) &car.bms);
	init_power_limit(&car.power_limit, false);

	// Create all tasks
	initRTOSObjects();

	for (uint8_t i = 0; i < 7; i++)
	{
		HAL_GPIO_TogglePin(GPIOD, LD4_Pin);
		HAL_Delay(250);
	}
}

// @authors: Ben Ng
//           Chris Fallon
// @brief: initialize all tasks
void initRTOSObjects()
{

	/* Create Tasks */
	if (xTaskCreate(taskPedalBoxMsgHandler, "PedalBoxMsgHandler", DEFAULT_STACK_SIZE, NULL, DEFAULT_PRIORITY, NULL) != pdPASS)
	{
		Error_Handler();
	}
	if (xTaskCreate(taskCarMainRoutine, "CarMain", DEFAULT_STACK_SIZE, NULL, DEFAULT_PRIORITY, NULL) != pdPASS)
	{
		Error_Handler();
	}

	// taskTX_CAN needs a CAN_Bus_TypeDef to function, but FreeRTOS cannot send non-void pointers
	// the task will cast the void * to a CAN_Bus_TypeDef * and then execute using it
	if (xTaskCreate(taskTX_CAN, "TX CAN DCAN", DEFAULT_STACK_SIZE, (void *) &car.dcan, DEFAULT_PRIORITY, NULL) != pdPASS)
	{
		Error_Handler();
	}
	if (xTaskCreate(taskTX_CAN, "TX CAN VCAN", DEFAULT_STACK_SIZE, (void *) &car.vcan, DEFAULT_PRIORITY, NULL) != pdPASS)
	{
		Error_Handler();
	}
	if (xTaskCreate(task_RX_CAN, "RX CAN", DEFAULT_STACK_SIZE, NULL, DEFAULT_PRIORITY, NULL) != pdPASS)
	{
		Error_Handler();
	}
	if (xTaskCreate(taskHeartbeat, "Heartbeat", DEFAULT_STACK_SIZE, NULL, DEFAULT_PRIORITY, NULL) != pdPASS)
	{
		Error_Handler();
	}
	HAL_GPIO_WritePin(GPIOD, LD4_Pin, GPIO_PIN_SET);
}


void ISR_StartButtonPressed()
{
	if (car.state == CAR_STATE_INIT)
	{
		if (HAL_GPIO_ReadPin(P_AIR_STATUS_GPIO_Port, P_AIR_STATUS_Pin) == (GPIO_PinState) PC_COMPLETE) //check if precharge has finished
		{
			car.state = CAR_STATE_PREREADY2DRIVE;

			//send acknowledge message to dashboard
			send_ack(ID_DASHBOARD_ACK, 1, (CAN_Bus_TypeDef *) &car.vcan);
		}
	}
	else {
		car.state = CAR_STATE_RESET;
		send_ack(ID_DASHBOARD_ACK, 2, (CAN_Bus_TypeDef *) &car.vcan);
	}
}

// @authors: Kai Strubel
//           Ben Ng
//           Chris Fallon
// @brief: function to communicte Main is alive
//         also handles enabling charging of LV batteries
void taskHeartbeat(void * params)
{

	TickType_t last_wake;
	while (1)
	{
		last_wake = xTaskGetTickCount();
		emdrive_sync((CAN_Bus_TypeDef *) &car.vcan);

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
		HAL_GPIO_TogglePin(LD5_GPIO_Port, LD5_Pin);
		// create Main status message and add to queue
		CanTxMsgTypeDef tx;
		tx.IDE = CAN_ID_STD;
		tx.RTR = CAN_RTR_DATA;
		tx.StdId = ID_MAIN;
		tx.DLC = 3;
		tx.Data[0] = car.state;
		tx.Data[1] = (car.pedalbox.apps_state_imp | car.pedalbox.apps_state_brake_plaus
				| car.pedalbox.apps_state_eor | car.pedalbox.apps_state_timeout);
		tx.Data[2] = 0;

		// if precharge is complete, turn on orange LED and send a 1 in the second byte of heartbeat message
		if (HAL_GPIO_ReadPin(P_AIR_STATUS_GPIO_Port, P_AIR_STATUS_Pin) == (GPIO_PinState) PC_COMPLETE)
		{
			tx.Data[2] = 1;
			HAL_GPIO_WritePin(LD3_GPIO_Port, LD3_Pin, GPIO_PIN_SET);
		}
		else
		{
			HAL_GPIO_WritePin(LD3_GPIO_Port, LD3_Pin, GPIO_PIN_RESET);
		}

		xQueueSendToBack(car.vcan.q_tx, &tx, 100);
		xQueueSendToBack(car.dcan.q_tx, &tx, 100);


		vTaskDelayUntil(&last_wake, HEARTBEAT_PERIOD);
	}
	// if returns kill task
	vTaskDelete(NULL);
}


void taskCarMainRoutine()
{
	//Initializations for Traction Control
	uint32_t last_time_tc = 0;
	uint16_t int_term_tc = 0;
	uint16_t prev_trq_tc = 0;
	TickType_t current_tick_time;

	while (PER == GREAT)
	{
		current_tick_time = xTaskGetTickCount();
		uint32_t current_time_ms = current_tick_time / portTICK_PERIOD_MS;
		int16_t torque_to_send = 0;

		// *******
		// Do this no matter what state.
		// *******
		if (car.brake >= BRAKE_PRESSED_THRESHOLD)
		{
			// Brake is pressed
			HAL_GPIO_WritePin(BRAKE_LIGHT_GPIO_Port, BRAKE_LIGHT_Pin, GPIO_PIN_SET);

			// EV 2.5, check if the throttle level is greater than 25% while brakes are on
			if (car.throttle_acc > APPS_BP_PLAUS_THRESHOLD)
			{
				car.pedalbox.apps_state_brake_plaus = PEDALBOX_STATUS_ERROR;
			} else {
				car.pedalbox.apps_state_brake_plaus = PEDALBOX_STATUS_NO_ERROR;
			}
		}
		else
		{
			//brake is not pressed
			HAL_GPIO_WritePin(BRAKE_LIGHT_GPIO_Port, BRAKE_LIGHT_Pin, GPIO_PIN_RESET);
		}

		// *******
		// State Dependent Block
		// *******
		if (car.state == CAR_STATE_INIT)
		{
			emdrive_control(EMDRIVE_STOP, (emdrive_t *) &car.emdrive, (CAN_Bus_TypeDef *) &car.vcan);

			// Assert these pins always
			HAL_GPIO_WritePin(SDC_CTRL_GPIO_Port, SDC_CTRL_Pin, GPIO_PIN_SET); // Close SDC
			setDCDCEnabled(0);
		}
		else if (car.state == CAR_STATE_ERROR)
		{
			HAL_GPIO_WritePin(SDC_CTRL_GPIO_Port, SDC_CTRL_Pin, GPIO_PIN_RESET); // Open SDC
			setDCDCEnabled(0);
		}
		else if (car.state == CAR_STATE_PREREADY2DRIVE)
		{
			setDCDCEnabled(1);
			emdrive_control(EMDRIVE_START, (emdrive_t *) &car.emdrive, (CAN_Bus_TypeDef *) &car.vcan);
			car.state = CAR_STATE_READY2DRIVE;  //car is started
			soundBuzzer(BUZZER_DELAY); //turn buzzer on for 2 seconds
		}
		else if (car.state == CAR_STATE_READY2DRIVE)
		{
			// Confirm the PC is not broken, enter reset otherwise
			if (HAL_GPIO_ReadPin(P_AIR_STATUS_GPIO_Port, P_AIR_STATUS_Pin) != (GPIO_PinState) PC_COMPLETE)
			{
				car.state = CAR_STATE_RESET;
			}
			else
			{
				//assert these pins during r2d
				//check if the age of the pedalbox message is greater than the timeout
				//T.6.2.10 b
				if (current_time_ms - car.pedalbox.msg_rx_time > PEDALBOX_TIMEOUT)
				{
					torque_to_send = 0;
					car.pedalbox.apps_state_timeout = PEDALBOX_STATUS_ERROR;
				}
				else
				{
					car.pedalbox.apps_state_timeout = PEDALBOX_STATUS_NO_ERROR;
				}

				if (!pedalbox_has_error((PedalBox_t *) &car.pedalbox))
				{
					torque_to_send = car.throttle_acc; //gets average
				}
				else if (car.pedalbox.apps_state_brake_plaus == PEDALBOX_STATUS_ERROR)
				{
					torque_to_send = 0;
				}

				if (car.power_limit.enabled == true) {
					torque_to_send = limit_torque(torque_to_send, &car.bms, &car.power_limit);
				}

				if (car.tract_cont_en == true) {
					torque_to_send = TractionControl(current_time_ms, &last_time_tc, torque_to_send, &int_term_tc, &prev_trq_tc);
				}

				// Check Temps here
				setFanSpeed(10);

				emdrive_move_the_car_yo(torque_to_send, (CAN_Bus_TypeDef *) &car.vcan);
				//wait until Constant 50 Hz rate
				vTaskDelayUntil(&current_tick_time, PERIOD_TORQUE_SEND);
			}
		}
		else if (car.state == CAR_STATE_RESET)
		{
			emdrive_control(EMDRIVE_STOP, (emdrive_t *) &car.emdrive, (CAN_Bus_TypeDef *) &car.vcan);
			setDCDCEnabled(0);
			car.state = CAR_STATE_INIT;
		}
		else if (car.state == CAR_STATE_RECOVER)
		{
			emdrive_control(EMDRIVE_STOP, (emdrive_t *) &car.emdrive, (CAN_Bus_TypeDef *) &car.vcan);
			setDCDCEnabled(1);
			emdrive_control(EMDRIVE_START, (emdrive_t *) &car.emdrive, (CAN_Bus_TypeDef *) &car.vcan);
			car.state = CAR_STATE_READY2DRIVE;
		}
		vTaskDelayUntil(&current_tick_time, pdMS_TO_TICKS(1));
	}

	vTaskDelete(NULL);
}

// @funcname setDCDCEnabled
// Disables/Enables DCDC's and other high power modules which require it.
void setDCDCEnabled(uint8_t enabled)
{
	if (HAL_GPIO_ReadPin(P_AIR_STATUS_GPIO_Port, P_AIR_STATUS_Pin) == (GPIO_PinState) PC_COMPLETE)
	{
		// Only allow for DCDCs to turn on if we have precharged
		enabled = 0;
	}

	if (!enabled)
	{
		setFanSpeed(0);
	}

	GPIO_PinState active_high = enabled ? GPIO_PIN_SET : GPIO_PIN_RESET;
	uint8_t dcdc_already_enabled = HAL_GPIO_ReadPin(DCDC_ENABLE_GPIO_Port, DCDC_ENABLE_Pin) == GPIO_PIN_SET;

	HAL_GPIO_WritePin(DCDC_ENABLE_GPIO_Port, DCDC_ENABLE_Pin, active_high);
	// Only need to delay if we are turning on the DCDCs
	if (enabled & !dcdc_already_enabled)
		vTaskDelay(500 / portTICK_RATE_MS);

	HAL_GPIO_WritePin(PUMP_GPIO_Port, PUMP_Pin, active_high);
}

// @funcname soundBuzzer
// @return none
void soundBuzzer(int time_ms) 
{
	HAL_GPIO_WritePin(BUZZER_GPIO_Port, BUZZER_Pin, GPIO_PIN_SET); //turn on buzzer
	vTaskDelay((uint32_t) time_ms / portTICK_RATE_MS);
	HAL_GPIO_WritePin(BUZZER_GPIO_Port, BUZZER_Pin, GPIO_PIN_RESET); //turn off buzzer
}

// @funcname setFanSpeed
// @param speed - Value 0-100 % fan duty cycle
void setFanSpeed(uint8_t speed)
{
	// TIM3 reloads at 20kHz, duty cycle
	TIM3->CCR1 = speed > 100 ? 100 : speed;
}
