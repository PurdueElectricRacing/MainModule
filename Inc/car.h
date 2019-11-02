/*
 * car.h
 *
 *  Created on: Jan 3, 2017
 *      Author: ben
 */

#ifndef CAR_H_
#define CAR_H_

#include "main.h"
#include "cmsis_os.h"
#include "stm32f4xx_hal.h"
#include <stdbool.h>
#include "imu.h"

#include "BMS.h"
#include "CAN_Bus.h"
#include "CANProcess.h"
#include "wheel_mod.h"

#include "PedalBox.h"
#include "power_limiting.h"
#include "traction_control.h"

//Can comment/uncomment as required
//#define PERCEPIO_TRACE
#define REGEN
//#define BRAKES
#define PERIOD_TORQUE_SEND    pdMS_TO_TICKS(25)
#define HEARTBEAT_PULSEWIDTH  pdMS_TO_TICKS(200)
#define HEARTBEAT_PERIOD      pdMS_TO_TICKS(500)
#define PEDALBOX_TIMEOUT      pdMS_TO_TICKS(1000)
#define POLL_DELAY            pdMS_TO_TICKS(50)
#define BUZZER_DELAY          pdMS_TO_TICKS(2000)
#define MAX_BRAKE_LEVEL       0xFFF
#define BOOST_MODE_TORQUE     2400 //240 Nm NOT SURE IF THIS IS RIGHT
#define MAX_CONTINUOUS_TORQUE 1600 // 125 Nm continuous
#define MAX_REGEN_TORQUE      -35
#define DONT_CARE             0

#define DEFAULT_STACK_SIZE 256 // number of WORDS FreeRTOS will allocate to a task, not bytes.
#define DEFAULT_PRIORITY   1   // default priority for RTOS tasks.

#define VCAN_RX_FIFO CAN_IT_RX_FIFO1_MSG_PENDING
#define DCAN_RX_FIFO CAN_IT_RX_FIFO0_MSG_PENDING

//rtos parameter defines
#define QUEUE_SIZE_RXCAN_1      16
#define QUEUE_SIZE_RXCAN_2      16
#define QUEUE_SIZE_PEDALBOXMSG  16
#define QUEUE_SIZE_TXCAN_1      10
#define QUEUE_SIZE_TXCAN_2      10
#define QUEUE_SIZE_MCFRAME      3

#define REGEN_CUTOFF_SPEED  200



typedef enum {
  BRAKE_LIGHT_OFF = GPIO_PIN_RESET,
  BRAKE_LIGHT_ON = GPIO_PIN_SET
} Brake_light_status_t;

typedef enum {
  PC_INPROGRESS = GPIO_PIN_SET,
  PC_COMPLETE = GPIO_PIN_RESET
} PC_status_t;

typedef enum {
  CAR_STATE_INIT  = 0,
  CAR_STATE_PREREADY2DRIVE = 1,
  CAR_STATE_READY2DRIVE    = 2,
  CAR_STATE_ERROR   = 3,
  CAR_STATE_RESET   = 4,
  CAR_STATE_RECOVER = 5
} Car_state_t;





typedef struct
{
  Car_state_t       state;

  BMS_t bms;
  power_limit_t power_limit;
  CAN_Bus_TypeDef vcan;
  CAN_Bus_TypeDef dcan;
  PedalBox_t pedalbox;
  wheel_module_t wheels;

  uint8_t  errorFlags;
  //calibration values
  int16_t       throttle_acc;       //sum of car's intended throttle messages from pedalbox since last cmd sent to MC
  float         brake;            //car's intended brake position
  uint32_t      apps_imp_first_time_ms;   //indicates when the first imp error was received
  
  bool tract_cont_en;
} Car_t;

extern CAN_HandleTypeDef hcan1;
extern CAN_HandleTypeDef hcan2;

//function prototypes
void carSetBrakeLight(Brake_light_status_t status);
void ISR_StartButtonPressed();
void carInit(CAN_HandleTypeDef * vcan, CAN_HandleTypeDef * dcan);
void taskPedalBoxMsgHandler();
void taskCarMainRoutine();
int mainModuleWatchdogTask();
void taskHeartbeat();
void initRTOSObjects();
void taskMotorControllerPoll();
void soundBuzzer(int time_ms);





#endif /* CAR_H_ */
