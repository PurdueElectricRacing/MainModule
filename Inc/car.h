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
#include "PedalBox.h"
#include "stm32f4xx_hal.h"
#include "CANProcess.h"
#include "BMS.h"
#include "motor_controller_functions.h"
#include "power_limiting.h"
#include <math.h>
#include "traction_control.h"
#include <bool.h>

//Can comment/uncomment as required
#define PERCEPIO_TRACE

#define PERIOD_TORQUE_SEND    25
#define HEARTBEAT_PULSEWIDTH  200
#define HEARTBEAT_PERIOD      100
#define PEDALBOX_TIMEOUT      1000
#define POLL_DELAY            50
#define MAX_BRAKE_LEVEL       0xFFF
#define MAX_THROTTLE_LEVEL    1600     //88 Nm
#define LC_THRESHOLD          10      // todo lc threshold DUMMY VALUE
#define LAUNCH_CONTROL_INTERVAL_MS  10
#define MAX_THROTTLE_LEVEL    1600     //160 Nm
#define DONT_CARE             0
#define BUZZER_DELAY          2000

//#define TEST_MC
#define MC_TEST_TORQUE (MAX_THROTTLE_LEVEL / 5)


//rtos parameter defines
#define QUEUE_SIZE_RXCAN_1      16
#define QUEUE_SIZE_RXCAN_2      16
#define QUEUE_SIZE_PEDALBOXMSG  16
#define QUEUE_SIZE_TXCAN_1      10
#define QUEUE_SIZE_TXCAN_2      10
#define QUEUE_SIZE_MCFRAME      3


typedef enum {
  BRAKE_LIGHT_OFF = GPIO_PIN_RESET,
  BRAKE_LIGHT_ON = GPIO_PIN_SET
} Brake_light_status_t;

typedef enum {
  PC_INPROGRESS = GPIO_PIN_SET,
  PC_COMPLETE = GPIO_PIN_RESET
} PC_status_t;

//launch control
typedef enum {
  LC_ACTIVATED,
  LC_DISABLED
} LC_status_t;

typedef enum {
  CAR_STATE_INIT,
  CAR_STATE_PREREADY2DRIVE,
  CAR_STATE_READY2DRIVE,
  CAR_STATE_ERROR,
  CAR_STATE_RESET,
  CAR_STATE_RECOVER
} Car_state_t;


typedef enum {
  CALIBRATE_NONE,
  CALIBRATE_THROTTLE_MIN,
  CALIBRATE_THROTTLE_MAX,
  CALIBRATE_BRAKE_MIN,
  CALIBRATE_BRAKE_MAX
} Calibrate_flag_t;



int BCparam;
int actualTorque0700;
int actualTorque1508;
int DCLimit;
int pedalTorque;
int actualTorque;
int speedActual;
int currentActual;
int commandCurrent;
int dcBusVoltage;
int motorTemperature;
int powerStageTemperature;
int airTemperature;
int actualCurrentLimit;
int errBitMap1;



typedef struct {
  int power_thresh;
  int power_soft_lim;
  int power_hard_lim;
  flag_t power_lim_en;
}power_lim_t;

typedef struct {
  Car_state_t       state;
  uint8_t         errorFlags;
  //calibration values
  int16_t       throttle_acc;       //sum of car's intended throttle messages from pedalbox since last cmd sent to MC
  float         brake;            //car's intended brake position
  uint32_t        pb_msg_rx_time;       //indicates when a pedalbox message was last received
  uint32_t        apps_imp_first_time_ms;   //indicates when the first imp error was received
  Pedalbox_status_t   apps_state_imp;   //the last pedalbox message imp sate
  Pedalbox_status_t   apps_state_bp_plaus;        //apps-brake plausibility status
  Pedalbox_status_t   apps_state_eor;       //apps-brake plausibility status
  Pedalbox_status_t   apps_state_timeout;       //apps-brake plausibility status
  Pedalbox_mode_t     pb_mode;          //determines whether pb will be analog or CAN
  
  LC_status_t         lc_status;
  BMS_t               bms_params;
  power_lim_t         pow_lim;

	//Wheel Speed (Traction Control)
	wheel_speed_t				wheel_rpm;
	bool              traction_en; //parameter to enable and disable traction control

	//RTOS objects, initialized in initRTOSObjects
	QueueHandle_t			q_rx_dcan;
	QueueHandle_t			q_tx_dcan;
	QueueHandle_t			q_rx_vcan;
	QueueHandle_t			q_tx_vcan;
	QueueHandle_t	 		q_pedalboxmsg;
	QueueHandle_t			q_mc_frame;

	CAN_HandleTypeDef *		phdcan;						//pointer to car's CAN peripheral handle
	CAN_HandleTypeDef *		phvcan;


} Car_t;

extern volatile Car_t car;

extern CAN_HandleTypeDef hcan1;
extern CAN_HandleTypeDef hcan2;

//function prototypes
void carSetBrakeLight(Brake_light_status_t status);
void ISR_StartButtonPressed();
void carInit();
void taskPedalBoxMsgHandler();
void taskCarMainRoutine();
int mainModuleWatchdogTask();
void taskHeartbeat();
void initRTOSObjects();
void taskBlink(void* can);
void taskMotorControllerPoll();
void soundBuzzer(int time_ms);


#endif /* CAR_H_ */
