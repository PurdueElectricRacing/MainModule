/*
 * PedalBox.h
 *
 *  Created on: Jan 12, 2017
 *      Author: ben
 */

#ifndef PEDALBOX_H_
#define PEDALBOX_H_

#include <stdbool.h>
#include "main.h"
#include "cmsis_os.h"
#include "stm32f4xx_hal.h"

#define THROTTLE_SENSOR_PLAUS_THRESH  0.1f
#define BRAKE_PRESSED_THRESHOLD       0.30f
#define APPS_BP_PLAUS_RESET_THRESHOLD 0.05f
#define APPS_BP_PLAUS_THRESHOLD       0.25f
#define THROTTLE_LOWER_BOUND          0.15f

//pedalbox defines //todo not sure if better to send whole frame or just pbmsg.
#define PEDALBOX1_FILTER                0 //filter number corresponding to the PEDALBOX1 message
#define PEDALBOX1_THROT1_7_0_BYTE       1
#define PEDALBOX1_THROT1_7_0_OFFSET     0
#define PEDALBOX1_THROT1_7_0_MASK       0b11111111
#define PEDALBOX1_THROT1_11_8_BYTE      0
#define PEDALBOX1_THROT1_11_8_OFFSET    0
#define PEDALBOX1_THROT1_11_8_MASK      0b00001111
#define PEDALBOX1_THROT2_7_0_BYTE       3
#define PEDALBOX1_THROT2_7_0_OFFSET     0
#define PEDALBOX1_THROT2_7_0_MASK       0b11111111
#define PEDALBOX1_THROT2_11_8_BYTE      2
#define PEDALBOX1_THROT2_11_8_OFFSET    0
#define PEDALBOX1_THROT2_11_8_MASK      0b00001111
//brake
#define PEDALBOX1_BRAKE1_7_0_BYTE       5
#define PEDALBOX1_BRAKE1_7_0_OFFSET     0
#define PEDALBOX1_BRAKE1_7_0_MASK       0b11111111
#define PEDALBOX1_BRAKE1_11_8_BYTE      4
#define PEDALBOX1_BRAKE1_11_8_OFFSET    0
#define PEDALBOX1_BRAKE1_11_8_MASK      0b00001111
#define PEDALBOX1_BRAKE2_7_0_BYTE       7
#define PEDALBOX1_BRAKE2_7_0_OFFSET     0
#define PEDALBOX1_BRAKE2_7_0_MASK       0b11111111
#define PEDALBOX1_BRAKE2_11_8_BYTE      6
#define PEDALBOX1_BRAKE2_11_8_OFFSET    0
#define PEDALBOX1_BRAKE2_11_8_MASK      0b00001111

#define PEDALBOX1_EOR_BYTE            3
#define PEDALBOX1_EOR_OFFSET          0
#define PEDALBOX1_EOR_MASK            0b00000001
#define PEDALBOX1_IMP_BYTE            3
#define PEDALBOX1_IMP_OFFSET          1
#define PEDALBOX1_IMP_MASK            0b00000010


typedef enum 
{
  THROTTLE_1_MIN = 0x0FFF,
  THROTTLE_1_MAX = 0x0490,
  
  THROTTLE_2_MIN = 0x0EA0,
  THROTTLE_2_MAX = 0x0320,
  
} Throttle_Thresholds_t;

typedef enum
{
  BRAKE_1_MIN = 0x027C,
  BRAKE_1_MAX = 0x0900,

  BRAKE_2_MIN = 0x026F,
  BRAKE_2_MAX = 0x0900,
} Brake_Thresholds_t;

typedef enum {
  PEDALBOX_STATUS_NO_ERROR           = 0,
  PEDALBOX_STATUS_ERROR              = 0b00000001, //generic error
  PEDALBOX_STATUS_ERROR_EOR          = 0b00000010, // encoder out of range
  PEDALBOX_STATUS_ERROR_APPSIMP      = 0b00000100,  //APPS Implausibility error, EV 2.3.5,
  PEDALBOX_STATUS_ERROR_APPSIMP_PREV = 0b00001000,  //APPS Implausibility error, provisional (before it has lasted .1 second)
  PEDALBOX_STATUS_ERROR_BPIMP        = 0b00010000,  //brake pedal implaus //EV 2.5.1,
} Pedalbox_status_t;


// Structure to hold data passed through the queue to pedalBoxMsgHandler
typedef struct _pedalbox_msg
{
  uint32_t         throttle1_raw;    // raw throttle data from pedalbox
  uint32_t         throttle2_raw;
  uint32_t         brake1_raw;
  uint32_t         brake2_raw;
  
} Pedalbox_msg_t;

typedef struct
{
  uint16_t        count;
  uint16_t        period_ms;
  Pedalbox_msg_t      msg;
} GeneratePedalboxMessages_t;


typedef struct 
{
  Pedalbox_status_t   apps_state_imp;   //the last pedalbox message imp sate
  Pedalbox_status_t   apps_state_brake_plaus;        //apps-brake plausibility status
  Pedalbox_status_t   apps_state_eor;       //apps-brake encoder out of range
  Pedalbox_status_t   apps_state_timeout;   
  QueueHandle_t pb_msg_q;
  uint32_t msg_rx_time;   
} PedalBox_t;

void pedalbox_init(volatile PedalBox_t * pb, uint16_t q_size);
void processPedalboxFrame(uint8_t * Data, volatile PedalBox_t * pedalbox);
#endif /* PEDALBOX_H_ */
