/*
 * PedalBox.h
 *
 *  Created on: Jan 12, 2017
 *      Author: ben
 */

#ifndef PEDALBOX_H_
#define PEDALBOX_H_

#include "stm32f4xx_hal.h"

#define THROTTLE_1_MIN   0x0FFF
#define THROTTLE_1_MAX   0x0490
#define THROTTLE_2_MIN   0x0EA0
#define THROTTLE_2_MAX   0x0320

#define BRAKE_PRESSED_THRESHOLD .3f
#define APPS_BP_PLAUS_RESET_THRESHOLD .05f  //EV 2.5
#define APPS_BP_PLAUS_THRESHOLD .25f //EV 2.5

typedef enum {
  PEDALBOX_STATUS_ERROR = 1, //generic error
  PEDALBOX_STATUS_ERROR_EOR, // encoder out of range
  PEDALBOX_STATUS_ERROR_APPSIMP,  //APPS Implausibility error, EV 2.3.5,
  PEDALBOX_STATUS_ERROR_APPSIMP_PREV,  //APPS Implausibility error, provisional (before it has lasted .1 second)
  PEDALBOX_STATUS_ERROR_BPIMP,  //brake pedal implaus //EV 2.5.1,
  PEDALBOX_STATUS_NO_ERROR = 0
} Pedalbox_status_t;

// Structure to hold data passed through the queue to pedalBoxMsgHandler
typedef struct _pedalbox_msg {
  int         throttle1_raw;    // raw throttle data from pedalbox
  int         throttle2_raw;
  int         brake1_raw;
  int         brake2_raw;
  
} Pedalbox_msg_t;

typedef struct {
  uint16_t        count;
  uint16_t        period_ms;
  Pedalbox_msg_t      msg;
} GeneratePedalboxMessages_t;

#endif /* PEDALBOX_H_ */
