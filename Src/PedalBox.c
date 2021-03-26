/*
 * PedalBox.c
 *
 *  Created on: Jan 12, 2017
 *      Author: ben
 */

#include "PedalBox.h"
#include "car.h"
#include "stdlib.h"
#include "stdbool.h"
#include <math.h>


extern volatile Car_t car;

bool pedalbox_has_error(PedalBox_t * pb)
{
  if (pb->apps_state_brake_plaus != PEDALBOX_STATUS_NO_ERROR ||
      pb->apps_state_eor != PEDALBOX_STATUS_NO_ERROR ||
      pb->apps_state_imp != PEDALBOX_STATUS_NO_ERROR ||
      pb->apps_state_timeout != PEDALBOX_STATUS_NO_ERROR)
  {
    return true;
  }
  return false;
}

// @author: Chris Fallon
// @brief: initialize all pedal box errors to no error;
void pedalbox_init(volatile PedalBox_t * pb, uint16_t q_size)
{
  pb->apps_state_brake_plaus = PEDALBOX_STATUS_NO_ERROR;
  pb->apps_state_eor = PEDALBOX_STATUS_NO_ERROR;
  pb->apps_state_imp = PEDALBOX_STATUS_NO_ERROR;
  pb->apps_state_timeout = PEDALBOX_STATUS_NO_ERROR;
  pb->pb_msg_q = xQueueCreate(q_size, sizeof(Pedalbox_msg_t));
  pb->msg_rx_time = UINT32_MAX;
}

void processPedalboxFrame(uint8_t * Data, PedalBox_t * pedalbox)
{
	Pedalbox_msg_t pedalboxmsg;

	///////////SCRUB DATA the from the CAN frame//////////////
	//mask then shift the throttle value data

	//build the data
	pedalboxmsg.throttle1_raw = (( ((uint16_t) Data[0]) & 0xFF) << 8) | Data[1];

	pedalboxmsg.throttle2_raw = (( ((uint16_t) Data[2]) & 0xFF) << 8) | Data[3];

	pedalboxmsg.brake1_raw = (( ((uint16_t) Data[4]) & 0xFF) << 8) | Data[5];

	pedalboxmsg.brake2_raw = (( ((uint16_t) Data[6]) & 0xFF) << 8) | Data[7];

	//send to pedalboxmsg to queue

	xQueueSendToBack(pedalbox->pb_msg_q, &pedalboxmsg, 5);
}

// @programmers:  Kai Strubel
//                Ben Ng      xbenng@gmail.com
//                Matt Flanagan
//                Chris Fallon
// @brief: Takes message from pedal box, runs safetly check, sets throttle.
//          Designed this way so pedalboxmsg's can be generated multiple ways,
void taskPedalBoxMsgHandler(void * params) {
  
  Pedalbox_msg_t pedalboxmsg;   //struct to store pedalbox msg
  static uint32_t t1min, t1max, t2min, t2max, b1min, b1max, b2min, b2max;   // Declared static for easy init to 0

  t1min = t2min = b1min = b2min = 0xffffffff;

  while (1) 
  {
  	BaseType_t queued = xQueuePeek(car.pedalbox.pb_msg_q, &pedalboxmsg, 5);
    if (queued == pdTRUE)
    {
        xQueueReceive(car.pedalbox.pb_msg_q, &pedalboxmsg, 5);
      //get current time in ms
      uint32_t current_time_ms = xTaskGetTickCount() / portTICK_PERIOD_MS;
      // update time stamp, indicates when a pedalbox message was last received
      car.pedalbox.msg_rx_time = current_time_ms;
      
      /////////////AUTO-RANGING///////////////
      if (car.state == CAR_STATE_INIT)
      {
        t1min = pedalboxmsg.throttle1_raw < t1min ? pedalboxmsg.throttle1_raw : t1min;
        t2min = pedalboxmsg.throttle2_raw < t2min ? pedalboxmsg.throttle2_raw : t2min;
        b1min = pedalboxmsg.brake1_raw < b1min ? pedalboxmsg.brake1_raw : b1min;
        b2min = pedalboxmsg.brake2_raw < b2min ? pedalboxmsg.brake2_raw : b2min;

        t1max = pedalboxmsg.throttle1_raw > t1max ? pedalboxmsg.throttle1_raw : t1max;
        t2max = pedalboxmsg.throttle2_raw > t2max ? pedalboxmsg.throttle2_raw : t2max;
        b1max = pedalboxmsg.brake1_raw > b1max ? pedalboxmsg.brake1_raw : b1max;
        b2max = pedalboxmsg.brake2_raw > b2max ? pedalboxmsg.brake2_raw : b2max;
      }
      else
      {
        if (t1max - t1min < 500 || t2max - t2min)
        {
            t1max = 1;
            t1min = 1;
            t2max = 1;
            t2min = 1;
        }
        if (b1max - b1min < 500 || b2max - b2min)
        {
            b1max = 1;
            b1min = 1;
            b2max = 1;
            b2min = 1;
        }
      }

      /////////////PROCESS DATA///////////////    
      //value 0-1, throttle 1 calibrated between min and max  
      float throttle1_cal = ((float)(abs(pedalboxmsg.throttle1_raw - t1min))) / (t1max - t1min);
      //value 0-1, throttle 2 calibrated between min and max
      float throttle2_cal = ((float)(abs(pedalboxmsg.throttle2_raw - t2min))) / (t2max - t2min);
      //value 0-1, brake 1 calibrated between min and max  
      float brake1_cal   = ((float)(abs(pedalboxmsg.brake1_raw - b1min))) / (b1max - b1min);
      //value 0-1, brake 2 calibrated between min and max 
      float brake2_cal   = ((float)(abs(pedalboxmsg.brake2_raw - b2min))) / (b2max - b2min);

      float throttle_avg = (throttle1_cal + throttle2_cal) / 2.0;
      float brake_avg    = (brake1_cal + brake2_cal) / 2.0;
      uint32_t avg_speed = (car.wheels.RL_rpm + car.wheels.RR_rpm) / 2.0;
      
      // T 6.2.8: Any failure of APPS must be detectable and treated as an implausibility
//      if (pedalboxmsg.throttle1_raw >= THROTTLE_1_MIN
//      		|| pedalboxmsg.throttle1_raw <= THROTTLE_1_MAX
//					|| pedalboxmsg.throttle2_raw >= THROTTLE_2_MIN
//					|| pedalboxmsg.throttle2_raw <= THROTTLE_2_MAX)
//      {
//        car.pedalbox.apps_state_eor = PEDALBOX_STATUS_ERROR_EOR;
//      }
//      else
//      {
//        car.pedalbox.apps_state_eor = PEDALBOX_STATUS_NO_ERROR;
//      }
      

      //T.6.2.3 Implausibility is defined as a deviation of more than 10% pedal travel between the sensors or
      //other failure as defined in this Section T.6.2. 
      //T.6.2.4 If an implausibility occurs between the values of the APPSs and persists for more than
      //100msec, the power to the (IC) electronic throttle / (EV) motor(s) must be immediately shut
      //down completely.
      //(EV only) It is not necessary to completely deactivate the Tractive System, the motor
      //controller(s) shutting down the power to the motor(s) is sufficient. 

      //Motor is shutdown in car.c when a Pedalbox error is seen
//      if (fabs(throttle1_cal - throttle2_cal) > THROTTLE_SENSOR_PLAUS_THRESH)
//      {
//        //if error is persistent
//        if (car.pedalbox.apps_state_imp == PEDALBOX_STATUS_ERROR_APPSIMP_PREV)
//        {
//          //if time between first error and this error >= 100ms
//          if (car.apps_imp_first_time_ms - current_time_ms >= 100)
//          {
//            car.pedalbox.apps_state_imp = PEDALBOX_STATUS_ERROR_APPSIMP;
//          }
//        }
//        else
//        {  //else this is the first message to have an imp error
//          //record the time
//          car.pedalbox.apps_state_imp = PEDALBOX_STATUS_ERROR_APPSIMP_PREV;
//          car.apps_imp_first_time_ms = current_time_ms;
//        }
//      }
//      else
//      {
//        car.pedalbox.apps_state_imp = PEDALBOX_STATUS_NO_ERROR;
//      }
      
      
//    BRAKE PLAUSIBILITY check
      // EV.2.4 APPS / Brake Pedal Plausibility Check
      // EV.2.4.1 The power to the motors must be immediately shut down completely, if the mechanical
      // brakes are actuated and the APPS signals more than 25% pedal travel at the same time. This
      // must be demonstrated when the motor controllers are under load.
      // EV.2.4.2 The motor power shut down must remain active until the APPS signals less than 5% pedal
      // travel, whether the brakes are still actuated or not.
//      if (throttle_avg >= .25 && brake_avg >= BRAKE_PRESSED_THRESHOLD)
//      {
//        car.pedalbox.apps_state_brake_plaus = PEDALBOX_STATUS_ERROR_BPIMP;
//      }
//      else if (throttle_avg <= APPS_BP_PLAUS_RESET_THRESHOLD)
//      { //latch until this condition
//        //EV 2.5.1, reset apps-brake pedal plausibility error only if throttle level is less than the .05
//        car.pedalbox.apps_state_brake_plaus = PEDALBOX_STATUS_NO_ERROR;
//      }
      
      
// set car variables
      car.brake = brake_avg;
      // TODO edit this to not be hardcoded values
      // should be able to remove these parameters with pb filtering

      if (car.throttle_acc == 0 && throttle_avg >= 0.9)
      {
          HAL_GPIO_TogglePin(GPIOD, LD6_Pin);
      }
      if (throttle_avg > THROTTLE_LOWER_BOUND)
      {
        if (throttle_avg >= 0.9)
        {
          car.throttle_acc = MAX_CONTINUOUS_TORQUE / 100;
        }
        else
        {
          //no errors, set throttle to value received from pedalbox
          car.throttle_acc = ((throttle_avg - 0.1) * MAX_CONTINUOUS_TORQUE / 0.8) / 100;
        }

      }
      else
			{
				car.throttle_acc = 0;
			}

#ifdef REGEN
#ifdef BRAKES
      if (avg_speed >= REGEN_CUTOFF_SPEED && brake_avg > 0.2)
      {
        car.throttle_acc = brake_pres_regen_torque(brake_avg, car.bms.pack_soc);
      }
#else
      if (avg_speed >= REGEN_CUTOFF_SPEED && throttle_avg < THROTTLE_LOWER_BOUND)
      {
        car.throttle_acc = throttle_pos_regen_torque(throttle_avg, car.bms.pack_soc);
      }
#endif
#endif
    }
    vTaskDelay(pdMS_TO_TICKS(1));
  }
  
  //if this task breaks from the loop kill it
  vTaskDelete(NULL);
}




