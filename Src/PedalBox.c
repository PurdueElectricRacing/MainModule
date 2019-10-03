/*
 * PedalBox.c
 *
 *  Created on: Jan 12, 2017
 *      Author: ben
 */

#include "PedalBox.h"
#include "car.h"
#include "stdlib.h"


// @author: Chris Fallon
// @brief: initialize all pedal box errors to no error;
void pedalbox_init(volatile PedalBox_t * pb)
{
  pb->apps_state_brake_plaus = PEDALBOX_STATUS_NO_ERROR;
  pb->apps_state_eor = PEDALBOX_STATUS_NO_ERROR;
  pb->apps_state_imp = PEDALBOX_STATUS_NO_ERROR;
  pb->apps_state_timeout = PEDALBOX_STATUS_NO_ERROR;
}

// @programmers:  Kai Strubel
//                Ben Ng      xbenng@gmail.com
// @brief: Takes message from pedal box, runs safetly check, sets throttle.
//          Designed this way so pedalboxmsg's can be generated multiple ways,
void taskPedalBoxMsgHandler(void * params) {
  
  Pedalbox_msg_t pedalboxmsg;   //struct to store pedalbox msg
  while (1) {
  
    if (xQueueReceive(car.pedalbox.pb_msg_q, &pedalboxmsg, 1000)) {
      //get current time in ms
      uint32_t current_time_ms = xTaskGetTickCount() / portTICK_PERIOD_MS;
      // update time stamp, indicates when a pedalbox message was last received
      car.pb_msg_rx_time = current_time_ms;
      
      /////////////PROCESS DATA///////////////    
      //value 0-1, throttle 1 calibrated between min and max  
      float     throttle1_cal = ((float)(pedalboxmsg.throttle1_raw - THROTTLE_1_MIN)) / (THROTTLE_1_MAX - THROTTLE_1_MIN);  
      //value 0-1, throttle 2 calibrated between min and max
      float     throttle2_cal = ((float)(pedalboxmsg.throttle2_raw - THROTTLE_2_MIN)) / (THROTTLE_2_MAX - THROTTLE_2_MIN);
      //value 0-1, brake 1 calibrated between min and max  
      float     brake1_cal    = ((float)(pedalboxmsg.brake1_raw - BRAKE_1_MIN)) / (BRAKE_1_MAX - BRAKE_1_MIN); 
      //value 0-1, brake 2 calibrated between min and max 
      float     brake2_cal    = ((float)(pedalboxmsg.brake2_raw - BRAKE_2_MIN)) / (BRAKE_2_MAX - BRAKE_2_MIN);  

      float     throttle_avg  = (throttle1_cal + throttle2_cal) / 2.0;
      float     brake_avg     = (brake1_cal + brake2_cal) / 2.0;
      float     avg_spd  = (car.wheel_rpms.RL_rpm + car.wheel_rpms.RR_rpm) / 2.0;
      
      
      // T 6.2.8: Any failure of APPS must be detectable and treated as an implausibility
      if (
        pedalboxmsg.throttle1_raw >= THROTTLE_1_MIN ||
        pedalboxmsg.throttle1_raw <= THROTTLE_1_MAX ||
        pedalboxmsg.throttle2_raw >= THROTTLE_2_MIN ||
        pedalboxmsg.throttle2_raw <= THROTTLE_2_MAX
      )
      {
        car.pedalbox.apps_state_eor = PEDALBOX_STATUS_ERROR_EOR;
      }
      else
      {
        car.pedalbox.apps_state_eor = PEDALBOX_STATUS_NO_ERROR;
      }
      

      //T.6.2.3 Implausibility is defined as a deviation of more than 10% pedal travel between the sensors or
      //other failure as defined in this Section T.6.2. 
      //T.6.2.4 If an implausibility occurs between the values of the APPSs and persists for more than
      //100msec, the power to the (IC) electronic throttle / (EV) motor(s) must be immediately shut
      //down completely.
      //(EV only) It is not necessary to completely deactivate the Tractive System, the motor
      //controller(s) shutting down the power to the motor(s) is sufficient. 

      //Motor is shutdown in car.c when a Pedalbox error is seen
      if (fabs(throttle1_cal - throttle2_cal) > THROTTLE_SENSOR_PLAUS_THRESH) { 
        //if error is persistent
        if (car.pedalbox.apps_state_imp == PEDALBOX_STATUS_ERROR_APPSIMP_PREV)
        { 
          //if time between first error and this error >= 100ms
          if (car.apps_imp_first_time_ms - current_time_ms >= 100)
          {
            car.pedalbox.apps_state_imp = PEDALBOX_STATUS_ERROR_APPSIMP;
          }
        }
        else
        {  //else this is the first message to have an imp error
          //record the time
          car.pedalbox.apps_state_imp = PEDALBOX_STATUS_ERROR_APPSIMP_PREV;
          car.apps_imp_first_time_ms = current_time_ms;
        }
      }
      else
      {
        car.pedalbox.apps_state_imp = PEDALBOX_STATUS_NO_ERROR;
      }
      
      
//    BRAKE PLAUSIBILITY check
      // EV.2.4 APPS / Brake Pedal Plausibility Check
      // EV.2.4.1 The power to the motors must be immediately shut down completely, if the mechanical
      // brakes are actuated and the APPS signals more than 25% pedal travel at the same time. This
      // must be demonstrated when the motor controllers are under load.
      // EV.2.4.2 The motor power shut down must remain active until the APPS signals less than 5% pedal
      // travel, whether the brakes are still actuated or not.
      if (throttle_avg >= .25 && brake_avg >= BRAKE_PRESSED_THRESHOLD)
      {
        car.pedalbox.apps_state_brake_plaus = PEDALBOX_STATUS_ERROR_BPIMP;
      }
      else if (throttle_avg <= APPS_BP_PLAUS_RESET_THRESHOLD)
      { //latch until this condition
        //EV 2.5.1, reset apps-brake pedal plausibility error only if throttle level is less than the .05
        car.pedalbox.apps_state_brake_plaus = PEDALBOX_STATUS_NO_ERROR;
      }
      
      
// set car variables
      car.brake = brake_avg;
      // TODO edit this to not be hardcoded values
      // should be able to remove these parameters with pb filtering
      if (throttle_avg > THROTTLE_LOWER_BOUND)
      {
        if (throttle_avg >= 0.9)
        {
          car.throttle_acc = MAX_CONTINUOUS_TORQUE;
        }
        else
        {
          //no errors, set throttle to value received from pedalbox
          car.throttle_acc = ((throttle_avg - 0.1) * MAX_CONTINUOUS_TORQUE / 0.8);
        }

      }
      else
			{
				car.throttle_acc = 0;
			}

      if (avg_spd >= REGEN_CUTOFF_SPEED && throttle_avg <= THROTTLE_LOWER_BOUND)
      {
				car.throttle_acc = MAX_REGEN_TORQUE;
      }
    }
  }
  
  //if this task breaks from the loop kill it
  vTaskDelete(NULL);
}


