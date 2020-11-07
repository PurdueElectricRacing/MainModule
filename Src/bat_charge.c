/*
 * bat_charge.c
 *
 *  Created on: Sep, 26 2020
 *      Author: Luke Oxley
 */
#include "bat_charge.h"
#include "main.h"
#include "cmsis_os.h"
#include "stm32f4xx_hal.h"

uint8_t g_temp_array[READ_MSG_SIZE];
uint8_t g_read_byte = 0;
uint8_t g_write_data[WRITE_MSG_SIZE + 1];
extern I2C_HandleTypeDef hi2c1;

void init_BQ21062(uint8_t * g_write_data);

void task_manage_charger() 
{
  uint8_t current_fault;
  uint8_t voltage_fault;
  uint8_t temp_fault;
  TickType_t current_tick_time;

  init_BQ21062(g_write_data);

  while(PER == GREAT)
  {
    current_tick_time = xTaskGetTickCount();

    g_read_byte = set_address(STAT0_ADDR, READ_ENABLE);
    HAL_I2C_Master_Receive(&hi2c1, g_read_byte, &g_temp_array[0], READ_MSG_SIZE, 0xFFFF);

    while (hi2c1.State != HAL_I2C_STATE_READY)
    {
        //Wait for the send to stop
    }

    current_fault = GET_BIT(g_temp_array[0], CURRENT_LIM_MASK);

    g_read_byte = set_address(STAT1_ADDR, READ_ENABLE);
    HAL_I2C_Master_Receive(&hi2c1, g_read_byte, &g_temp_array[0], READ_MSG_SIZE, 0xFFFF);

    while (hi2c1.State != HAL_I2C_STATE_READY)
    {
        //Wait for the send to stop
    }

    voltage_fault = GET_BIT(g_temp_array[0], OVERVOLTAGE_MASK);
    temp_fault = GET_BIT(g_temp_array[0], HOT_MASK);

    //TODO Report faults to something else

    vTaskDelayUntil(&current_tick_time, pdMS_TO_TICKS(200));
  }

}

void setChargeEnable(uint8_t enable)
{
    g_write_data[0] = set_address(ICCTRL2_ADDR, WRITE_ENABLE);
  
    if(enable)
    {
      g_write_data[1] = ICCTRL2_DEF | CHRG_ENABLE_MASK;
    }
    else
    {
      g_write_data[1] = ICCTRL2_DEF | CHRG_DISABLE_MASK;
    }

    HAL_I2C_Master_Transmit(&hi2c1, g_write_data[0], &g_write_data[1], WRITE_MSG_SIZE, 0xFFFF);

    while (hi2c1.State != HAL_I2C_STATE_READY)
    {
	  //Wait for the send to stop
    }
}



void init_BQ21062(uint8_t * g_write_data)
{
  //set current charge to 500mA
  //set I_Charge_Range to 1 for 2.5mA step
  //Set I_Charge to CHRG current steps (500mA = 2.5mA * 200)
  uint8_t timeout = 0;
	g_write_data[0] = set_address(PCHRGCTRL_ADDR, WRITE_ENABLE);
  g_write_data[1] = PCHRGCTRL_DEF;

  HAL_I2C_Master_Transmit(&hi2c1, g_write_data[0], &g_write_data[1], WRITE_MSG_SIZE, 0xFFFF);

  for(timeout = 0; hi2c1.State != HAL_I2C_STATE_READY && timeout < WRITE_TIMEOUT; timeout++)
	{
	  //Wait for the send to stop
	}
  if (timeout >= WRITE_TIMEOUT)
	{
		//FAULTED
	}
	//vTaskDelay(WRITE_REQ_WAIT);

	g_write_data[0] = set_address(ICHG_CTRL_ADDR, WRITE_ENABLE);
	g_write_data[1] = (uint8_t) CHRG_CURRENT_STEPS;
	HAL_I2C_Master_Transmit(&hi2c1, g_write_data[0], &g_write_data[1], WRITE_MSG_SIZE, 0xFFFF);

  for(timeout = 0; hi2c1.State != HAL_I2C_STATE_READY && timeout < WRITE_TIMEOUT; timeout++)
	{
	  //Wait for the send to stop
	}
	if (timeout >= WRITE_TIMEOUT)
	{
		//FAULTED
	}
	//vTaskDelay(WRITE_REQ_WAIT);
}
