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

uint8_t temp_array[READ_MSG_SIZE];
uint8_t read_byte = 0;
uint8_t write_data[WRITE_MSG_SIZE + 1];
extern I2C_HandleTypeDef hi2c1;

void init_BQ21062(uint8_t * write_data);

void task_manage_charger() 
{
    uint8_t currentF, voltageF, tempF;

    init_BQ21062(write_data);

    while(1)
    {
        read_byte = set_address(STAT0_ADDR, READ_ENABLE);
        HAL_I2C_Master_Receive(&hi2c1, read_byte, &temp_array[0], READ_MSG_SIZE, 0xFFFF);
        while (hi2c1.State != HAL_I2C_STATE_READY)
	    {
	        //Wait for the send to stop
	    }
        currentF = get_bit(temp_array[0], CURRENT_LIM_BIT);

        read_byte = set_address(STAT1_ADDR, READ_ENABLE);
        HAL_I2C_Master_Receive(&hi2c1, read_byte, &temp_array[0], READ_MSG_SIZE, 0xFFFF);
        while (hi2c1.State != HAL_I2C_STATE_READY)
	    {
	        //Wait for the send to stop
	    }
        voltageF = get_bit(temp_array[0], OVERVOLTAGE_BIT);
        tempF = get_bit(temp_array[0], HOT_BIT);

        //TODO Report faults to something else
        
        vTaskDelayUntil(200); 
    }

}

void setChargeEnable(uint8_t enable)
{
    write_data[0] = set_address(ICCTRL2_ADDR, WRITE_ENABLE);
  
    if(enable) write_data[1] = ICCTRL2_DEF || CHRG_ENABLE;
    else write_data[1] = ICCTRL2_DEF || CHRG_DISABLE;
    HAL_I2C_Master_Transmit(&hi2c1, write_data[0], &write_data[1], WRITE_MSG_SIZE, 0xFFFF);

    while (hi2c1.State != HAL_I2C_STATE_READY)
	{
	  //Wait for the send to stop
	}
    return;
}



static void init_BQ21062(uint8_t * write_data)
{
    //set current charge to 500mA
        //set I_Charge_Range to 1 for 2.5mA step
        //Set I_Charge to CHRG current steps (500mA = 2.5mA * 200)
    uint8_t timeout = 0;
	write_data[0] = set_address(PCHRGCTRL_ADDR, WRITE_ENABLE);
    write_data[1] = PCHRGCTRL_DEF;

    HAL_I2C_Master_Transmit(&hi2c1, write_data[0], &write_data[1], WRITE_MSG_SIZE, 0xFFFF);

    while (hi2c1.State != HAL_I2C_STATE_READY && timeout++ < WRITE_TIMEOUT)
	{
	  //Wait for the send to stop
	}
    if (timeout >= WRITE_TIMEOUT)
	{
		//FAULTED
	}
	//vTaskDelay(WRITE_REQ_WAIT);

    timeout = 0;
	write_data[0] = set_address(ICHG_CTRL_ADDR, WRITE_ENABLE);
	write_data[1] = (uint8_t) CHRG_CURRENT_STEPS;
	HAL_I2C_Master_Transmit(&hi2c1, write_data[0], &write_data[1], WRITE_MSG_SIZE, 0xFFFF);
	while (hi2c1.State != HAL_I2C_STATE_READY && timeout++ < WRITE_TIMEOUT)
	{
	  //Wait for the send to stop
	}
	if (timeout >= WRITE_TIMEOUT)
	{
		//FAULTED
	}
	//vTaskDelay(WRITE_REQ_WAIT);
}
