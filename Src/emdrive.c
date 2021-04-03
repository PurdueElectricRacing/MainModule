#include "emdrive.h"
#include "car.h"
#include "main.h"
#include <string.h>

// @brief: Init function
// @author: Chris Fallon
void emdrive_init(emdrive_t * drive)
{
  memset(&drive->data, 0, sizeof(emdrive_tpdo_t)); // init all data points to 0
  drive->state = PRE_OPERATION;
  
  drive->err = OK;

  drive->warning = false;
}

// @brief: Function for syncing the motor controller
//         intended to be sent periodically by an RTOS task,
//         but can be used by itself
// @author: Chris Fallon
void emdrive_sync(CAN_Bus_TypeDef * can)
{
  CanTxMsgTypeDef tx;
  tx.IDE = CAN_ID_STD;
  tx.RTR = CAN_RTR_DATA;
  tx.StdId = ID_EMDRIVE_MASTER_SYNC;
  tx.DLC = 1;
  tx.Data[0] = 0;
  xQueueSendToBack(can->q_tx, &tx, 100);
}

// @brief:  Sends SDO (service data object) editing a value in
//          the dictionary
// @author: Dawson Moore
static void sendSDO(emdrive_sdo_size_t size, uint16_t od, uint8_t sub_od, uint32_t data, CAN_Bus_TypeDef* can)
{
    // Locals
    CanTxMsgTypeDef tx;
    uint8_t i;

    tx.IDE = CAN_ID_STD;
    tx.RTR = CAN_RTR_DATA;
    tx.StdId = ID_EMDRIVE_SDO_RX | NODE_ID;     // Note: RX is from Emdrive perspective (as per datasheets)
    tx.DLC = 8;
    tx.Data[0] = (uint8_t) size;
    tx.Data[1] = (uint8_t) od;
    tx.Data[2] = (uint8_t) (od >> 8);
    tx.Data[3] = (uint8_t) sub_od;

    // Reverse the bytes
    for (i = 0; i < 4; i++)
    {
        tx.Data[i + 4] = (uint8_t) (data >> (i * 8));
    }

    xQueueSendToBack(can->q_tx, &tx, 1000);
}

// @brief: Function for sending config parameters to the emdrive
// @author: Chris Fallon & Dawson Moore
void emdrive_control(emdrive_nmt_command_t action, emdrive_t* drive, CAN_Bus_TypeDef* can)
{
    // Locals
    CanTxMsgTypeDef tx;

    tx.IDE = CAN_ID_STD;
    tx.RTR = CAN_RTR_DATA;

    if (drive->state == PRE_OPERATION && action == EMDRIVE_STOP)
    {
    	// Set to pre-op mode
		tx.StdId = ID_EMDRIVE_NMT_CONTROL;
		tx.DLC = 2;
		tx.Data[0] = EMDRIVE_PRE_OP;
		tx.Data[1] = 0x00;
		xQueueSendToBack(can->q_tx, &tx, 100);
    }
    else if (drive->state == PRE_OPERATION && action == EMDRIVE_START)
    {
        /*
         * Start sequence:
         * Send NMT pre-operation to ensure we're in a known state
         * Send NMT start to move to operation mode
         * Set controlword to 6
         * Set controlword to 15
         */

        // Set to pre-op mode
        tx.StdId = ID_EMDRIVE_NMT_CONTROL;
        tx.DLC = 2;
        tx.Data[0] = EMDRIVE_PRE_OP;
        tx.Data[1] = 0x00;
        xQueueSendToBack(can->q_tx, &tx, 100);

        // Might be good to add some wait states here
        vTaskDelay(200);

        // Set to op mode
        tx.Data[0] = EMDRIVE_START;
        xQueueSendToBack(can->q_tx, &tx, 100);

        // Only move state once we're for sure in operational mode
        drive->state = OPERATION;

        vTaskDelay(200);

        // Set controlword to 6
        sendSDO(BYTES_2, CONTROLWORD, 0, 6, can);

        vTaskDelay(200);

        // Set controlword to 15
        sendSDO(BYTES_2, CONTROLWORD, 0, 15, can);
    }
    else if (drive->state == OPERATION && action == EMDRIVE_STOP)
    {
        // Set to pre-op mode
        tx.StdId = ID_EMDRIVE_NMT_CONTROL;
        tx.DLC = 2;
        tx.Data[0] = EMDRIVE_PRE_OP;
        tx.Data[1] = 0x00;
        xQueueSendToBack(can->q_tx, &tx, 100);

        // Only move state once we're for sure in pre-operational mode
        drive->state = PRE_OPERATION;
    }

    // Make sure the torque command is 0 so we don't have any accidents
    emdrive_move_the_car_yo(0, can);
}

// @brief: function for parsing the data returned from the emdrive
//         TPDO1 - TPDO3 values. These are sent after the sync function 
//         initiates the transaction.
void emdrive_parse_pdo(CAN_IDs_t id, uint8_t * data, emdrive_t * drive)
{
  emdrive_tpdo_t * pdo = &(drive->data);

  if (id == (ID_EMDRIVE_SLAVE_PDO_1 | NODE_ID))
  {
    pdo->torque_actual = parse_from_lil_16(data + BEGIN_DATA_BYTE(6));
    pdo->position_actual = parse_from_lil_32(data + BEGIN_DATA_BYTE(2));
    pdo->status_word = parse_from_lil_16(data);
    emdrive_check_statusword(drive);
  }
  else if (id == (ID_EMDRIVE_SLAVE_PDO_2 | NODE_ID))
  {
    pdo->electrical_power = parse_from_lil_16(data + BEGIN_DATA_BYTE(4));
    pdo->error_codes = parse_from_lil_32(data + BEGIN_DATA_BYTE(2));
    pdo->motor_temp = data[1];
    pdo->emdrive_temp = data[0];
  }
  else if (id == (ID_EMDRIVE_SLAVE_PDO_3 | NODE_ID))
  {
    pdo->phase_b_current = parse_from_lil_16(data + BEGIN_DATA_BYTE(6));
    pdo->velocity = parse_from_lil_32((data + BEGIN_DATA_BYTE(2)));
    pdo->actual_current = parse_from_lil_16(data);
  }

} 

// @brief: Function for commanding torque from motor controller
// @author: Chris Fallon & Dawson Moore
void emdrive_move_the_car_yo(int16_t torque, CAN_Bus_TypeDef * can)
{
    sendSDO(BYTES_2, TORQUE, 0, torque, can);
}

// @brief: Function which checks the statusword 
// @return: 
uint16_t emdrive_check_statusword(emdrive_t * drive)
{
  uint16_t status = drive->data.status_word;
  if (status & (1 << WARNING))
  {
    drive->warning = true;
  }
  if (status & (1 << FAULT))
  {
    drive->err = drive->data.error_codes;
  }
  return 0;
}
