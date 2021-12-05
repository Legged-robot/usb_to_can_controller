/**
  ******************************************************************************
  * @file    can.c
  * @brief   This file provides code for the configuration
  *          of the CAN instances.
  ******************************************************************************
  * @attention
  *
  * <h2><center>&copy; Copyright (c) 2021 STMicroelectronics.
  * All rights reserved.</center></h2>
  *
  * This software component is licensed by ST under Ultimate Liberty license
  * SLA0044, the "License"; You may not use this file except in compliance with
  * the License. You may obtain a copy of the License at:
  *                             www.st.com/SLA0044
  *
  ******************************************************************************
  */

/* Includes ------------------------------------------------------------------*/
#include "can.h"

/* USER CODE BEGIN 0 */

#include "math_ops.h"

// Value Limits
#define P_MIN -12.5f
#define P_MAX 12.5f
#define V_MIN -65.0f
#define V_MAX 65.0f
#define KP_MIN 0.0f
#define KP_MAX 500.0f
#define KD_MIN 0.0f
#define KD_MAX 5.0f
#define I_MIN -30.0f
#define I_MAX 30.0f

/* USER CODE END 0 */

CAN_HandleTypeDef hcan1;

/* CAN1 init function */
void MX_CAN1_Init(void)
{

  /* USER CODE BEGIN CAN1_Init 0 */

  /* USER CODE END CAN1_Init 0 */

  /* USER CODE BEGIN CAN1_Init 1 */

  /* USER CODE END CAN1_Init 1 */
  hcan1.Instance = CAN1;
  hcan1.Init.Prescaler = 6;
  hcan1.Init.Mode = CAN_MODE_NORMAL;
  hcan1.Init.SyncJumpWidth = CAN_SJW_1TQ;
  hcan1.Init.TimeSeg1 = CAN_BS1_11TQ;
  hcan1.Init.TimeSeg2 = CAN_BS2_2TQ;
  hcan1.Init.TimeTriggeredMode = DISABLE;
  hcan1.Init.AutoBusOff = DISABLE;
  hcan1.Init.AutoWakeUp = DISABLE;
  hcan1.Init.AutoRetransmission = DISABLE;
  hcan1.Init.ReceiveFifoLocked = DISABLE;
  hcan1.Init.TransmitFifoPriority = DISABLE;
  if (HAL_CAN_Init(&hcan1) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN CAN1_Init 2 */

  /* USER CODE END CAN1_Init 2 */

}

void HAL_CAN_MspInit(CAN_HandleTypeDef* canHandle)
{

  GPIO_InitTypeDef GPIO_InitStruct = {0};
  if(canHandle->Instance==CAN1)
  {
  /* USER CODE BEGIN CAN1_MspInit 0 */

  /* USER CODE END CAN1_MspInit 0 */
    /* CAN1 clock enable */
    __HAL_RCC_CAN1_CLK_ENABLE();

    __HAL_RCC_GPIOD_CLK_ENABLE();
    /**CAN1 GPIO Configuration
    PD0     ------> CAN1_RX
    PD1     ------> CAN1_TX
    */
    GPIO_InitStruct.Pin = GPIO_PIN_0|GPIO_PIN_1;
    GPIO_InitStruct.Mode = GPIO_MODE_AF_PP;
    GPIO_InitStruct.Pull = GPIO_NOPULL;
    GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_VERY_HIGH;
    GPIO_InitStruct.Alternate = GPIO_AF9_CAN1;
    HAL_GPIO_Init(GPIOD, &GPIO_InitStruct);

    /* CAN1 interrupt Init */
    HAL_NVIC_SetPriority(CAN1_RX0_IRQn, 0, 0);
    HAL_NVIC_EnableIRQ(CAN1_RX0_IRQn);
  /* USER CODE BEGIN CAN1_MspInit 1 */

  /* USER CODE END CAN1_MspInit 1 */
  }
}

void HAL_CAN_MspDeInit(CAN_HandleTypeDef* canHandle)
{

  if(canHandle->Instance==CAN1)
  {
  /* USER CODE BEGIN CAN1_MspDeInit 0 */

  /* USER CODE END CAN1_MspDeInit 0 */
    /* Peripheral clock disable */
    __HAL_RCC_CAN1_CLK_DISABLE();

    /**CAN1 GPIO Configuration
    PD0     ------> CAN1_RX
    PD1     ------> CAN1_TX
    */
    HAL_GPIO_DeInit(GPIOD, GPIO_PIN_0|GPIO_PIN_1);

    /* CAN1 interrupt Deinit */
    HAL_NVIC_DisableIRQ(CAN1_RX0_IRQn);
  /* USER CODE BEGIN CAN1_MspDeInit 1 */

  /* USER CODE END CAN1_MspDeInit 1 */
  }
}

/* USER CODE BEGIN 1 */

void CAN_User_Config(CAN_HandleTypeDef* canHandle, CANRxMessage *rx_msg, CANTxMessage *tx_msg)
{
	/*##-2- Configure the CAN Filter ###########################################*/
	/* Can configuration brief https://controllerstech.com/can-protocol-in-stm32/ */
	rx_msg->filter.FilterBank=9; 	// which filter bank to use from the assigned ones
	rx_msg->filter.SlaveStartFilterBank = 20; // how many filters to assign to the CAN1 (master can)
	rx_msg->filter.FilterFIFOAssignment=CAN_FILTER_FIFO0; 	// set fifo assignment
	rx_msg->filter.FilterIdHigh=CAN_ID<<5; 				// CAN ID - Standard ID in message starts from <<5 bit in idHigh
	rx_msg->filter.FilterIdLow=0x0;
	rx_msg->filter.FilterMaskIdHigh=CAN_ID_MASK<<5;
	rx_msg->filter.FilterMaskIdLow=0x0;
	rx_msg->filter.FilterMode = CAN_FILTERMODE_IDMASK;
	rx_msg->filter.FilterScale=CAN_FILTERSCALE_32BIT;
	rx_msg->filter.FilterActivation=ENABLE;

	if (HAL_CAN_ConfigFilter(canHandle, &rx_msg->filter) != HAL_OK)
	{
		/* Filter configuration Error */
		Error_Handler();
	}

	/*##-3- Start the CAN peripheral ###########################################*/
	if (HAL_CAN_Start(canHandle) != HAL_OK)
	{
		/* Start Error */
		Error_Handler();
	}

	/*##-4- Activate CAN RX notification #######################################*/
	if (HAL_CAN_ActivateNotification(canHandle, CAN_IT_RX_FIFO0_MSG_PENDING) != HAL_OK)
	{
		/* Notification Error */
		Error_Handler();
	}

	/*##-5- Configure Transmission process #####################################*/
//	tx_msg->tx_header.DLC = 8; 			// message size of 8 byte
	tx_msg->tx_header.IDE = CAN_ID_STD; 		// set identifier to standard
	tx_msg->tx_header.RTR = CAN_RTR_DATA; 	// set data type to remote transmission request?
//	tx_msg->tx_header.StdId = CAN_MASTER;  // recipient CAN ID
	// tx_msg->tx_header.TransmitGlobalTime = DISABLE;
}

/* CAN Command Packet Structure
 *
 * 16 bit position command, between -4*pi and 4*pi
 * 12 bit velocity command, between -30 and + 30 rad/s
 * 12 bit kp, between 0 and 500 N-m/rad
 * 12 bit kd, between 0 and 100 N-m*s/rad
 * 12 bit feed forward torque, between -18 and 18 N-m
 * CAN Packet is 8 8-bit words
 * Formatted as follows.  For each quantity, bit 0 is LSB
 * 0: [position[15-8]]
 * 1: [position[7-0]]
 * 2: [velocity[11-4]]
 * 3: [velocity[3-0], kp[11-8]]
 * 4: [kp[7-0]]
 * 5: [kd[11-4]]
 * 6: [kd[3-0], torque[11-8]]
 * 7: [torque[7-0]]
 */ 


void pack_pvkt(uint8_t data[], float p_des, float v_des, float kp, float kd, float t_ff)
{

     // limit data to be within bounds 
     p_des = fminf(fmaxf(P_MIN, p_des), P_MAX);
     v_des = fminf(fmaxf(V_MIN, v_des), V_MAX);
     kp = fminf(fmaxf(KP_MIN, kp), KP_MAX);
     kd = fminf(fmaxf(KD_MIN, kd), KD_MAX);
     t_ff = fminf(fmaxf(I_MIN, t_ff), I_MAX);

     // convert floats to unsigned ints 
     uint32_t p_uint = float_to_uint(p_des, P_MIN, P_MAX, 16);
     uint32_t v_uint = float_to_uint(v_des, V_MIN, V_MAX, 12);
     uint32_t kp_uint = float_to_uint(kp, KP_MIN, KP_MAX, 12);
     uint32_t kd_uint = float_to_uint(kd, KD_MIN, KD_MAX, 12);
     uint32_t t_uint = float_to_uint(t_ff, I_MIN, I_MAX, 12);

     // pack ints into the CAN buffer 
     data[0] = p_uint>>8;
     data[1] = p_uint&0xFF;
     data[2] = v_uint>>4;
     data[3] = ((v_uint&0xF)<<4)|(kp_uint>>8);
     data[4] = kp_uint&0xFF;
     data[5] = kd_uint>>4;
     data[6] = ((kd_uint&0xF)<<4)|(t_uint>>8);
     data[7] = t_uint&0xff;
}

/* CAN Reply Packet Structure 
 *
 * 16 bit position, between -4*pi and 4*pi
 * 12 bit velocity, between -30 and + 30 rad/s
 * 12 bit current, between -40 and 40;
 * CAN Packet is 5 8-bit words
 * Formatted as follows.  For each quantity, bit 0 is LSB
 * 0: [position[15-8]]
 * 1: [position[7-0]]
 * 2: [velocity[11-4]]
 * 3: [velocity[3-0], current[11-8]]
 * 4: [current[7-0]]
 */

void unpack_pvt(float* p, float* v, float* i, uint8_t data[])
{
	/// unpack ints from can buffer ///
  uint32_t p_uint = (data[0]<<8)|data[1];
  uint32_t v_uint = (data[2]<<4)|(data[3]>>4);
  uint32_t i_uint = ((data[3]&0xF)<<8)|data[4];
	/// convert ints to floats ///
	*p = uint_to_float(p_uint, P_MIN, P_MAX, 16);
	*v = uint_to_float(v_uint, V_MIN, V_MAX, 12);
	*i = uint_to_float(i_uint, -I_MAX, I_MAX, 12);
	//printf("%d  %f\n\r", id, p);
	//printf("%d  %.3f   %.3f   %.3f\n\r", id, p, v, i);
}


/* USER CODE END 1 */

/************************ (C) COPYRIGHT STMicroelectronics *****END OF FILE****/
