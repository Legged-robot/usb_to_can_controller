/**
  ******************************************************************************
  * @file    can.h
  * @brief   This file contains all the function prototypes for
  *          the can.c file
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
/* Define to prevent recursive inclusion -------------------------------------*/
#ifndef __CAN_H__
#define __CAN_H__

#ifdef __cplusplus
extern "C" {
#endif

/* Includes ------------------------------------------------------------------*/
#include "main.h"

/* USER CODE BEGIN Includes */

/* USER CODE END Includes */

extern CAN_HandleTypeDef hcan1;

/* USER CODE BEGIN Private defines */

// Only accepts messages addressed to 0x00 device
#define CAN_ID 		0x00
#define CAN_ID_MASK 0x7FF

/* USER CODE END Private defines */

void MX_CAN1_Init(void);

/* USER CODE BEGIN Prototypes */
typedef struct {
	uint8_t id;
	uint8_t data[6];
	CAN_RxHeaderTypeDef rx_header;
	CAN_FilterTypeDef filter;
} CANRxMessage ;

typedef struct {
	uint8_t id;
  uint8_t data[8];
	CAN_TxHeaderTypeDef tx_header;
} CANTxMessage ;

union python_msg {
  float values[5];
  uint8_t bytes[20]; 
};

void CAN_User_Config(CAN_HandleTypeDef* canHandle, CANRxMessage *rx_msg, CANTxMessage *tx_msg);
void pack_pvkt(uint8_t data[], float p_des, float v_des, float kp, float kd, float t_ff);
void unpack_pvt(float* p, float* v, float* i, uint8_t data[]);

/* USER CODE END Prototypes */

#ifdef __cplusplus
}
#endif

#endif /* __CAN_H__ */

/************************ (C) COPYRIGHT STMicroelectronics *****END OF FILE****/
