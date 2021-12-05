/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file           : main.c
  * @brief          : Main program body
  ******************************************************************************
  * @attention
  *
  * <h2><center>&copy; Copyright (c) 2021 STMicroelectronics.
  * All rights reserved.</center></h2>
  *
  * This software component is licensed by ST under BSD 3-Clause license,
  * the "License"; You may not use this file except in compliance with the
  * License. You may obtain a copy of the License at:
  *                        opensource.org/licenses/BSD-3-Clause
  *
  ******************************************************************************
  */
/* USER CODE END Header */
/* Includes ------------------------------------------------------------------*/
#include "main.h"
#include "can.h"
#include "usb_device.h"
#include "gpio.h"

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */
#include "usbd_cdc_if.h"
#include <string.h>
/* USER CODE END Includes */

/* Private typedef -----------------------------------------------------------*/
/* USER CODE BEGIN PTD */

/* USER CODE END PTD */

/* Private define ------------------------------------------------------------*/
/* USER CODE BEGIN PD */

#define CAN_H			 	hcan1				// CAN handle
#define SPECIAL_CMD_LEN		8

/* USER CODE END PD */

/* Private macro -------------------------------------------------------------*/
/* USER CODE BEGIN PM */

/* USER CODE END PM */

/* Private variables ---------------------------------------------------------*/

/* USER CODE BEGIN PV */

/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
/* USER CODE BEGIN PFP */

/* USER CODE END PFP */

/* Private user code ---------------------------------------------------------*/
/* USER CODE BEGIN 0 */
uint8_t buff[20] = "Hello world! \r\n";
CANTxMessage can_tx;
CANRxMessage can_rx;
volatile uint8_t flag_can_rx = 0;
static union python_msg tx;
/* USER CODE END 0 */

/**
  * @brief  The application entry point.
  * @retval int
  */
int main(void)
{
  /* USER CODE BEGIN 1 */

  /* USER CODE END 1 */

  /* MCU Configuration--------------------------------------------------------*/

  /* Reset of all peripherals, Initializes the Flash interface and the Systick. */
  HAL_Init();

  /* USER CODE BEGIN Init */

  /* USER CODE END Init */

  /* Configure the system clock */
  SystemClock_Config();

  /* USER CODE BEGIN SysInit */

  /* USER CODE END SysInit */

  /* Initialize all configured peripherals */
  MX_GPIO_Init();
  MX_USB_DEVICE_Init();
  MX_CAN1_Init();
  /* USER CODE BEGIN 2 */

	/* CAN setup */
	CAN_User_Config(&CAN_H, &can_rx, &can_tx);

  /* USER CODE END 2 */

  /* Infinite loop */
  /* USER CODE BEGIN WHILE */
  while (1)
  {
    /* USER CODE END WHILE */
//	HAL_Delay(100);
	if (flag_can_rx){
		// Transmit data together with address through USB (from CAN interrupt doesnt work)
		flag_can_rx = 0;
		CDC_Transmit_FS(tx.bytes, 13);
	}


    /* USER CODE BEGIN 3 */
  }
  /* USER CODE END 3 */
}

/**
  * @brief System Clock Configuration
  * @retval None
  */
void SystemClock_Config(void)
{
  RCC_OscInitTypeDef RCC_OscInitStruct = {0};
  RCC_ClkInitTypeDef RCC_ClkInitStruct = {0};

  /** Configure the main internal regulator output voltage
  */
  __HAL_RCC_PWR_CLK_ENABLE();
  __HAL_PWR_VOLTAGESCALING_CONFIG(PWR_REGULATOR_VOLTAGE_SCALE1);
  /** Initializes the RCC Oscillators according to the specified parameters
  * in the RCC_OscInitTypeDef structure.
  */
  RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_HSE;
  RCC_OscInitStruct.HSEState = RCC_HSE_ON;
  RCC_OscInitStruct.PLL.PLLState = RCC_PLL_ON;
  RCC_OscInitStruct.PLL.PLLSource = RCC_PLLSOURCE_HSE;
  RCC_OscInitStruct.PLL.PLLM = 4;
  RCC_OscInitStruct.PLL.PLLN = 168;
  RCC_OscInitStruct.PLL.PLLP = RCC_PLLP_DIV2;
  RCC_OscInitStruct.PLL.PLLQ = 7;
  if (HAL_RCC_OscConfig(&RCC_OscInitStruct) != HAL_OK)
  {
    Error_Handler();
  }
  /** Initializes the CPU, AHB and APB buses clocks
  */
  RCC_ClkInitStruct.ClockType = RCC_CLOCKTYPE_HCLK|RCC_CLOCKTYPE_SYSCLK
                              |RCC_CLOCKTYPE_PCLK1|RCC_CLOCKTYPE_PCLK2;
  RCC_ClkInitStruct.SYSCLKSource = RCC_SYSCLKSOURCE_PLLCLK;
  RCC_ClkInitStruct.AHBCLKDivider = RCC_SYSCLK_DIV1;
  RCC_ClkInitStruct.APB1CLKDivider = RCC_HCLK_DIV4;
  RCC_ClkInitStruct.APB2CLKDivider = RCC_HCLK_DIV2;

  if (HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_5) != HAL_OK)
  {
    Error_Handler();
  }
}

/* USER CODE BEGIN 4 */

void HAL_CAN_RxFifo0MsgPendingCallback(CAN_HandleTypeDef *hcan)
{
	/* Get RX message */
	if (hcan != &CAN_H){
		printf("Check can receiver!");
	}
	if (HAL_CAN_GetRxMessage(&CAN_H, CAN_RX_FIFO0, &can_rx.rx_header, can_rx.data) != HAL_OK)
	{
		/* Reception Error */
		Error_Handler();
	}

	if(can_rx.rx_header.DLC != 6){	// 6 byte compressed values
		printf("Warning: CAN packet of length 6 expected, received %ld .\n", can_rx.rx_header.DLC);
	}

	unpack_pvt(&tx.values[0], &tx.values[1], &tx.values[2], can_rx.data);	// uncompress to 3 * (4 byte float value)

	printf("Received package intended for CAN ID: 0x%lx \n", can_rx.rx_header.StdId);

	tx.bytes[12] = (uint8_t) can_rx.data[5]; //add address as last byte (address should fit within 8 bits)

	// From CAN interrupt transmission doesn't work, schedule transmission in non-interrupt mode
	flag_can_rx = 1;

}

void CDC_Receive_Callback(uint8_t* Buf, uint16_t Len)
{
	uint32_t TxMailbox;
	can_tx.tx_header.StdId = Buf[Len-1]; //Last byte is destination address

	if (Buf[0] == 0xFF && (Len-1) == SPECIAL_CMD_LEN) { // SPECIAL_CMD_LEN bytes special command + 1 byte address
		can_tx.tx_header.DLC = Len - 1;
		HAL_CAN_AddTxMessage(&CAN_H, &can_tx.tx_header, Buf, &TxMailbox);	// Send message
	}
	else {
		if(Len!= 21){	// 5 * (4 byte float value) + 1 byte address
		  printf("Warning: CDC packet of length 21 expected, received %d .\n", Len);
		}
		can_tx.tx_header.DLC = 8; //Data will be compressed to 8 bytes
		union python_msg rx;
		memcpy(rx.bytes, Buf, Len-1);

		// rx should have data ordered in sequence: p_des, v_des, kp, kd, t_ff
		pack_pvkt(can_tx.data, rx.values[0], rx.values[1], rx.values[2], rx.values[3], rx.values[4]);
		HAL_CAN_AddTxMessage(&CAN_H, &can_tx.tx_header, can_tx.data, &TxMailbox);	// Send message
	}
//	uint8_t bytes[] = {0x48,0x65,0x79,0x20,0x77,0x68,0x61,0x74,0x73,0x20,0x70,0x0d,0x0a,0x70,0x70,0x70,0x0a,0x0a,0x0a,0x0a};
//	CDC_Transmit_FS(bytes, 13);
}
/* USER CODE END 4 */

/**
  * @brief  This function is executed in case of error occurrence.
  * @retval None
  */
void Error_Handler(void)
{
  /* USER CODE BEGIN Error_Handler_Debug */
  /* User can add his own implementation to report the HAL error return state */
  __disable_irq();
  while (1)
  {
  }
  /* USER CODE END Error_Handler_Debug */
}

#ifdef  USE_FULL_ASSERT
/**
  * @brief  Reports the name of the source file and the source line number
  *         where the assert_param error has occurred.
  * @param  file: pointer to the source file name
  * @param  line: assert_param error line source number
  * @retval None
  */
void assert_failed(uint8_t *file, uint32_t line)
{
  /* USER CODE BEGIN 6 */
  /* User can add his own implementation to report the file name and line number,
     ex: printf("Wrong parameters value: file %s on line %d\r\n", file, line) */
  /* USER CODE END 6 */
}
#endif /* USE_FULL_ASSERT */

/************************ (C) COPYRIGHT STMicroelectronics *****END OF FILE****/