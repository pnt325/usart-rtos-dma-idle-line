/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * File Name          : freertos.c
  * Description        : Code for freertos applications
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
/* USER CODE END Header */

/* Includes ------------------------------------------------------------------*/
#include "FreeRTOS.h"
#include "task.h"
#include "main.h"
#include "cmsis_os.h"

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */
#include "usart.h"
#include "queue.h"
#include <string.h>
#include <stdio.h>
/* USER CODE END Includes */

/* Private typedef -----------------------------------------------------------*/
/* USER CODE BEGIN PTD */
typedef struct
{
	uint8_t len;
	uint8_t buf[USART_BUF_SIZE/2];
}TxPacket_t;
/* USER CODE END PTD */

/* Private define ------------------------------------------------------------*/
/* USER CODE BEGIN PD */

/* USER CODE END PD */

/* Private macro -------------------------------------------------------------*/
/* USER CODE BEGIN PM */

/* USER CODE END PM */

/* Private variables ---------------------------------------------------------*/
/* USER CODE BEGIN Variables */
static QueueHandle_t txQueue;
/* USER CODE END Variables */
/* Definitions for usartTask */
osThreadId_t usartTaskHandle;
const osThreadAttr_t usartTask_attributes = {
  .name = "usartTask",
  .stack_size = 1024 * 4,
  .priority = (osPriority_t) osPriorityNormal,
};
/* Definitions for txTask */
osThreadId_t txTaskHandle;
const osThreadAttr_t txTask_attributes = {
  .name = "txTask",
  .stack_size = 1024 * 4,
  .priority = (osPriority_t) osPriorityLow,
};

/* Private function prototypes -----------------------------------------------*/
/* USER CODE BEGIN FunctionPrototypes */

/* USER CODE END FunctionPrototypes */

void usartTaskFunc(void *argument);
void txTaskFunc(void *argument);

void MX_FREERTOS_Init(void); /* (MISRA C 2004 rule 8.1) */

/**
  * @brief  FreeRTOS initialization
  * @param  None
  * @retval None
  */
void MX_FREERTOS_Init(void) {
  /* USER CODE BEGIN Init */

  /* USER CODE END Init */

  /* USER CODE BEGIN RTOS_MUTEX */
  /* add mutexes, ... */
  /* USER CODE END RTOS_MUTEX */

  /* USER CODE BEGIN RTOS_SEMAPHORES */
  /* add semaphores, ... */
  /* USER CODE END RTOS_SEMAPHORES */

  /* USER CODE BEGIN RTOS_TIMERS */
  /* start timers, add new ones, ... */
  /* USER CODE END RTOS_TIMERS */

  /* USER CODE BEGIN RTOS_QUEUES */
	txQueue = xQueueCreate(10, sizeof(TxPacket_t));
	if(txQueue == 0)
	{
		while(1)
		{
			;
		}
	}
  /* add queues, ... */
  /* USER CODE END RTOS_QUEUES */

  /* Create the thread(s) */
  /* creation of usartTask */
  usartTaskHandle = osThreadNew(usartTaskFunc, NULL, &usartTask_attributes);

  /* creation of txTask */
  txTaskHandle = osThreadNew(txTaskFunc, NULL, &txTask_attributes);

  /* USER CODE BEGIN RTOS_THREADS */
  /* add threads, ... */
  /* USER CODE END RTOS_THREADS */

  /* USER CODE BEGIN RTOS_EVENTS */
  /* add events, ... */
  /* USER CODE END RTOS_EVENTS */

}

/* USER CODE BEGIN Header_usartTaskFunc */
/**
  * @brief  Function implementing the usartTask thread.
  * @param  argument: Not used
  * @retval None
  */
/* USER CODE END Header_usartTaskFunc */
void usartTaskFunc(void *argument)
{
  /* USER CODE BEGIN usartTaskFunc */
	usartStart();
	TxPacket_t rxPacket;
  /* Infinite loop */
  for(;;)
  {
	  if(usartRecvBlocking(rxPacket.buf, &rxPacket.len))
	  {
		  // send data to task send
		  if(xQueueSend(txQueue, &rxPacket, pdMS_TO_TICKS(10)) != pdPASS)
		  {
			  Error_Handler();
		  }
	  }
  }
  /* USER CODE END usartTaskFunc */
}

/* USER CODE BEGIN Header_txTaskFunc */
/**
* @brief Function implementing the txTask thread.
* @param argument: Not used
* @retval None
*/
/* USER CODE END Header_txTaskFunc */
void txTaskFunc(void *argument)
{
  /* USER CODE BEGIN txTaskFunc */
	TxPacket_t txpacket;
	char str[64] = {0};
  /* Infinite loop */
  for(;;)
  {
	  xQueueReceive(txQueue, &txpacket, portMAX_DELAY);
	  sprintf(str, "DMA receive len = %d\r\n", txpacket.len);
	  usartSendBlocking((uint8_t*)str, strlen(str));
	  usartSendBlocking(txpacket.buf, txpacket.len);
  }
  /* USER CODE END txTaskFunc */
}

/* Private application code --------------------------------------------------*/
/* USER CODE BEGIN Application */

/* USER CODE END Application */

/************************ (C) COPYRIGHT STMicroelectronics *****END OF FILE****/
