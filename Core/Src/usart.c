/**
  ******************************************************************************
  * @file    usart.c
  * @brief   This file provides code for the configuration
  *          of the USART instances.
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
#include "usart.h"

/* USER CODE BEGIN 0 */
#include <stdio.h>
#include "main.h"
#include "FreeRTOS.h"
#include "semphr.h"
#include "task.h"
#include "queue.h"

#define USART_DMA_BUF_SIZE          USART_BUF_SIZE
#define USART_RX_NOTIFY_COUNT_MAX   10

static SemaphoreHandle_t usartRxNotify;
static StaticSemaphore_t usartRxNotifyBuf;
static SemaphoreHandle_t usartTxNotify;
static StaticSemaphore_t usartTxNotifyBuf;
static SemaphoreHandle_t usartBuzzy;
static StaticSemaphore_t usartBuzzyBuf;

static TaskHandle_t taskRx;
static uint8_t dma_rxbuf[USART_DMA_BUF_SIZE];
static uint8_t dma_txbuf[USART_DMA_BUF_SIZE];
static uint8_t lpos;

static void usartInit(void);
static void usartRecv(void);
static void usartRxTask(const void *params);
/* USER CODE END 0 */

UART_HandleTypeDef huart6;
DMA_HandleTypeDef hdma_usart6_rx;
DMA_HandleTypeDef hdma_usart6_tx;

/* USART6 init function */

void MX_USART6_UART_Init(void)
{

    /* USER CODE BEGIN USART6_Init 0 */

    /* USER CODE END USART6_Init 0 */

    /* USER CODE BEGIN USART6_Init 1 */

    /* USER CODE END USART6_Init 1 */
    huart6.Instance = USART6;
    huart6.Init.BaudRate = 115200;
    huart6.Init.WordLength = UART_WORDLENGTH_8B;
    huart6.Init.StopBits = UART_STOPBITS_1;
    huart6.Init.Parity = UART_PARITY_NONE;
    huart6.Init.Mode = UART_MODE_TX_RX;
    huart6.Init.HwFlowCtl = UART_HWCONTROL_NONE;
    huart6.Init.OverSampling = UART_OVERSAMPLING_16;
    huart6.Init.OneBitSampling = UART_ONE_BIT_SAMPLE_DISABLE;
    huart6.AdvancedInit.AdvFeatureInit = UART_ADVFEATURE_NO_INIT;
    if (HAL_UART_Init(&huart6) != HAL_OK)
    {
        Error_Handler();
    }
    /* USER CODE BEGIN USART6_Init 2 */

    /* USER CODE END USART6_Init 2 */
}

void HAL_UART_MspInit(UART_HandleTypeDef *uartHandle)
{

    GPIO_InitTypeDef GPIO_InitStruct = {0};
    if (uartHandle->Instance == USART6)
    {
        /* USER CODE BEGIN USART6_MspInit 0 */

        /* USER CODE END USART6_MspInit 0 */
        /* USART6 clock enable */
        __HAL_RCC_USART6_CLK_ENABLE();

        __HAL_RCC_GPIOC_CLK_ENABLE();
        /**USART6 GPIO Configuration
    PC7     ------> USART6_RX
    PC6     ------> USART6_TX
    */
        GPIO_InitStruct.Pin = GPIO_PIN_7 | GPIO_PIN_6;
        GPIO_InitStruct.Mode = GPIO_MODE_AF_PP;
        GPIO_InitStruct.Pull = GPIO_NOPULL;
        GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_VERY_HIGH;
        GPIO_InitStruct.Alternate = GPIO_AF8_USART6;
        HAL_GPIO_Init(GPIOC, &GPIO_InitStruct);

        /* USART6 DMA Init */
        /* USART6_RX Init */
        hdma_usart6_rx.Instance = DMA2_Stream1;
        hdma_usart6_rx.Init.Channel = DMA_CHANNEL_5;
        hdma_usart6_rx.Init.Direction = DMA_PERIPH_TO_MEMORY;
        hdma_usart6_rx.Init.PeriphInc = DMA_PINC_DISABLE;
        hdma_usart6_rx.Init.MemInc = DMA_MINC_ENABLE;
        hdma_usart6_rx.Init.PeriphDataAlignment = DMA_PDATAALIGN_BYTE;
        hdma_usart6_rx.Init.MemDataAlignment = DMA_MDATAALIGN_BYTE;
        hdma_usart6_rx.Init.Mode = DMA_CIRCULAR;
        hdma_usart6_rx.Init.Priority = DMA_PRIORITY_LOW;
        hdma_usart6_rx.Init.FIFOMode = DMA_FIFOMODE_DISABLE;
        if (HAL_DMA_Init(&hdma_usart6_rx) != HAL_OK)
        {
            Error_Handler();
        }

        __HAL_LINKDMA(uartHandle, hdmarx, hdma_usart6_rx);

        /* USART6_TX Init */
        hdma_usart6_tx.Instance = DMA2_Stream6;
        hdma_usart6_tx.Init.Channel = DMA_CHANNEL_5;
        hdma_usart6_tx.Init.Direction = DMA_MEMORY_TO_PERIPH;
        hdma_usart6_tx.Init.PeriphInc = DMA_PINC_DISABLE;
        hdma_usart6_tx.Init.MemInc = DMA_MINC_ENABLE;
        hdma_usart6_tx.Init.PeriphDataAlignment = DMA_PDATAALIGN_BYTE;
        hdma_usart6_tx.Init.MemDataAlignment = DMA_MDATAALIGN_BYTE;
        hdma_usart6_tx.Init.Mode = DMA_NORMAL;
        hdma_usart6_tx.Init.Priority = DMA_PRIORITY_LOW;
        hdma_usart6_tx.Init.FIFOMode = DMA_FIFOMODE_DISABLE;
        if (HAL_DMA_Init(&hdma_usart6_tx) != HAL_OK)
        {
            Error_Handler();
        }

        __HAL_LINKDMA(uartHandle, hdmatx, hdma_usart6_tx);

        /* USART6 interrupt Init */
        HAL_NVIC_SetPriority(USART6_IRQn, 5, 0);
        HAL_NVIC_EnableIRQ(USART6_IRQn);
        /* USER CODE BEGIN USART6_MspInit 1 */

        /* USER CODE END USART6_MspInit 1 */
    }
}

void HAL_UART_MspDeInit(UART_HandleTypeDef *uartHandle)
{

    if (uartHandle->Instance == USART6)
    {
        /* USER CODE BEGIN USART6_MspDeInit 0 */

        /* USER CODE END USART6_MspDeInit 0 */
        /* Peripheral clock disable */
        __HAL_RCC_USART6_CLK_DISABLE();

        /**USART6 GPIO Configuration
            PC7     ------> USART6_RX
            PC6     ------> USART6_TX
            */
        HAL_GPIO_DeInit(GPIOC, GPIO_PIN_7 | GPIO_PIN_6);

        /* USART6 DMA DeInit */
        HAL_DMA_DeInit(uartHandle->hdmarx);
        HAL_DMA_DeInit(uartHandle->hdmatx);

        /* USART6 interrupt Deinit */
        HAL_NVIC_DisableIRQ(USART6_IRQn);
        /* USER CODE BEGIN USART6_MspDeInit 1 */

        /* USER CODE END USART6_MspDeInit 1 */
    }
}

/* USER CODE BEGIN 1 */

/**
 * @brief Start receive data
 */
void usartStart(void)
{
    usartInit();
    __HAL_UART_ENABLE_IT(&huart6, UART_IT_IDLE);
    HAL_UART_Receive_DMA(&huart6, dma_rxbuf, USART_DMA_BUF_SIZE);
}

bool usartRecvBlocking(uint8_t *data, uint8_t *dataLen)
{
    uint8_t cpos;

    if (data == NULL || dataLen == NULL)
        return false;

    xSemaphoreTake(usartRxNotify, portMAX_DELAY);
    cpos = (USART_DMA_BUF_SIZE - __HAL_DMA_GET_COUNTER(huart6.hdmarx));
    dataLen[0] = 0;

    // Copy data
    while (lpos != cpos)
    {
        data[dataLen[0]] = dma_rxbuf[lpos];
        dataLen[0]++;
        lpos = (lpos + 1) % USART_BUF_SIZE;
    }

    return dataLen[0] ? true : false;
}

void usartSendBlocking(uint8_t *data, uint8_t dataLen)
{
    uint8_t i;
    xSemaphoreTake(usartBuzzy, portMAX_DELAY);

    // give data to dma buffer
    for (i = 0; i < dataLen; i++)
    {
        dma_txbuf[i] = data[i];
    }
    HAL_UART_Transmit_DMA(&huart6, dma_txbuf, dataLen);

    // wait for dma done
    xSemaphoreTake(usartTxNotify, portMAX_DELAY);

    // free usart tx port
    xSemaphoreGive(usartBuzzy);
}

static void usartInit(void)
{
    // initialize some of variable freeRTOS
    usartRxNotify = xSemaphoreCreateCountingStatic(10, 0, &usartRxNotifyBuf);
    usartTxNotify = xSemaphoreCreateBinaryStatic(&usartTxNotifyBuf);
    usartBuzzy = xSemaphoreCreateBinaryStatic(&usartBuzzyBuf);
    xSemaphoreGive(usartBuzzy); // usart not buzzy at first run
    lpos = 0;

    taskRx = xTaskCreate(usartRxTask, "usartRxTask", configMINIMAL_STACK_SIZE, NULL, 24, NULL);
}

static void usartRecv(void)
{
    BaseType_t pxHigherPriorityTaskWoken = pdFALSE;
    xSemaphoreGiveFromISR(usartRxNotify, &pxHigherPriorityTaskWoken);
    portYIELD_FROM_ISR(pxHigherPriorityTaskWoken);
}

static void usartRxTask(const void *params)
{
    uint8_t pos;
    for(;;)
    {
        // handle packet protocol
        xSemaphoreTake(usartTxNotify, portMAX_DELAY);
        pos = (USART_DMA_BUF_SIZE - __HAL_DMA_GET_COUNTER(huart6.hdmarx));
        while(lpos != pos)
        {
            // Handle special packet data
            /**
             * Check with special byte then copy data to packet
             * Delivery to where need it.
             */
        }
    }
}

// interrupt callback
__USED void HAL_UART_RxCpltCallback(UART_HandleTypeDef *huart)
{
    usartRecv();
}

__USED void HAL_UART_RxHalfCpltCallback(UART_HandleTypeDef *huart)
{
    usartRecv();
}

__USED void HAL_UART_TxCpltCallback(UART_HandleTypeDef *huart)
{
    BaseType_t pxHigherPriorityTaskWoken = pdFALSE;
    xSemaphoreGiveFromISR(usartTxNotify, &pxHigherPriorityTaskWoken);
    portYIELD_FROM_ISR(pxHigherPriorityTaskWoken);
}

/* USER CODE END 1 */

/************************ (C) COPYRIGHT STMicroelectronics *****END OF FILE****/
