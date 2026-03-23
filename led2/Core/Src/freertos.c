/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * File Name          : freertos.c
  * Description        : Code for freertos applications
  ******************************************************************************
  * @attention
  *
  * Copyright (c) 2026 STMicroelectronics.
  * All rights reserved.
  *
  * This software is licensed under terms that can be found in the LICENSE file
  * in the root directory of this software component.
  * If no LICENSE file comes with this software, it is provided AS-IS.
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
/* USER CODE END Includes */

/* Private typedef -----------------------------------------------------------*/
/* USER CODE BEGIN PTD */

/* USER CODE END PTD */

/* Private define ------------------------------------------------------------*/
/* USER CODE BEGIN PD */
#define FRAME_HEADER_1   0xAA
#define FRAME_HEADER_2   0x55
#define FRAME_CMD        0x10
#define CYLINDER_HOLD_MS 2000
#define DEBOUNCE_MS      20
#define SWITCH_POLL_MS   10
/* USER CODE END PD */

/* Private macro -------------------------------------------------------------*/
/* USER CODE BEGIN PM */

/* USER CODE END PM */

/* Private variables ---------------------------------------------------------*/
/* USER CODE BEGIN Variables */
static osSemaphoreId_t detectSemHandle;
static osSemaphoreId_t uartSemHandle;
static osSemaphoreId_t uartDoneSemHandle;
/* USER CODE END Variables */
/* Definitions for defaultTask */
osThreadId_t defaultTaskHandle;
const osThreadAttr_t defaultTask_attributes = {
  .name = "defaultTask",
  .stack_size = 128 * 4,
  .priority = (osPriority_t) osPriorityNormal,
};

/* Private function prototypes -----------------------------------------------*/
/* USER CODE BEGIN FunctionPrototypes */
static void vDetectTask(void *argument);
static void vControlTask(void *argument);
static void vUartTask(void *argument);
/* USER CODE END FunctionPrototypes */

void StartDefaultTask(void *argument);

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
  detectSemHandle  = osSemaphoreNew(1, 0, NULL);
  uartSemHandle    = osSemaphoreNew(1, 0, NULL);
  uartDoneSemHandle = osSemaphoreNew(1, 0, NULL);
  /* USER CODE END RTOS_SEMAPHORES */

  /* USER CODE BEGIN RTOS_TIMERS */
  /* start timers, add new ones, ... */
  /* USER CODE END RTOS_TIMERS */

  /* USER CODE BEGIN RTOS_QUEUES */
  /* add queues, ... */
  /* USER CODE END RTOS_QUEUES */

  /* Create the thread(s) */
  /* creation of defaultTask */
  defaultTaskHandle = osThreadNew(StartDefaultTask, NULL, &defaultTask_attributes);

  /* USER CODE BEGIN RTOS_THREADS */
  /* add threads, ... */
  static const osThreadAttr_t detectTask_attr  = { .name = "DetectTask",  .stack_size = 128 * 4, .priority = osPriorityNormal };
  static const osThreadAttr_t controlTask_attr = { .name = "ControlTask", .stack_size = 128 * 4, .priority = osPriorityAboveNormal };
  static const osThreadAttr_t uartTask_attr    = { .name = "UartTask",    .stack_size = 128 * 4, .priority = osPriorityNormal };
  osThreadNew(vDetectTask,  NULL, &detectTask_attr);
  osThreadNew(vControlTask, NULL, &controlTask_attr);
  osThreadNew(vUartTask,    NULL, &uartTask_attr);
  /* USER CODE END RTOS_THREADS */

  /* USER CODE BEGIN RTOS_EVENTS */
  /* add events, ... */
  /* USER CODE END RTOS_EVENTS */

}

/* USER CODE BEGIN Header_StartDefaultTask */
/**
  * @brief  Function implementing the defaultTask thread.
  * @param  argument: Not used
  * @retval None
  */
/* USER CODE END Header_StartDefaultTask */
void StartDefaultTask(void *argument)
{
  /* USER CODE BEGIN StartDefaultTask */
  /* Infinite loop */
  for(;;)
  {
    osDelay(1);
  }
  /* USER CODE END StartDefaultTask */
}

/* Private application code --------------------------------------------------*/
/* USER CODE BEGIN Application */

/**
 * @brief 检测任务：轮询光电开关 (PA6 低电平有效)，去抖后通知控制任务
 */
static void vDetectTask(void *argument)
{
    for (;;)
    {
        if (HAL_GPIO_ReadPin(Switch_GPIO_Port, Switch_Pin) == GPIO_PIN_RESET)
        {
            osDelay(DEBOUNCE_MS);   /* 去抖：等待后再次确认 */
            if (HAL_GPIO_ReadPin(Switch_GPIO_Port, Switch_Pin) == GPIO_PIN_RESET)
            {
                osSemaphoreRelease(detectSemHandle);

                /* 等待物体离开，避免重复触发 */
                while (HAL_GPIO_ReadPin(Switch_GPIO_Port, Switch_Pin) == GPIO_PIN_RESET)
                {
                    osDelay(SWITCH_POLL_MS);
                }
            }
        }
        osDelay(SWITCH_POLL_MS);
    }
}

/**
 * @brief 控制任务：拉低 ENA 停机 → 触发气缸 500ms → 等串口发完 → 恢复 ENA
 */
static void vControlTask(void *argument)
{
    for (;;)
    {
        osSemaphoreAcquire(detectSemHandle, osWaitForever);

        /* 停止传送带 */
        HAL_GPIO_WritePin(ENA_GPIO_Port, ENA_Pin, GPIO_PIN_RESET);

        /* 触发气缸 */
        HAL_GPIO_WritePin(Cylinder_GPIO_Port, Cylinder_Pin, GPIO_PIN_SET);
        osDelay(CYLINDER_HOLD_MS);
        HAL_GPIO_WritePin(Cylinder_GPIO_Port, Cylinder_Pin, GPIO_PIN_RESET);

        /* 通知串口任务发送帧，并等待发送完成 */
        osSemaphoreRelease(uartSemHandle);
        osSemaphoreAcquire(uartDoneSemHandle, osWaitForever);

        /* 恢复传送带 */
        HAL_GPIO_WritePin(ENA_GPIO_Port, ENA_Pin, GPIO_PIN_SET);
    }
}

/**
 * @brief 串口任务：组帧并发送，格式：AA 55 10 [data] [XOR]
 */
static void vUartTask(void *argument)
{
    static const uint8_t payload[] = {0x01, 0x00};  /* 固定占位数据 */
    uint8_t frame[6];
    uint8_t xor_val;
    size_t i;

    for (;;)
    {
        osSemaphoreAcquire(uartSemHandle, osWaitForever);

        /* 组帧 */
        frame[0] = FRAME_HEADER_1;
        frame[1] = FRAME_HEADER_2;
        frame[2] = FRAME_CMD;
        frame[3] = payload[0];
        frame[4] = payload[1];

        /* XOR 校验：对命令字节 + 数据字节做异或 */
        xor_val = FRAME_CMD;
        for (i = 0; i < sizeof(payload); i++)
        {
            xor_val ^= payload[i];
        }
        frame[5] = xor_val;

        HAL_UART_Transmit(&huart1, frame, sizeof(frame), 100);

        /* 通知控制任务：串口发送完毕 */
        osSemaphoreRelease(uartDoneSemHandle);
    }
}

/* USER CODE END Application */

