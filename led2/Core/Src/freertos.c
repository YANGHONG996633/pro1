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

/* Jetson 回复帧协议: BB STATUS XOR (3字节) */
#define REPLY_HEADER     0xBB
#define REPLY_DEFECT     0x01   /* 有缺陷 → 剔除 */
#define REPLY_NORMAL     0x02   /* 无缺陷 → 放行 */

#define CYLINDER_KICK_MS 1000   /* PA7 高电平持续时间 */
#define UART_REPLY_TIMEOUT_MS 30000 /* 等待 Jetson 回复超时 (30s, 给足推理时间) */
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
static volatile uint8_t g_uartReply;   /* 上位机回复字节：0x01 或 0x02 */
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
  static const osThreadAttr_t uartTask_attr    = { .name = "UartTask",    .stack_size = 256 * 4, .priority = osPriorityNormal };
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
 * @brief 控制任务：停带 → 发串口 → 等上位机回复 → 按回复动作 → 启带
 */
static void vControlTask(void *argument)
{
    for (;;)
    {
        osSemaphoreAcquire(detectSemHandle, osWaitForever);

        /* 停止传送带 */
        HAL_GPIO_WritePin(ENA_GPIO_Port, ENA_Pin, GPIO_PIN_RESET);

        /* 通知串口任务发送帧，并等待上位机回复（串口任务会填充 g_uartReply） */
        osSemaphoreRelease(uartSemHandle);
        osSemaphoreAcquire(uartDoneSemHandle, osWaitForever);

        if (g_uartReply == REPLY_DEFECT)
        {
            /* 上位机判定为缺陷：PA7 高电平 1s（剔除动作），再启带 */
            HAL_GPIO_WritePin(Cylinder_GPIO_Port, Cylinder_Pin, GPIO_PIN_SET);
            osDelay(CYLINDER_KICK_MS);
            HAL_GPIO_WritePin(Cylinder_GPIO_Port, Cylinder_Pin, GPIO_PIN_RESET);
        }
        /* else REPLY_NORMAL 或超时：直接启带，不触发 PA7 */

        /* 恢复传送带 */
        HAL_GPIO_WritePin(ENA_GPIO_Port, ENA_Pin, GPIO_PIN_SET);
    }
}

/**
 * @brief 串口任务：组帧发送，然后阻塞等待 Jetson 回复并解析
 *        Jetson 回复帧格式: BB STATUS XOR (3字节)
 *        STATUS=0x01 剔除 | 0x02 放行, XOR = BB ^ STATUS
 */
static void vUartTask(void *argument)
{
    static const uint8_t payload[] = {0x01, 0x00};  /* 固定占位数据 */
    uint8_t frame[6];
    uint8_t xor_val;
    uint8_t reply_buf[3];
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

        /* 等待 Jetson 回复3字节: BB STATUS XOR */
        if (HAL_UART_Receive(&huart1, reply_buf, 3, UART_REPLY_TIMEOUT_MS) == HAL_OK)
        {
            /* 校验回复帧: 帧头=0xBB, XOR=BB^STATUS */
            if (reply_buf[0] == REPLY_HEADER &&
                reply_buf[2] == (reply_buf[0] ^ reply_buf[1]))
            {
                g_uartReply = reply_buf[1];
            }
            else
            {
                g_uartReply = REPLY_NORMAL;  /* 校验失败，按无缺陷处理 */
            }
        }
        else
        {
            g_uartReply = REPLY_NORMAL;  /* 超时：按无缺陷处理 */
        }

        /* 通知控制任务：回复已就绪 */
        osSemaphoreRelease(uartDoneSemHandle);
    }
}

/* USER CODE END Application */

