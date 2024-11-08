/* USER CODE BEGIN Header */
/**
 ******************************************************************************
 * File Name          : freertos.c
 * Description        : Code for freertos applications
 ******************************************************************************
 * @attention
 *
 * Copyright (c) 2024 STMicroelectronics.
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
#include "adc.h"
#include "cmsis_os.h"
#include "main.h"
#include "task.h"
#include "tim.h"
/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */

/* USER CODE END Includes */

/* Private typedef -----------------------------------------------------------*/
/* USER CODE BEGIN PTD */

/* USER CODE END PTD */

/* Private define ------------------------------------------------------------*/
/* USER CODE BEGIN PD */

/* USER CODE END PD */

/* Private macro -------------------------------------------------------------*/
/* USER CODE BEGIN PM */
#define ADC_DIALA_INDEX 0
#define ADC_DIALB_INDEX 1
#define ADC_ALTMS_INDEX 2
#define ADC_MODE_INDEX  3
enum mode_e
{
    MODE_FREE,
    MODE_OFF,
    MODE_ALT,
};
struct status_bin
{
    /* data */
    int         knob_a_dial;
    int         knob_b_dial;
    enum mode_e mode_selset;
    int         mode_alt_ms;
    int         mode_alt_flag;
    int         mode_alt_cnt;
};
struct status_bin machine;
uint32_t          adc_value;
uint32_t          adc_array[5];
#define ADC_REF     3.3
#define DIVIDER_RES 100 * 1000.0
#define DIAL0_RES   0.0
#define DIAL1_RES   50 * 1000.0
#define DIAL2_RES   40 * 1000.0
#define DIAL3_RES   30 * 1000.0
#define DIAL4_RES   20 * 1000.0
#define DIAL5_RES   10 * 1000.0
#define DIAL6_RES   1.0
#define DIAL_RANGE  2 * 1000
/* USER CODE END PM */

/* Private variables ---------------------------------------------------------*/
/* USER CODE BEGIN Variables */

/* USER CODE END Variables */
osThreadId          defaultTaskHandle;
uint32_t            defaultTaskBuffer[128];
osStaticThreadDef_t defaultTaskControlBlock;
osThreadId          button_taskHandle;
uint32_t            myTask02Buffer[128];
osStaticThreadDef_t myTask02ControlBlock;
osThreadId          motor_taskHandle;
uint32_t            motor_taskBuffer[128];
osStaticThreadDef_t motor_taskControlBlock;

/* Private function prototypes -----------------------------------------------*/
/* USER CODE BEGIN FunctionPrototypes */

/* USER CODE END FunctionPrototypes */

void StartDefaultTask(void const* argument);
void StartTask02(void const* argument);
void StartTask03(void const* argument);

void MX_FREERTOS_Init(void); /* (MISRA C 2004 rule 8.1) */

/* GetIdleTaskMemory prototype (linked to static allocation support) */
void vApplicationGetIdleTaskMemory(StaticTask_t** ppxIdleTaskTCBBuffer, StackType_t** ppxIdleTaskStackBuffer, uint32_t* pulIdleTaskStackSize);

/* USER CODE BEGIN GET_IDLE_TASK_MEMORY */
static StaticTask_t xIdleTaskTCBBuffer;
static StackType_t  xIdleStack[configMINIMAL_STACK_SIZE];

void vApplicationGetIdleTaskMemory(StaticTask_t** ppxIdleTaskTCBBuffer, StackType_t** ppxIdleTaskStackBuffer, uint32_t* pulIdleTaskStackSize)
{
    *ppxIdleTaskTCBBuffer   = &xIdleTaskTCBBuffer;
    *ppxIdleTaskStackBuffer = &xIdleStack[0];
    *pulIdleTaskStackSize   = configMINIMAL_STACK_SIZE;
    /* place for user code */
}
/* USER CODE END GET_IDLE_TASK_MEMORY */

/**
 * @brief  FreeRTOS initialization
 * @param  None
 * @retval None
 */
void MX_FREERTOS_Init(void)
{
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
    /* add queues, ... */
    /* USER CODE END RTOS_QUEUES */

    /* Create the thread(s) */
    /* definition and creation of defaultTask */
    osThreadStaticDef(defaultTask, StartDefaultTask, osPriorityNormal, 0, 128, defaultTaskBuffer, &defaultTaskControlBlock);
    defaultTaskHandle = osThreadCreate(osThread(defaultTask), NULL);

    /* definition and creation of button_task */
    osThreadStaticDef(button_task, StartTask02, osPriorityRealtime, 0, 128, myTask02Buffer, &myTask02ControlBlock);
    button_taskHandle = osThreadCreate(osThread(button_task), NULL);

    /* definition and creation of motor_task */
    osThreadStaticDef(motor_task, StartTask03, osPriorityNormal, 0, 128, motor_taskBuffer, &motor_taskControlBlock);
    motor_taskHandle = osThreadCreate(osThread(motor_task), NULL);

    /* USER CODE BEGIN RTOS_THREADS */
    /* add threads, ... */
    /* USER CODE END RTOS_THREADS */
}

/* USER CODE BEGIN Header_StartDefaultTask */
/**
 * @brief  Function implementing the defaultTask thread.
 * @param  argument: Not used
 * @retval None
 */
/* USER CODE END Header_StartDefaultTask */
void StartDefaultTask(void const* argument)
{
    /* USER CODE BEGIN StartDefaultTask */
    /* Infinite loop */
    for (;;)
    {
        osDelay(1);
    }
    /* USER CODE END StartDefaultTask */
}

/* USER CODE BEGIN Header_StartTask02 */
/**
 * @brief Function implementing the button_task thread.
 * @param argument: Not used
 * @retval None
 */
/* USER CODE END Header_StartTask02 */
void StartTask02(void const* argument)
{
    /* USER CODE BEGIN StartTask02 */
    int i;
    /* Infinite loop */
    for (;;)
    {
        for (i = 0; i < 4; i++)
        {
            HAL_ADC_Start(&hadc1);
            HAL_ADC_PollForConversion(&hadc1, 0xffff);
            adc_value    = HAL_ADC_GetValue(&hadc1);
            adc_array[i] = adc_value;
        }
        HAL_ADC_Stop(&hadc1);
        if (adc_array[0] < DIVIDER_RES / (DIAL1_RES - DIAL_RANGE + DIVIDER_RES) * 4096.0)
        {
            machine.knob_a_dial = 0;
        }
        else if (adc_array[0] < DIVIDER_RES / (DIAL2_RES - DIAL_RANGE + DIVIDER_RES) * 4096.0)
        {
            machine.knob_a_dial = 1;
        }
        else if (adc_array[0] < DIVIDER_RES / (DIAL3_RES - DIAL_RANGE + DIVIDER_RES) * 4096.0)
        {
            machine.knob_a_dial = 2;
        }
        else if (adc_array[0] < DIVIDER_RES / (DIAL4_RES - DIAL_RANGE + DIVIDER_RES) * 4096.0)
        {
            machine.knob_a_dial = 3;
        }
        else if (adc_array[0] < DIVIDER_RES / (DIAL5_RES - DIAL_RANGE + DIVIDER_RES) * 4096.0)
        {
            machine.knob_a_dial = 4;
        }
        else if (adc_array[0] < DIVIDER_RES / (DIAL_RANGE + DIVIDER_RES) * 4096.0)
        {
            machine.knob_a_dial = 5;
        }
        else if (adc_array[0] > DIVIDER_RES / (DIAL_RANGE + DIVIDER_RES) * 4096.0)
        {
            machine.knob_a_dial = 6;
        }
        //===============================================================================
        if (adc_array[1] < DIVIDER_RES / (DIAL1_RES - DIAL_RANGE + DIVIDER_RES) * 4096.0)
        {
            machine.knob_b_dial = 0;
        }
        else if (adc_array[1] < DIVIDER_RES / (DIAL2_RES - DIAL_RANGE + DIVIDER_RES) * 4096.0)
        {
            machine.knob_b_dial = 1;
        }
        else if (adc_array[1] < DIVIDER_RES / (DIAL3_RES - DIAL_RANGE + DIVIDER_RES) * 4096.0)
        {
            machine.knob_b_dial = 2;
        }
        else if (adc_array[1] < DIVIDER_RES / (DIAL4_RES - DIAL_RANGE + DIVIDER_RES) * 4096.0)
        {
            machine.knob_b_dial = 3;
        }
        else if (adc_array[1] < DIVIDER_RES / (DIAL5_RES - DIAL_RANGE + DIVIDER_RES) * 4096.0)
        {
            machine.knob_b_dial = 4;
        }
        else if (adc_array[1] < DIVIDER_RES / (DIAL_RANGE + DIVIDER_RES) * 4096.0)
        {
            machine.knob_b_dial = 5;
        }
        else if (adc_array[1] > DIVIDER_RES / (DIAL_RANGE + DIVIDER_RES) * 4096.0)
        {
            machine.knob_b_dial = 6;
        }
        //===============================================================================
        if (adc_array[ADC_ALTMS_INDEX] < DIVIDER_RES / (DIAL1_RES - DIAL_RANGE + DIVIDER_RES) * 4096.0)
        {
            machine.mode_alt_ms = 3 * 1000.0 + 0 * 3.8;
        }
        else if (adc_array[ADC_ALTMS_INDEX] < DIVIDER_RES / (DIAL2_RES - DIAL_RANGE + DIVIDER_RES) * 4096.0)
        {
            machine.mode_alt_ms = 3 * 1000.0 + 1 * 3.8;
        }
        else if (adc_array[ADC_ALTMS_INDEX] < DIVIDER_RES / (DIAL3_RES - DIAL_RANGE + DIVIDER_RES) * 4096.0)
        {
            machine.mode_alt_ms = 3 * 1000.0 + 2 * 3.8;
        }
        else if (adc_array[ADC_ALTMS_INDEX] < DIVIDER_RES / (DIAL4_RES - DIAL_RANGE + DIVIDER_RES) * 4096.0)
        {
            machine.mode_alt_ms = 3 * 1000.0 + 3 * 3.8;
        }
        else if (adc_array[ADC_ALTMS_INDEX] < DIVIDER_RES / (DIAL5_RES - DIAL_RANGE + DIVIDER_RES) * 4096.0)
        {
            machine.mode_alt_ms = 3 * 1000.0 + 4 * 3.8;
        }
        else if (adc_array[ADC_ALTMS_INDEX] < DIVIDER_RES / (DIAL_RANGE + DIVIDER_RES) * 4096.0)
        {
            machine.mode_alt_ms = 3 * 1000.0 + 5 * 3.8;
        }
        else if (adc_array[ADC_ALTMS_INDEX] > DIVIDER_RES / (DIAL_RANGE + DIVIDER_RES) * 4096.0)
        {
            machine.mode_alt_ms = 3 * 1000.0 + 6 * 3.8;
        }
        //===============================================================================
        if (adc_array[ADC_MODE_INDEX] < DIVIDER_RES / (DIAL5_RES - DIAL_RANGE + DIVIDER_RES) * 4096.0)
        {
            machine.mode_selset = MODE_OFF;
        }
        else if (adc_array[ADC_ALTMS_INDEX] < DIVIDER_RES / (DIAL_RANGE + DIVIDER_RES) * 4096.0)
        {
            machine.mode_selset = MODE_FREE;
        }
        else if (adc_array[ADC_ALTMS_INDEX] > DIVIDER_RES / (DIAL_RANGE + DIVIDER_RES) * 4096.0)
        {
            machine.mode_selset   = MODE_ALT;
            machine.mode_alt_cnt  = 0;
            machine.mode_alt_flag = 0;
        }
        osDelay(1000);
    }
    /* USER CODE END StartTask02 */
}

/* USER CODE BEGIN Header_StartTask03 */
/**
 * @brief Function implementing the motor_task thread.
 * @param argument: Not used
 * @retval None
 */
/* USER CODE END Header_StartTask03 */
void StartTask03(void const* argument)
{
    /* USER CODE BEGIN StartTask03 */
    HAL_TIM_PWM_Start(&htim3, TIM_CHANNEL_3);
    HAL_TIM_PWM_Start(&htim3, TIM_CHANNEL_4);
    /* Infinite loop */
    for (;;)
    {
        if (machine.mode_selset == MODE_FREE)
        {
            //===================================================
            if (machine.knob_a_dial == 1)
            {
                __HAL_TIM_SET_COMPARE(&htim3, TIM_CHANNEL_3, 321);
            }
            else if (machine.knob_a_dial == 2)
            {
                __HAL_TIM_SET_COMPARE(&htim3, TIM_CHANNEL_3, 503);
            }
            else if (machine.knob_a_dial == 3)
            {
                __HAL_TIM_SET_COMPARE(&htim3, TIM_CHANNEL_3, 700);
            }
            else if (machine.knob_a_dial == 4)
            {
                __HAL_TIM_SET_COMPARE(&htim3, TIM_CHANNEL_3, 793);
            }
            else if (machine.knob_a_dial == 5)
            {
                __HAL_TIM_SET_COMPARE(&htim3, TIM_CHANNEL_3, 970);
            }
            else
            {
                __HAL_TIM_SET_COMPARE(&htim3, TIM_CHANNEL_3, 000);
            }
            //===================================================
            if (machine.knob_b_dial == 1)
            {
                __HAL_TIM_SET_COMPARE(&htim3, TIM_CHANNEL_4, 321);
            }
            else if (machine.knob_b_dial == 2)
            {
                __HAL_TIM_SET_COMPARE(&htim3, TIM_CHANNEL_4, 503);
            }
            else if (machine.knob_b_dial == 3)
            {
                __HAL_TIM_SET_COMPARE(&htim3, TIM_CHANNEL_4, 700);
            }
            else if (machine.knob_b_dial == 4)
            {
                __HAL_TIM_SET_COMPARE(&htim3, TIM_CHANNEL_4, 793);
            }
            else if (machine.knob_b_dial == 5)
            {
                __HAL_TIM_SET_COMPARE(&htim3, TIM_CHANNEL_4, 970);
            }
            else
            {
                __HAL_TIM_SET_COMPARE(&htim3, TIM_CHANNEL_4, 000);
            }
        }
        else if (machine.mode_selset == MODE_OFF)
        {
            __HAL_TIM_SET_COMPARE(&htim3, TIM_CHANNEL_4, 000);
            __HAL_TIM_SET_COMPARE(&htim3, TIM_CHANNEL_3, 000);
        }
        else if (machine.mode_selset == MODE_ALT)
        {
            if (machine.mode_alt_flag == 0)
            {
                if (machine.knob_a_dial == 1)
                {
                    __HAL_TIM_SET_COMPARE(&htim3, TIM_CHANNEL_3, 321);
                }
                else if (machine.knob_a_dial == 2)
                {
                    __HAL_TIM_SET_COMPARE(&htim3, TIM_CHANNEL_3, 503);
                }
                else if (machine.knob_a_dial == 3)
                {
                    __HAL_TIM_SET_COMPARE(&htim3, TIM_CHANNEL_3, 700);
                }
                else if (machine.knob_a_dial == 4)
                {
                    __HAL_TIM_SET_COMPARE(&htim3, TIM_CHANNEL_3, 793);
                }
                else if (machine.knob_a_dial == 5)
                {
                    __HAL_TIM_SET_COMPARE(&htim3, TIM_CHANNEL_3, 970);
                }
                else
                {
                    __HAL_TIM_SET_COMPARE(&htim3, TIM_CHANNEL_3, 000);
                }
                __HAL_TIM_SET_COMPARE(&htim3, TIM_CHANNEL_4, 000);
                if (machine.mode_alt_cnt > machine.mode_alt_ms / 20)
                {
                    machine.mode_alt_flag = 1;
                    machine.mode_alt_cnt  = 0;
                }
                machine.mode_alt_cnt++;
            }
            else
            {
                if (machine.knob_b_dial == 1)
                {
                    __HAL_TIM_SET_COMPARE(&htim3, TIM_CHANNEL_4, 321);
                }
                else if (machine.knob_b_dial == 2)
                {
                    __HAL_TIM_SET_COMPARE(&htim3, TIM_CHANNEL_4, 503);
                }
                else if (machine.knob_b_dial == 3)
                {
                    __HAL_TIM_SET_COMPARE(&htim3, TIM_CHANNEL_4, 700);
                }
                else if (machine.knob_b_dial == 4)
                {
                    __HAL_TIM_SET_COMPARE(&htim3, TIM_CHANNEL_4, 793);
                }
                else if (machine.knob_b_dial == 5)
                {
                    __HAL_TIM_SET_COMPARE(&htim3, TIM_CHANNEL_4, 970);
                }
                else
                {
                    __HAL_TIM_SET_COMPARE(&htim3, TIM_CHANNEL_4, 000);
                }
                __HAL_TIM_SET_COMPARE(&htim3, TIM_CHANNEL_3, 000);
            }
            if (machine.mode_alt_cnt > machine.mode_alt_ms / 20)
            {
                machine.mode_alt_flag = 0;
                machine.mode_alt_cnt  = 0;
            }
            machine.mode_alt_cnt++;
        }
        osDelay(pdMS_TO_TICKS(20));
    }
    /* USER CODE END StartTask03 */
}

/* Private application code --------------------------------------------------*/
/* USER CODE BEGIN Application */

/* USER CODE END Application */
