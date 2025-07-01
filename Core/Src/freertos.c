/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * File Name          : freertos.c
  * Description        : Code for freertos applications
  ******************************************************************************
  * @attention
  *
  * Copyright (c) 2025 STMicroelectronics.
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
#include <math.h>


#define M_PI 	3.1415926535f

#define ANGLE_MIN_DEG   10.0f
#define ANGLE_MAX_DEG   80.0f

#define ADC_MIN         10U
#define ADC_MAX         4096

#define ANGLE_MIN_RAD   (ANGLE_MIN_DEG * (M_PI / 180.0f))
#define ANGLE_MAX_RAD   (ANGLE_MAX_DEG * (M_PI / 180.0f))

#define Y_MIN           -0.9848077530f   // -cos(10∞)
#define Y_MAX           -0.1736481777f   // -cos(80∞)

/* USER CODE END Includes */

/* Private typedef -----------------------------------------------------------*/
/* USER CODE BEGIN PTD */

/* USER CODE END PTD */

/* Private define ------------------------------------------------------------*/
/* USER CODE BEGIN PD */
struct button_without_fix {
  GPIO_PinState pos_current;
  GPIO_PinState pos_previous;
  GPIO_PinState pos_out;
  GPIO_PinState pos_normal;
  uint16_t hold_counter;
};

typedef enum {
    Blynk_off,
    Blynk_right,
    Blynk_left,
    Blynk_warning,
} Blynk_types;
/* USER CODE END PD */

/* Private macro -------------------------------------------------------------*/
/* USER CODE BEGIN PM */

/* USER CODE END PM */

/* Private variables ---------------------------------------------------------*/
/* USER CODE BEGIN Variables */
static struct button_without_fix 
Fuel_level1_low={GPIO_PIN_SET,GPIO_PIN_SET,GPIO_PIN_RESET,GPIO_PIN_SET,0},
Fuel_level2_low={GPIO_PIN_SET,GPIO_PIN_SET,GPIO_PIN_RESET,GPIO_PIN_SET,0},
Left_in={GPIO_PIN_SET,GPIO_PIN_SET,GPIO_PIN_RESET,GPIO_PIN_SET,0},
Right_in={GPIO_PIN_SET,GPIO_PIN_SET,GPIO_PIN_RESET,GPIO_PIN_SET,0},
Warning_in={GPIO_PIN_SET,GPIO_PIN_SET,GPIO_PIN_RESET,GPIO_PIN_SET,0},
CAL={GPIO_PIN_SET,GPIO_PIN_SET,GPIO_PIN_RESET,GPIO_PIN_SET,0};
extern IWDG_HandleTypeDef hiwdg;
Blynk_types current_blynk_flag=Blynk_off; 
uint16_t adc_max=ADC_MAX;
uint16_t adc_min=ADC_MIN;
/* USER CODE END Variables */
/* Definitions for Product_IDLE */
osThreadId_t Product_IDLEHandle;
const osThreadAttr_t Product_IDLE_attributes = {
  .name = "Product_IDLE",
  .stack_size = 512 * 4,
  .priority = (osPriority_t) osPriorityNormal,
};
/* Definitions for Led_task */
osThreadId_t Led_taskHandle;
const osThreadAttr_t Led_task_attributes = {
  .name = "Led_task",
  .stack_size = 512 * 4,
  .priority = (osPriority_t) osPriorityBelowNormal5,
};
/* Definitions for Blynk */
osThreadId_t BlynkHandle;
const osThreadAttr_t Blynk_attributes = {
  .name = "Blynk",
  .stack_size = 128 * 4,
  .priority = (osPriority_t) osPriorityNormal3,
};

/* Private function prototypes -----------------------------------------------*/
/* USER CODE BEGIN FunctionPrototypes */
void buttin_proc_without_tim(struct button_without_fix *button,GPIO_TypeDef *GPIOx, uint16_t GPIO_Pin);
static uint8_t get_vertical_percent(uint16_t adc_value,uint16_t adc_max,uint16_t adc_min);
/* USER CODE END FunctionPrototypes */

void Start_Product_IDLE_Task(void *argument);
void Start_Led_task(void *argument);
void StartBlynkTask(void *argument);

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
  /* add queues, ... */
  /* USER CODE END RTOS_QUEUES */

  /* Create the thread(s) */
  /* creation of Product_IDLE */
  Product_IDLEHandle = osThreadNew(Start_Product_IDLE_Task, NULL, &Product_IDLE_attributes);

  /* creation of Led_task */
  Led_taskHandle = osThreadNew(Start_Led_task, NULL, &Led_task_attributes);

  /* creation of Blynk */
  BlynkHandle = osThreadNew(StartBlynkTask, NULL, &Blynk_attributes);

  /* USER CODE BEGIN RTOS_THREADS */
  /* add threads, ... */
  /* USER CODE END RTOS_THREADS */

  /* USER CODE BEGIN RTOS_EVENTS */
  /* add events, ... */
  /* USER CODE END RTOS_EVENTS */

}

/* USER CODE BEGIN Header_Start_Product_IDLE_Task */
/**
  * @brief  Function implementing the Product_IDLE thread.
  * @param  argument: Not used
  * @retval None
  */
/* USER CODE END Header_Start_Product_IDLE_Task */
void Start_Product_IDLE_Task(void *argument)
{
  /* USER CODE BEGIN Start_Product_IDLE_Task */
  /* Infinite loop */
  for(;;)
  {
    osDelay(10);
    HAL_IWDG_Refresh(&hiwdg);
    buttin_proc_without_tim(&Warning_in,Warning_in_GPIO_Port,Warning_in_Pin);
    buttin_proc_without_tim(&Right_in,Right_in_GPIO_Port,Right_in_Pin);
    buttin_proc_without_tim(&Left_in,Left_in_GPIO_Port,Left_in_Pin);
    buttin_proc_without_tim(&Fuel_level1_low,Fuel_level1_low_GPIO_Port,Fuel_level1_low_Pin);
    buttin_proc_without_tim(&Fuel_level2_low,Fuel_level2_low_GPIO_Port,Fuel_level2_low_Pin);
    buttin_proc_without_tim(&CAL,CAL_GPIO_Port,CAL_Pin);
    if(Right_in.pos_out){
      current_blynk_flag=Blynk_right;
    }
    if(Left_in.pos_out){
      current_blynk_flag=Blynk_left;
    }
    if(Warning_in.pos_out){
      current_blynk_flag=Blynk_warning;
    }
  }
  /* USER CODE END Start_Product_IDLE_Task */
}

/* USER CODE BEGIN Header_Start_Led_task */
/**
* @brief Function implementing the Led_task thread.
* @param argument: Not used
* @retval None
*/
/* USER CODE END Header_Start_Led_task */
void Start_Led_task(void *argument)
{
  /* USER CODE BEGIN Start_Led_task */
  /* Infinite loop */
  for(;;)
  {
    osDelay(1);
  }
  /* USER CODE END Start_Led_task */
}

/* USER CODE BEGIN Header_StartBlynkTask */
/**
* @brief Function implementing the Blynk thread.
* @param argument: Not used
* @retval None
*/
/* USER CODE END Header_StartBlynkTask */
void StartBlynkTask(void *argument)
{
  /* USER CODE BEGIN StartBlynkTask */
  uint8_t blynk_output=0;
  /* Infinite loop */
  for(;;)
  {
    osDelay(333); 
    switch (current_blynk_flag){
    case Blynk_off:
      blynk_output=0;
      HAL_GPIO_WritePin(Right_out_gpio_GPIO_Port,Right_out_gpio_Pin,GPIO_PIN_RESET);
      HAL_GPIO_WritePin(Left_out_GPIO_GPIO_Port,Left_out_GPIO_Pin,GPIO_PIN_RESET);
      break;
    case Blynk_left:
      if(blynk_output){
        blynk_output=0;
        HAL_GPIO_WritePin(Right_out_gpio_GPIO_Port,Right_out_gpio_Pin,GPIO_PIN_RESET);
        HAL_GPIO_WritePin(Left_out_GPIO_GPIO_Port,Left_out_GPIO_Pin,GPIO_PIN_RESET);
      }else{
        blynk_output=1;
        HAL_GPIO_WritePin(Right_out_gpio_GPIO_Port,Right_out_gpio_Pin,GPIO_PIN_RESET);
        HAL_GPIO_WritePin(Left_out_GPIO_GPIO_Port,Left_out_GPIO_Pin,GPIO_PIN_SET);
      }
      break;
    case Blynk_right:
      if(blynk_output){
        blynk_output=0;
        HAL_GPIO_WritePin(Right_out_gpio_GPIO_Port,Right_out_gpio_Pin,GPIO_PIN_RESET);
        HAL_GPIO_WritePin(Left_out_GPIO_GPIO_Port,Left_out_GPIO_Pin,GPIO_PIN_RESET);
      }else{
        blynk_output=1;
        HAL_GPIO_WritePin(Right_out_gpio_GPIO_Port,Right_out_gpio_Pin,GPIO_PIN_SET);
        HAL_GPIO_WritePin(Left_out_GPIO_GPIO_Port,Left_out_GPIO_Pin,GPIO_PIN_RESET);
      }
      break;
    case Blynk_warning:
      if(blynk_output){
        blynk_output=0;
        HAL_GPIO_WritePin(Right_out_gpio_GPIO_Port,Right_out_gpio_Pin,GPIO_PIN_RESET);
        HAL_GPIO_WritePin(Left_out_GPIO_GPIO_Port,Left_out_GPIO_Pin,GPIO_PIN_RESET);
      }else{
        blynk_output=1;
        HAL_GPIO_WritePin(Right_out_gpio_GPIO_Port,Right_out_gpio_Pin,GPIO_PIN_SET);
        HAL_GPIO_WritePin(Left_out_GPIO_GPIO_Port,Left_out_GPIO_Pin,GPIO_PIN_SET);
      }
      break;
    }
  }
  /* USER CODE END StartBlynkTask */
}

/* Private application code --------------------------------------------------*/
/* USER CODE BEGIN Application */
void buttin_proc_without_tim(struct button_without_fix *button,GPIO_TypeDef *GPIOx, uint16_t GPIO_Pin){
  button->pos_previous=button->pos_current;
  button->pos_current=HAL_GPIO_ReadPin(GPIOx,GPIO_Pin);
  if ((button->pos_previous==button->pos_current)&&(button->pos_current!=button->pos_normal)){
    button->pos_out=GPIO_PIN_SET;
  }else{
    button->pos_out=GPIO_PIN_RESET;
    button->hold_counter=0;
  }
}



// ‘ункци€ возвращает процент вертикального перемещени€ 0Ц100%
static uint8_t get_vertical_percent(uint16_t adc_value,uint16_t adc_max,uint16_t adc_min)
{
    // 1) ќграничиваем значение ј÷ѕ
    if (adc_value < adc_min) adc_value = adc_min;
    if (adc_value > adc_max) adc_value = adc_max;

    // 2) Ћинейна€ интерпол€ци€: получаем ? ? [angle_min_radЕangle_max_rad]
    float theta = ANGLE_MIN_RAD +
                  (ANGLE_MAX_RAD - ANGLE_MIN_RAD) *
                  ((float)(adc_value - adc_min) / (float)(adc_max - adc_min));

    // 3)  осинус Ч высота поплавка
    float y = -cosf(theta);

    // 4) Ќормализаци€ в диапазон [0Е1]
    float p = (y - Y_MIN) / (Y_MAX - Y_MIN);
    if (p < 0.0f) p = 0.0f;
    else if (p > 1.0f) p = 1.0f;

    // 5) ¬ процентах 0Ц100 с правильным округлением
    return (uint8_t)(p * 100.0f + 0.5f);
}
/* USER CODE END Application */

