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
#include "stm32f1xx_hal_flash.h" // Add this include for FLASH_EraseInitTypeDef

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */
#include <math.h>
#include "sh1106.h"
#include "stdio.h"

#ifndef M_PI
#define M_PI 	3.1415926535f
#endif

#define ANGLE_MIN_DEG   10.0f
#define ANGLE_MAX_DEG   80.0f

#define ADC_MIN         10U
#define ADC_MAX         4096

#define ANGLE_MIN_RAD   (ANGLE_MIN_DEG * (M_PI / 180.0f))
#define Y_MIN           -0.9848077530f   // -cos(10 deg)

#define Y_MIN           -0.9848077530f   // -cos(10�)
#define Y_MAX           -0.1736481777f   // -cos(80�)


#define ADC_CHANNEL_COUNT 3
#define ADC_AVG_DEPTH     10

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


typedef struct
{
    uint16_t adc_raw;     // Текущее значение с АЦП
    int16_t  value;       // Пересчитанное значение
    uint16_t adc_min;     // Минимальное значение АЦП (калибровка)
    uint16_t adc_max;     // Максимальное значение АЦП (калибровка)
    int16_t  value_min;   // Минимум выходного значения (например, -1000)
    int16_t  value_max;   // Максимум выходного значения (например, +1000)
} AnalogSensor_t;

void Flash_read();
uint32_t Flash_write();
FLASH_EraseInitTypeDef Erase;
void DefaultAnalogSensors(void);
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
extern ADC_HandleTypeDef hadc1;
Blynk_types current_blynk_flag=Blynk_off; 
uint16_t adc_max=ADC_MAX;
uint16_t adc_min=ADC_MIN;
AnalogSensor_t Voltage;
AnalogSensor_t Level1_ai;
AnalogSensor_t Level2_ai;
static uint16_t ADC_dma[ADC_CHANNEL_COUNT];                     // Данные от DMA
static uint16_t adc_history[ADC_CHANNEL_COUNT][ADC_AVG_DEPTH]; // История выборок
static uint8_t  adc_index = 0;                                  // Индекс текущей выборки
static uint16_t adc_filtered[ADC_CHANNEL_COUNT];               // Усреднённые значения
uint8_t blynk_output=0;
static uint8_t cal_ongoing_flag=0;

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
static inline uint32_t flash_read(uint32_t address) {
    return *(volatile uint32_t*)address;
}
void buttin_proc_without_tim(struct button_without_fix *button,GPIO_TypeDef *GPIOx, uint16_t GPIO_Pin);
//static uint8_t get_vertical_percent(uint16_t adc_value,uint16_t adc_max,uint16_t adc_min);
void AnalogSensor_StartCalibration(AnalogSensor_t* sensor);
void AnalogSensor_CalibrateMinMax(AnalogSensor_t* sensor, uint16_t new_adc_value);
void AnalogSensor_Update(AnalogSensor_t* sensor, uint16_t new_adc_value);
void ADC_ProcessNewSamples(void);

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
  
  Flash_read();
  /* Infinite loop */
  for(;;)
  {
    HAL_ADC_Stop_DMA(&hadc1);
    HAL_ADC_Start_DMA(&hadc1,(uint32_t*)&ADC_dma,3);
    osDelay(10);
    ADC_ProcessNewSamples();
    AnalogSensor_Update(&Level1_ai,adc_filtered[0]);
    AnalogSensor_Update(&Level2_ai,adc_filtered[1]);
    AnalogSensor_Update(&Voltage,adc_filtered[2]);
    HAL_IWDG_Refresh(&hiwdg);
    buttin_proc_without_tim(&Warning_in,Warning_in_GPIO_Port,Warning_in_Pin);
    buttin_proc_without_tim(&Right_in,Right_in_GPIO_Port,Right_in_Pin);
    buttin_proc_without_tim(&Left_in,Left_in_GPIO_Port,Left_in_Pin);
    buttin_proc_without_tim(&Fuel_level1_low,Fuel_level1_low_GPIO_Port,Fuel_level1_low_Pin);
    buttin_proc_without_tim(&Fuel_level2_low,Fuel_level2_low_GPIO_Port,Fuel_level2_low_Pin);
    buttin_proc_without_tim(&CAL,CAL_GPIO_Port,CAL_Pin);
    current_blynk_flag=Blynk_off; // Сброс флага Blynk
    if(Right_in.pos_out){
      current_blynk_flag=Blynk_right;
    }
    if(Left_in.pos_out){
      current_blynk_flag=Blynk_left;
    }
    if(Warning_in.pos_out){
      current_blynk_flag=Blynk_warning;
    }
    
    if(CAL.hold_counter>1000){
      if(cal_ongoing_flag){
        AnalogSensor_CalibrateMinMax(&Level1_ai,adc_filtered[0]);
        AnalogSensor_CalibrateMinMax(&Level2_ai,adc_filtered[1]);
      }else{
        cal_ongoing_flag=1;
        AnalogSensor_StartCalibration(&Level1_ai);
        AnalogSensor_StartCalibration(&Level2_ai);
      }
    }else if(cal_ongoing_flag){
      cal_ongoing_flag=0;
      Flash_write();
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
     //----------Horse---------------
  ssd1306_Init();
  ssd1306_Fill(Black);
  ssd1306_UpdateScreen();
  // Display the start screen at multiple cursor positions for visual effect or initialization sequence.
  startScreen();
  ssd1306_Fill(Black);
  ssd1306_SetCursor(32,16); 
  startScreen();
  ssd1306_Fill(Black);
  ssd1306_SetCursor(64,32); 
  startScreen();
  ssd1306_Fill(Black);
  ssd1306_SetCursor(96,16); 
  startScreen();
  ssd1306_Fill(Black);
  //------------------------------------
  /* Infinite loop */
  for(;;)
  {
    osDelay(100);

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
/**
 * @brief  Processes the state of a button without using a timer.
 * @param  button: Pointer to the button structure to update.
 * @param  GPIOx: GPIO port where the button is connected.
 * @param  GPIO_Pin: GPIO pin number for the button.
 * @note   Updates the button state, output, and hold counter based on pin readings.
 */
void buttin_proc_without_tim(struct button_without_fix *button,GPIO_TypeDef *GPIOx, uint16_t GPIO_Pin){
  button->pos_previous=button->pos_current;
  button->pos_current=HAL_GPIO_ReadPin(GPIOx,GPIO_Pin);
  if ((button->pos_previous==button->pos_current)&&(button->pos_current!=button->pos_normal)){
    button->pos_out=GPIO_PIN_SET;
    if (button->hold_counter < 65534)
      button->hold_counter++;
    
  }else{
    button->pos_out=GPIO_PIN_RESET;
    button->hold_counter=0;
  }
}



// линеаризация данных с АЦП
void AnalogSensor_Update(AnalogSensor_t* sensor, uint16_t new_adc_value)
{
    sensor->adc_raw = new_adc_value;

    if (sensor->adc_max <= sensor->adc_min)
    {
        sensor->value = 0; // Ошибка калибровки
        return;
    }

    // Линейная интерполяция
    int32_t scaled = (int32_t)(new_adc_value - sensor->adc_min) *
                     (sensor->value_max - sensor->value_min);
    scaled /= (sensor->adc_max - sensor->adc_min);
    scaled += sensor->value_min;

    // Ограничим в пределах value_min и value_max
    if (scaled < sensor->value_min) scaled = sensor->value_min;
    if (scaled > sensor->value_max) scaled = sensor->value_max;

    sensor->value = (int16_t)scaled;
}

void AnalogSensor_CalibrateMinMax(AnalogSensor_t* sensor, uint16_t new_adc_value)
{
    if (new_adc_value < sensor->adc_min || sensor->adc_min == 0xFFFF)
    {
        sensor->adc_min = new_adc_value;
    }

    if (new_adc_value > sensor->adc_max)
    {
        sensor->adc_max = new_adc_value;
    }
}

void AnalogSensor_StartCalibration(AnalogSensor_t* sensor)
{
  if(sensor->adc_min<3000){
    sensor->adc_min+= 1000;       // максимально возможное значение для старта поиска минимума
  }
  if(sensor->adc_max>1500){
    sensor->adc_max-= 1000;      // минимально возможное значение для старта поиска максимума
  }
}


uint32_t Flash_write(){
  taskENTER_CRITICAL();
  uint32_t flash_ret;
  HAL_FLASH_Unlock();
  Erase.TypeErase=FLASH_TYPEERASE_PAGES;
  Erase.PageAddress=User_Page_Adress[0];
  Erase.NbPages=1;  //1kBytes
  //  Delay_switching backlight_on tooth_sp Deept_of_cut_mm Deept_of_cut_pulses M1
  if (HAL_FLASHEx_Erase(&Erase, &flash_ret) != HAL_OK) {
    HAL_FLASH_Lock();
    taskEXIT_CRITICAL();
    
    return flash_ret;
  }
  // Упаковка данных кнопок (9 бит)
  
  // Запись данных
  HAL_FLASH_Program(FLASH_TYPEPROGRAM_WORD, User_Page_Adress[0], Voltage.adc_max<<16|Voltage.adc_min);
  HAL_FLASH_Program(FLASH_TYPEPROGRAM_WORD, User_Page_Adress[1], Voltage.value_max<<16|Voltage.value_min); 
  
  HAL_FLASH_Program(FLASH_TYPEPROGRAM_WORD, User_Page_Adress[2], Level1_ai.adc_max<<16|Level1_ai.adc_min);
  HAL_FLASH_Program(FLASH_TYPEPROGRAM_WORD, User_Page_Adress[3], Level1_ai.value_max<<16|Level1_ai.value_min);
  
  HAL_FLASH_Program(FLASH_TYPEPROGRAM_WORD, User_Page_Adress[4], Level2_ai.adc_max<<16|Level2_ai.adc_min);
  HAL_FLASH_Program(FLASH_TYPEPROGRAM_WORD, User_Page_Adress[5], Level2_ai.value_max<<16|Level2_ai.value_min);
  
  HAL_FLASH_Lock();
  taskEXIT_CRITICAL();
  return 0xFFFFFFFF; // Успешная запись
  
}

void Flash_read(void) {
    uint32_t temp;

    // Проверка: если первая ячейка пустая — загрузить дефолтные значения
    temp = flash_read(User_Page_Adress[0]);
    if (temp == 0xFFFFFFFF) {
        DefaultAnalogSensors();
        return;
    }

    // ---- Voltage ----
    Voltage.adc_max   = (temp >> 16) & 0xFFFF;
    Voltage.adc_min   = temp & 0xFFFF;

    temp = flash_read(User_Page_Adress[1]);
    Voltage.value_max = (temp >> 16) & 0xFFFF;
    Voltage.value_min = temp & 0xFFFF;

    // ---- Level1_ai ----
    temp = flash_read(User_Page_Adress[2]);
    Level1_ai.adc_max   = (temp >> 16) & 0xFFFF;
    Level1_ai.adc_min   = temp & 0xFFFF;

    temp = flash_read(User_Page_Adress[3]);
    Level1_ai.value_max = (temp >> 16) & 0xFFFF;
    Level1_ai.value_min = temp & 0xFFFF;

    // ---- Level2_ai ----
    temp = flash_read(User_Page_Adress[4]);
    Level2_ai.adc_max   = (temp >> 16) & 0xFFFF;
    Level2_ai.adc_min   = temp & 0xFFFF;

    temp = flash_read(User_Page_Adress[5]);
    Level2_ai.value_max = (temp >> 16) & 0xFFFF;
    Level2_ai.value_min = temp & 0xFFFF;
}

void DefaultAnalogSensors(void)
{
    Voltage.adc_min   = 200;
    Voltage.adc_max   = 3900;
    Voltage.value_min = 0;
    Voltage.value_max = 1500;
    Voltage.adc_raw   = 0;
    Voltage.value     = 0;

    Level1_ai.adc_min   = 200;
    Level1_ai.adc_max   = 3900;
    Level1_ai.value_min = 0;
    Level1_ai.value_max = 100;
    Level1_ai.adc_raw   = 0;
    Level1_ai.value     = 0;

    Level2_ai.adc_min   = 200;
    Level2_ai.adc_max   = 3900;
    Level2_ai.value_min = 0;
    Level2_ai.value_max = 100;
    Level2_ai.adc_raw   = 0;
    Level2_ai.value     = 0;
}


void ADC_ProcessNewSamples(void)
{
    for (int ch = 0; ch < ADC_CHANNEL_COUNT; ++ch)
    {
        adc_history[ch][adc_index] = ADC_dma[ch];
    }

    adc_index++;
    if (adc_index >= ADC_AVG_DEPTH)
        adc_index = 0;

    // После накопления всех выборок можно усреднять
    for (int ch = 0; ch < ADC_CHANNEL_COUNT; ++ch)
    {
        uint32_t sum = 0;
        for (int i = 0; i < ADC_AVG_DEPTH; ++i)
        {
            sum += adc_history[ch][i];
        }
        adc_filtered[ch] = sum / ADC_AVG_DEPTH;
    }
}

void DisplayLevelsAndStatus(AnalogSensor_t* level1, AnalogSensor_t* level2, AnalogSensor_t* voltage, Blynk_types blynk_flag, uint8_t calibration_mode)
{
    char buf[20];

    ssd1306_Fill(Black);

    // Линия 1
    ssd1306_SetCursor(0, 0);
    if (cal_ongoing_flag)
        sprintf(buf, "L1: CAL");
    else
    {
        int perc = level1->value;  // проценты уже есть
        if (perc < 0) perc = 0;
        if (perc > 100) perc = 100;
        sprintf(buf, "L1: %d%%", perc);
    }
    ssd1306_WriteString(buf, Font_11x18, White);

    // Линия 2
    ssd1306_SetCursor(0, 32);
    if (cal_ongoing_flag)
        sprintf(buf, "L2: CAL");
    else
    {
        int perc = level2->value;
        if (perc < 0) perc = 0;
        if (perc > 100) perc = 100;
        sprintf(buf, "L2: %d%%", perc);
    }
    ssd1306_WriteString(buf, Font_11x18, White);

    // Справа — напряжение или иконка поворотника
    if (blynk_flag == Blynk_off)
    {
        int volt_perc = voltage->value;
        if (volt_perc < 0) volt_perc = 0;
        if (volt_perc > 100) volt_perc = 100;
        sprintf(buf, "%d%%", volt_perc);
        ssd1306_SetCursor(90, 0);
        ssd1306_WriteString(buf, Font_11x18, White);
    }
    else
    {
        ssd1306_SetCursor(90, 0);
        switch (blynk_flag)
        {
            case Blynk_right:
                DrawArrowRight(White);
                break;
            case Blynk_left:
                DrawArrowLeft(White);
                break;
            case Blynk_warning:
                DrawWarningTriangle(White);
                break;
            default:
                break;
        }
    }

    ssd1306_UpdateScreen();
}

/* USER CODE END Application */

