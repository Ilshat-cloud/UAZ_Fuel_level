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
#include "sh1106.h"
#include "stdio.h"
#include "string.h"
#include "i2c.h"

#ifndef M_PI
#define M_PI 	3.1415926535f
#endif

#define PRECENT
#define ANGLE_MIN_DEG   10.0f
#define ANGLE_MAX_DEG   80.0f

#define ANGLE_MIN_RAD   (ANGLE_MIN_DEG * (M_PI / 180.0f))
#define Y_MIN           -0.9848077530f   // -cos(10 deg)

#define Y_MIN           -0.9848077530f   // -cos(10�)
#define Y_MAX           -0.1736481777f   // -cos(80�)


#define ADC_CHANNEL_COUNT 3
#define ADC_AVG_DEPTH     10

#define btn_ll Fuel_level1_low
#define btn_hh Fuel_level2_low
#define btn_auto Left_in
#define btn_on Right_in
#define RELAY_OUTPUT_PIN Left_out_GPIO_Pin
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

typedef enum {
    LEVEL_OFF = 0,
    LEVEL_AUTO,
    LEVEL_ON
} Level_t;

typedef enum {
    AUTO_STATE_WAITING = 0,
    AUTO_STATE_LL_TRIGGERED,
    AUTO_STATE_HH_TRIGGERED,
    AUTO_STATE_ERROR
} Auto_state_t;

typedef struct
{
    uint16_t adc_raw;     // Текущее значение с АЦП
    int16_t  value;       // Пересчитанное значение
    uint16_t adc_min;     // Минимальное значение АЦП (калибровка)
    uint16_t adc_max;     // Максимальное значение АЦП (калибровка)
    int16_t  value_min;   // Минимум выходного значения (например, -1000)
    int16_t  value_max;   // Максимум выходного значения (например, +1000)
} AnalogSensor_t;

FLASH_EraseInitTypeDef Erase;
void DefaultAnalogSensors(void);
/* USER CODE END PD */

/* Private macro -------------------------------------------------------------*/
/* USER CODE BEGIN PM */

/* USER CODE END PM */

/* Private variables ---------------------------------------------------------*/
/* USER CODE BEGIN Variables */
extern SSD1306_t SSD1306;
extern I2C_HandleTypeDef hi2c1;
extern IWDG_HandleTypeDef hiwdg;
extern ADC_HandleTypeDef hadc1;

static struct button_without_fix 
Fuel_level1_low={GPIO_PIN_SET,GPIO_PIN_SET,GPIO_PIN_RESET,GPIO_PIN_SET,0},
Fuel_level2_low={GPIO_PIN_SET,GPIO_PIN_SET,GPIO_PIN_RESET,GPIO_PIN_SET,0},
Left_in={GPIO_PIN_SET,GPIO_PIN_SET,GPIO_PIN_RESET,GPIO_PIN_SET,0},
Right_in={GPIO_PIN_SET,GPIO_PIN_SET,GPIO_PIN_RESET,GPIO_PIN_SET,0};

Blynk_types current_blynk_flag=Blynk_off;
volatile Level_t current_ctrl = LEVEL_AUTO;
volatile Auto_state_t auto_state = AUTO_STATE_WAITING;
AnalogSensor_t Voltage;
static uint16_t ADC_dma[ADC_CHANNEL_COUNT];
static uint16_t adc_history[ADC_CHANNEL_COUNT][ADC_AVG_DEPTH];
static uint8_t  adc_index = 0;
static uint16_t adc_filtered[ADC_CHANNEL_COUNT];

uint8_t blynk_output=0;

/* USER CODE END Variables */
/* Definitions for Product_IDLE */
osThreadId_t Product_IDLEHandle;
const osThreadAttr_t Product_IDLE_attributes = {
  .name = "Product_IDLE",
  .stack_size = 256 * 4,
  .priority = (osPriority_t) osPriorityNormal,
};
/* Definitions for Led_task */
osThreadId_t Led_taskHandle;
const osThreadAttr_t Led_task_attributes = {
  .name = "Led_task",
  .stack_size = 512 * 4,
  .priority = (osPriority_t) osPriorityBelowNormal,
};
/* Definitions for Blynk */
osThreadId_t BlynkHandle;
const osThreadAttr_t Blynk_attributes = {
  .name = "Blynk",
  .stack_size = 128 * 4,
  .priority = (osPriority_t) osPriorityNormal,
};

/* Private function prototypes -----------------------------------------------*/
/* USER CODE BEGIN FunctionPrototypes */

void buttin_proc_without_tim(struct button_without_fix *button,GPIO_TypeDef *GPIOx, uint16_t GPIO_Pin);
void ADC_ProcessNewSamples(void);
void AnalogSensor_Update(AnalogSensor_t* sensor, uint16_t new_adc_value);

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
  HAL_ADCEx_Calibration_Start(&hadc1);
  
  for(;;)
  {
    // Запуск АЦП для замера напряжения платы
    HAL_ADC_Stop_DMA(&hadc1);
    HAL_ADC_Start_DMA(&hadc1, (uint32_t*)&ADC_dma, 3);
    osDelay(10); // Период опроса 10мс идеально подходит для buttin_proc
    
    ADC_ProcessNewSamples();
    AnalogSensor_Update(&Voltage, adc_filtered[2]);
    HAL_IWDG_Refresh(&hiwdg);
    
    // Опрашиваем физические пины старой платы с родной логикой антидребезга
    buttin_proc_without_tim(&btn_ll, Fuel_level1_low_GPIO_Port, Fuel_level1_low_Pin);
    buttin_proc_without_tim(&btn_hh, Fuel_level2_low_GPIO_Port, Fuel_level2_low_Pin);
    buttin_proc_without_tim(&btn_auto, Left_in_GPIO_Port, Left_in_Pin);
    buttin_proc_without_tim(&btn_on, Right_in_GPIO_Port, Right_in_Pin);

    // Логика выбора режима (активное состояние кнопок перенесено на pos_out)
    // В исходном коде pos_out взводится в SET, когда состояние отлично от pos_normal (нажато)
    if (btn_on.pos_out == GPIO_PIN_SET) {
        current_ctrl = LEVEL_ON;
    } else if (btn_auto.pos_out == GPIO_PIN_SET) {
        current_ctrl = LEVEL_AUTO;
    } else {
        current_ctrl = LEVEL_OFF;
    }

    // Конечный автомат логики реле и ошибок
    switch (current_ctrl) {
        case LEVEL_OFF:
            HAL_GPIO_WritePin(Left_out_GPIO_GPIO_Port, Left_out_GPIO_Pin, GPIO_PIN_RESET); // Выключить Реле
            auto_state = AUTO_STATE_WAITING;
            break;
            
        case LEVEL_AUTO:
            // Внимание: проверка аномалии уровней. Нажатие кнопок определяется через GPIO_PIN_SET на выходе pos_out
            if ((btn_ll.pos_out != GPIO_PIN_SET) && (btn_hh.pos_out == GPIO_PIN_SET)) {
                auto_state = AUTO_STATE_ERROR; // Ошибка: верхний датчик сработал, а нижний нет
            } else if (btn_hh.pos_out == GPIO_PIN_SET) { 
                auto_state = AUTO_STATE_HH_TRIGGERED;
            } else if (btn_ll.pos_out == GPIO_PIN_SET) {
                auto_state = AUTO_STATE_LL_TRIGGERED;   
            } else {
                auto_state = AUTO_STATE_WAITING;
                
            }

            switch (auto_state) {
                case AUTO_STATE_WAITING:
                  HAL_GPIO_WritePin(Left_out_GPIO_GPIO_Port, Left_out_GPIO_Pin, GPIO_PIN_RESET);
                case AUTO_STATE_LL_TRIGGERED:
                    // В этих состояниях в режиме AUTO насос/реле молчит или ждет
                    break;
                case AUTO_STATE_HH_TRIGGERED:
                    HAL_GPIO_WritePin(Left_out_GPIO_GPIO_Port, Left_out_GPIO_Pin, GPIO_PIN_SET); // Включаем реле
                    break;
                case AUTO_STATE_ERROR:
                    HAL_GPIO_WritePin(Left_out_GPIO_GPIO_Port, Left_out_GPIO_Pin, GPIO_PIN_RESET); // Выключаем при ошибке
                    break;
            }
            break;
            
        case LEVEL_ON:
            HAL_GPIO_WritePin(Left_out_GPIO_GPIO_Port, Left_out_GPIO_Pin, GPIO_PIN_SET); // Принудительно включаем реле
            break;
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
  char buf[32];
  uint8_t buffer[240];
  char numbuf[10];
  ssd1306_HardResetAndReinit();
  ssd1306_Fill(Black);
  ssd1306_UpdateScreen();
    // Координаты для анимации startScreen
  const uint8_t anim_coords[4][2] = {{0, 32}, {32, 16}, {64, 32}, {96, 16}};
  //----------Horse---------------
  
  ssd1306_HardResetAndReinit();
  ssd1306_Fill(Black);
  ssd1306_UpdateScreen();
  uint16_t id=(uint16_t)HAL_GetUIDw0();
  // Display the start screen at multiple cursor positions for visual effect or initialization sequence.
  snprintf(numbuf, 5, "%d", id);
  
  for (int i = 0; i < 4; i++) {
    ssd1306_Fill(Black);
    ssd1306_SetCursor(0, 0);
    ssd1306_WriteString(numbuf, Font_7x10, White);
    ssd1306_SetCursor(anim_coords[i][0], anim_coords[i][1]); 
    startScreen();
  }
  ssd1306_Fill(Black);
  //------------------------------------
  for(;;)
  {
    osDelay(100);
    ssd1306_Fill(Black);

    // 1. Вывод текущего режима управления (LEVEL)
    ssd1306_SetCursor(0, 0);
    ssd1306_WriteString("CTRL MODE:", Font_7x10, White);
    ssd1306_SetCursor(0, 12);
    if (current_ctrl == LEVEL_OFF) {
        ssd1306_WriteString("OFF", Font_11x18, White);
    } else if (current_ctrl == LEVEL_ON) {
        ssd1306_WriteString("MANUAL ON", Font_11x18, White);
    } else if (current_ctrl == LEVEL_AUTO) {
        ssd1306_WriteString("AUTOMATIC", Font_11x18, White);
    }

    // 2. Вывод состояния автомата (Датчики и Ошибки)
    ssd1306_SetCursor(0, 35);
    ssd1306_WriteString("STATE:", Font_7x10, White);
    ssd1306_SetCursor(0, 46);
    
    if (current_ctrl == LEVEL_AUTO) {
        switch (auto_state) {
            case AUTO_STATE_WAITING:
                ssd1306_WriteString("EMPTY / WAIT", Font_7x10, White);
                break;
            case AUTO_STATE_LL_TRIGGERED:
                ssd1306_WriteString("LEVEL LOW", Font_7x10, White);
                break;
            case AUTO_STATE_HH_TRIGGERED:
                ssd1306_WriteString("LEVEL HIGH (PUMP)", Font_7x10, White);
                break;
            case AUTO_STATE_ERROR:
                // Мигающая ошибка на экране
                if (blynk_output) {
                    ssd1306_WriteString("!!! ERR: SENSORS !!!", Font_7x10, White);
                }
                break;
        }
    } else if (current_ctrl == LEVEL_OFF && blynk_output) {
        // Если выключен и активна фаза блинка — пишем предупреждение о простое/ошибке из логики
        ssd1306_WriteString("SYSTEM DISABLED", Font_7x10, White);
    } else {
        ssd1306_WriteString("RUNNING", Font_7x10, White);
    }

    // 3. Вольтметр (Справа на экране)
    int volt_perc = Voltage.value;
    if (volt_perc < Voltage.value_min) volt_perc = Voltage.value_min;
    if (volt_perc > Voltage.value_max) volt_perc = Voltage.value_max;
    sprintf(buf, "V: %d.%d", volt_perc / 10, volt_perc % 10);
    ssd1306_SetCursor(85, 0);
    ssd1306_WriteString(buf, Font_7x10, White);
    
    // Иконка работы реле (имитация светодиода на экране)
    ssd1306_SetCursor(90, 25);
    if (HAL_GPIO_ReadPin(Left_out_GPIO_GPIO_Port, Left_out_GPIO_Pin) == GPIO_PIN_SET) {
        ssd1306_WriteString("[RELAY]", Font_7x10, White);
    }

    // Обновление экрана с авто-восстановлением шины I2C в случае сбоя
    if(ssd1306_UpdateScreen() == -1){
      HAL_I2C_DeInit(&hi2c1);
      MX_I2C1_Init();
      ssd1306_HardResetAndReinit();
    }
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
  for(;;)
  {
    osDelay(500); // Интервал изменения состояния (период 1 сек)
    blynk_output = !blynk_output;
    
    // Дублируем блинк на физический диод платы PC13, если требуется
    //HAL_GPIO_TogglePin(GPIOC, GPIO_PIN_13);
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
    sensor->value = (int16_t)scaled;
}

void ADC_ProcessNewSamples(void) {
    for (uint8_t ch = 0; ch < ADC_CHANNEL_COUNT; ch++) {
        adc_history[ch][adc_index] = ADC_dma[ch];
        uint32_t sum = 0;
        for (uint8_t i = 0; i < ADC_AVG_DEPTH; i++) {
            sum += adc_history[ch][i];
        }
        adc_filtered[ch] = sum / ADC_AVG_DEPTH;
    }
    adc_index = (adc_index + 1) % ADC_AVG_DEPTH;
}


/* USER CODE END Application */

