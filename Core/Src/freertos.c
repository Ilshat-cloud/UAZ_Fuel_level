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
#include "string.h"
#include "i2c.h"

#ifndef M_PI
#define M_PI 	3.1415926535f
#endif

#define ANGLE_MIN_DEG   10.0f
#define ANGLE_MAX_DEG   80.0f

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

static const uint16_t adc_table[11] =
{
    4096, 3718, 3340, 2962, 2584,
    2206, 1828, 1450, 1072,  694,0
};

static const uint16_t r20_table[11] =                   //r потенциометра умноженное на 20
{
       0,  667, 1333, 2000, 2667,
    3333, 4000, 4667, 5333, 6000,2000
};


uint16_t R_From_ADC(uint16_t adc);
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
extern SSD1306_t SSD1306;
extern I2C_HandleTypeDef hi2c1;
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

AnalogSensor_t Voltage;
AnalogSensor_t Level1_ai;
AnalogSensor_t Level2_ai;
static uint16_t ADC_dma[ADC_CHANNEL_COUNT];                     // Данные от DMA
static uint16_t adc_history[ADC_CHANNEL_COUNT][ADC_AVG_DEPTH]; // История выборок
static uint8_t  adc_index = 0;                                  // Индекс текущей выборки
static uint16_t adc_filtered[ADC_CHANNEL_COUNT];               // Усреднённые значения
uint8_t blynk_output=0;
static uint8_t cal_ongoing_flag=0;
uint8_t reverse_in_sequence=0, analog_reverse=0;
#pragma location = ".rodata"
__root static const uint8_t warning_triangle[240] =  { 0x0, 0x0, 0x0, 0x0, 0x0, 0x0, 0x0, 0x0, 0x0, 0x80, 0x0, 0x0, 0x0, 0x0, 0x1, 0xc0, 0x0, 0x0, 0x0, 0x0, 0x1, 0xc0, 0x0, 0x0, 0x0, 0x0, 0x3, 0x60, 0x0, 0x0, 0x0, 0x0, 0x6, 0x30, 0x0, 0x0, 0x0, 0x0, 0x4, 0x10, 0x0, 0x0, 0x0, 0x0, 0xc, 0x18, 0x0, 0x0, 0x0, 0x0, 0x18, 0x8c, 0x0, 0x0, 0x0, 0x0, 0x11, 0xc4, 0x0, 0x0, 0x0, 0x0, 0x31, 0xc6, 0x0, 0x0, 0x0, 0x0, 0x63, 0x63, 0x0, 0x0, 0x0, 0x0, 0x46, 0x31, 0x0, 0x0, 0x0, 0x0, 0xc4, 0x11, 0x80, 0x0, 0x0, 0x1, 0x8c, 0x18, 0xc0, 0x0, 0x0, 0x1, 0x18, 0xc, 0x40, 0x0, 0x0, 0x3, 0x10, 0x4, 0x60, 0x0, 0x0, 0x6, 0x30, 0x6, 0x30, 0x0, 0x0, 0x4, 0x60, 0x3, 0x10, 0x0, 0x0, 0xc, 0x40, 0x1, 0x18, 0x0, 0x0, 0x18, 0xc0, 0x1, 0x8c, 0x0, 0x0, 0x11, 0x80, 0x0, 0xc4, 0x0, 0x0, 0x31, 0x0, 0x0, 0x46, 0x0, 0x0, 0x63, 0x0, 0x0, 0x63, 0x0, 0x0, 0x46, 0x0, 0x0, 0x31, 0x0, 0x0, 0xc4, 0x0, 0x0, 0x11, 0x80, 0x1, 0x8c, 0x0, 0x0, 0x18, 0xc0, 0x1, 0x18, 0x0, 0x0, 0xc, 0x40, 0x3, 0x10, 0x0, 0x0, 0x4, 0x60, 0x6, 0x30, 0x0, 0x0, 0x6, 0x30, 0x4, 0x60, 0x0, 0x0, 0x3, 0x10, 0xc, 0x40, 0x0, 0x0, 0x1, 0x18, 0x18, 0xc0, 0x0, 0x0, 0x1, 0x8c, 0x11, 0xff, 0xff, 0xff, 0xff, 0xc4, 0x30, 0x0, 0x0, 0x0, 0x0, 0x6, 0x60, 0x0, 0x0, 0x0, 0x0, 0x3, 0x7f, 0xff, 0xff, 0xff, 0xff, 0xff, 0x0, 0x0, 0x0, 0x0, 0x0, 0x0, 0x0, 0x0, 0x0, 0x0, 0x0, 0x0, 0x0, 0x0, 0x0, 0x0, 0x0, 0x0 };
#pragma location = ".rodata"
__root static const uint8_t warning_triangle_fill[240] =  { 0x0, 0x0, 0x0, 0x0, 0x0, 0x0, 0x0, 0x0, 0x0, 0x80, 0x0, 0x0, 0x0, 0x0, 0x1, 0xc0, 0x0, 0x0, 0x0, 0x0, 0x1, 0xc0, 0x0, 0x0, 0x0, 0x0, 0x3, 0xe0, 0x0, 0x0, 0x0, 0x0, 0x7, 0xf0, 0x0, 0x0, 0x0, 0x0, 0x7, 0xf0, 0x0, 0x0, 0x0, 0x0, 0xf, 0xf8, 0x0, 0x0, 0x0, 0x0, 0x1f, 0xfc, 0x0, 0x0, 0x0, 0x0, 0x1f, 0xfc, 0x0, 0x0, 0x0, 0x0, 0x3f, 0xfe, 0x0, 0x0, 0x0, 0x0, 0x7f, 0x7f, 0x0, 0x0, 0x0, 0x0, 0x7e, 0x3f, 0x0, 0x0, 0x0, 0x0, 0xfc, 0x1f, 0x80, 0x0, 0x0, 0x1, 0xfc, 0x1f, 0xc0, 0x0, 0x0, 0x1, 0xf8, 0xf, 0xc0, 0x0, 0x0, 0x3, 0xf0, 0x7, 0xe0, 0x0, 0x0, 0x7, 0xf0, 0x7, 0xf0, 0x0, 0x0, 0x7, 0xe0, 0x3, 0xf0, 0x0, 0x0, 0xf, 0xc0, 0x1, 0xf8, 0x0, 0x0, 0x1f, 0xc0, 0x1, 0xfc, 0x0, 0x0, 0x1f, 0x80, 0x0, 0xfc, 0x0, 0x0, 0x3f, 0x0, 0x0, 0x7e, 0x0, 0x0, 0x7f, 0x0, 0x0, 0x7f, 0x0, 0x0, 0x7e, 0x0, 0x0, 0x3f, 0x0, 0x0, 0xfc, 0x0, 0x0, 0x1f, 0x80, 0x1, 0xfc, 0x0, 0x0, 0x1f, 0xc0, 0x1, 0xf8, 0x0, 0x0, 0xf, 0xc0, 0x3, 0xf0, 0x0, 0x0, 0x7, 0xe0, 0x7, 0xf0, 0x0, 0x0, 0x7, 0xf0, 0x7, 0xe0, 0x0, 0x0, 0x3, 0xf0, 0xf, 0xc0, 0x0, 0x0, 0x1, 0xf8, 0x1f, 0xc0, 0x0, 0x0, 0x1, 0xfc, 0x1f, 0xff, 0xff, 0xff, 0xff, 0xfc, 0x3f, 0xff, 0xff, 0xff, 0xff, 0xfe, 0x7f, 0xff, 0xff, 0xff, 0xff, 0xff, 0x7f, 0xff, 0xff, 0xff, 0xff, 0xff, 0x0, 0x0, 0x0, 0x0, 0x0, 0x0, 0x0, 0x0, 0x0, 0x0, 0x0, 0x0, 0x0, 0x0, 0x0, 0x0, 0x0, 0x0};

#pragma location = ".rodata"
__root static const uint8_t arrow_right[240] =  { 0x0, 0x0, 0x0, 0x0, 0x0, 0x0, 0x0, 0x0, 0x0, 0x80, 0x0, 0x0, 0x0, 0x0, 0x0, 0xc0, 0x0, 0x0, 0x0, 0x0, 0x0, 0xe0, 0x0, 0x0, 0x0, 0x0, 0x0, 0xb0, 0x0, 0x0, 0x0, 0x0, 0x0, 0x98, 0x0, 0x0, 0x0, 0x0, 0x0, 0x8e, 0x0, 0x0, 0x0, 0x0, 0x0, 0x83, 0x0, 0x0, 0x0, 0x0, 0x0, 0x81, 0x80, 0x0, 0x0, 0x0, 0x0, 0x80, 0xe0, 0x0, 0x0, 0x0, 0x0, 0x80, 0x30, 0x0, 0x0, 0x0, 0x0, 0x80, 0x18, 0x0, 0xff, 0xff, 0xff, 0x80, 0xe, 0x0, 0xff, 0xff, 0xff, 0x80, 0x3, 0x0, 0xc0, 0x0, 0x0, 0x0, 0x1, 0x80, 0xc0, 0x0, 0x0, 0x0, 0x0, 0xe0, 0xc0, 0x0, 0x0, 0x0, 0x0, 0x30, 0xc0, 0x0, 0x0, 0x0, 0x0, 0x1c, 0xc0, 0x0, 0x0, 0x0, 0x0, 0x6, 0xc0, 0x0, 0x0, 0x0, 0x0, 0x3, 0xc0, 0x0, 0x0, 0x0, 0x0, 0x3, 0xc0, 0x0, 0x0, 0x0, 0x0, 0x6, 0xc0, 0x0, 0x0, 0x0, 0x0, 0x1c, 0xc0, 0x0, 0x0, 0x0, 0x0, 0x30, 0xc0, 0x0, 0x0, 0x0, 0x0, 0xe0, 0xff, 0xff, 0xff, 0x80, 0x1, 0x80, 0xff, 0xff, 0xff, 0x80, 0x3, 0x0, 0x0, 0x0, 0x0, 0x80, 0xe, 0x0, 0x0, 0x0, 0x0, 0x80, 0x18, 0x0, 0x0, 0x0, 0x0, 0x80, 0x30, 0x0, 0x0, 0x0, 0x0, 0x80, 0xe0, 0x0, 0x0, 0x0, 0x0, 0x81, 0x80, 0x0, 0x0, 0x0, 0x0, 0x83, 0x0, 0x0, 0x0, 0x0, 0x0, 0x8e, 0x0, 0x0, 0x0, 0x0, 0x0, 0x98, 0x0, 0x0, 0x0, 0x0, 0x0, 0xb0, 0x0, 0x0, 0x0, 0x0, 0x0, 0xe0, 0x0, 0x0, 0x0, 0x0, 0x0, 0xc0, 0x0, 0x0, 0x0, 0x0, 0x0, 0x80, 0x0, 0x0, 0x0, 0x0, 0x0, 0x0, 0x0, 0x0 };
#pragma location = ".rodata"
__root static const uint8_t arrow_right_fill[240] = { 0x0, 0x0, 0x0, 0x0, 0x0, 0x0, 0x0, 0x0, 0x0, 0x80, 0x0, 0x0, 0x0, 0x0, 0x0, 0xc0, 0x0, 0x0, 0x0, 0x0, 0x0, 0xe0, 0x0, 0x0, 0x0, 0x0, 0x0, 0xf0, 0x0, 0x0, 0x0, 0x0, 0x0, 0xf8, 0x0, 0x0, 0x0, 0x0, 0x0, 0xfe, 0x0, 0x0, 0x0, 0x0, 0x0, 0xff, 0x0, 0x0, 0x0, 0x0, 0x0, 0xff, 0x80, 0x0, 0x0, 0x0, 0x0, 0xff, 0xe0, 0x0, 0x0, 0x0, 0x0, 0xff, 0xf0, 0x0, 0x0, 0x0, 0x0, 0xff, 0xf8, 0x0, 0xff, 0xff, 0xff, 0xff, 0xfe, 0x0, 0xff, 0xff, 0xff, 0xff, 0xff, 0x0, 0xff, 0xff, 0xff, 0xff, 0xff, 0x80, 0xff, 0xff, 0xff, 0xff, 0xff, 0xe0, 0xff, 0xff, 0xff, 0xff, 0xff, 0xf0, 0xff, 0xff, 0xff, 0xff, 0xff, 0xfc, 0xff, 0xff, 0xff, 0xff, 0xff, 0xfe, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xfe, 0xff, 0xff, 0xff, 0xff, 0xff, 0xfc, 0xff, 0xff, 0xff, 0xff, 0xff, 0xf0, 0xff, 0xff, 0xff, 0xff, 0xff, 0xe0, 0xff, 0xff, 0xff, 0xff, 0xff, 0x80, 0xff, 0xff, 0xff, 0xff, 0xff, 0x0, 0x0, 0x0, 0x0, 0xff, 0xfe, 0x0, 0x0, 0x0, 0x0, 0xff, 0xf8, 0x0, 0x0, 0x0, 0x0, 0xff, 0xf0, 0x0, 0x0, 0x0, 0x0, 0xff, 0xe0, 0x0, 0x0, 0x0, 0x0, 0xff, 0x80, 0x0, 0x0, 0x0, 0x0, 0xff, 0x0, 0x0, 0x0, 0x0, 0x0, 0xfe, 0x0, 0x0, 0x0, 0x0, 0x0, 0xf8, 0x0, 0x0, 0x0, 0x0, 0x0, 0xf0, 0x0, 0x0, 0x0, 0x0, 0x0, 0xe0, 0x0, 0x0, 0x0, 0x0, 0x0, 0xc0, 0x0, 0x0, 0x0, 0x0, 0x0, 0x80, 0x0, 0x0, 0x0, 0x0, 0x0, 0x0, 0x0, 0x0 };

static const uint8_t arrow_left[240] @ ".rodata"=   { 0x0, 0x0, 0x0, 0x0, 0x0, 0x0, 0x0, 0x0, 0x0, 0x0, 0x0, 0x0, 0x0, 0x0, 0x2, 0x0, 0x0, 0x0, 0x0, 0x0, 0x6, 0x0, 0x0, 0x0, 0x0, 0x0, 0xe, 0x0, 0x0, 0x0, 0x0, 0x0, 0x1a, 0x0, 0x0, 0x0, 0x0, 0x0, 0x32, 0x0, 0x0, 0x0, 0x0, 0x0, 0x62, 0x0, 0x0, 0x0, 0x0, 0x1, 0xc2, 0x0, 0x0, 0x0, 0x0, 0x3, 0x2, 0x0, 0x0, 0x0, 0x0, 0xe, 0x2, 0x0, 0x0, 0x0, 0x0, 0x18, 0x2, 0x0, 0x0, 0x0, 0x0, 0x30, 0x2, 0x0, 0x0, 0x0, 0x0, 0xe0, 0x3, 0xff, 0xff, 0xff, 0x1, 0x80, 0x3, 0xff, 0xff, 0xff, 0x3, 0x0, 0x0, 0x0, 0x0, 0x3, 0xe, 0x0, 0x0, 0x0, 0x0, 0x3, 0x18, 0x0, 0x0, 0x0, 0x0, 0x3, 0x70, 0x0, 0x0, 0x0, 0x0, 0x3, 0xc0, 0x0, 0x0, 0x0, 0x0, 0x3, 0xc0, 0x0, 0x0, 0x0, 0x0, 0x3, 0x70, 0x0, 0x0, 0x0, 0x0, 0x3, 0x18, 0x0, 0x0, 0x0, 0x0, 0x3, 0xe, 0x0, 0x0, 0x0, 0x0, 0x3, 0x3, 0x0, 0x0, 0x0, 0x0, 0x3, 0x1, 0x80, 0x3, 0xff, 0xff, 0xff, 0x0, 0xe0, 0x3, 0xff, 0xff, 0xff, 0x0, 0x30, 0x2, 0x0, 0x0, 0x0, 0x0, 0x18, 0x2, 0x0, 0x0, 0x0, 0x0, 0xe, 0x2, 0x0, 0x0, 0x0, 0x0, 0x3, 0x2, 0x0, 0x0, 0x0, 0x0, 0x1, 0xc2, 0x0, 0x0, 0x0, 0x0, 0x0, 0x62, 0x0, 0x0, 0x0, 0x0, 0x0, 0x32, 0x0, 0x0, 0x0, 0x0, 0x0, 0x1a, 0x0, 0x0, 0x0, 0x0, 0x0, 0xe, 0x0, 0x0, 0x0, 0x0, 0x0, 0x6, 0x0, 0x0, 0x0, 0x0, 0x0, 0x2, 0x0, 0x0, 0x0, 0x0, 0x0, 0x0, 0x0, 0x0, 0x0, 0x0, 0x0, 0x0, 0x0, 0x0, 0x0 };
static const uint8_t arrow_left_fill[240] @ ".rodata" = { 0x0, 0x0, 0x0, 0x0, 0x0, 0x0, 0x0, 0x0, 0x0, 0x0, 0x0, 0x0, 0x0, 0x0, 0x2, 0x0, 0x0, 0x0, 0x0, 0x0, 0x6, 0x0, 0x0, 0x0, 0x0, 0x0, 0xe, 0x0, 0x0, 0x0, 0x0, 0x0, 0x1e, 0x0, 0x0, 0x0, 0x0, 0x0, 0x3e, 0x0, 0x0, 0x0, 0x0, 0x0, 0x7e, 0x0, 0x0, 0x0, 0x0, 0x1, 0xfe, 0x0, 0x0, 0x0, 0x0, 0x3, 0xfe, 0x0, 0x0, 0x0, 0x0, 0xf, 0xfe, 0x0, 0x0, 0x0, 0x0, 0x1f, 0xfe, 0x0, 0x0, 0x0, 0x0, 0x3f, 0xfe, 0x0, 0x0, 0x0, 0x0, 0xff, 0xff, 0xff, 0xff, 0xff, 0x1, 0xff, 0xff, 0xff, 0xff, 0xff, 0x3, 0xff, 0xff, 0xff, 0xff, 0xff, 0xf, 0xff, 0xff, 0xff, 0xff, 0xff, 0x1f, 0xff, 0xff, 0xff, 0xff, 0xff, 0x7f, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0x7f, 0xff, 0xff, 0xff, 0xff, 0xff, 0x1f, 0xff, 0xff, 0xff, 0xff, 0xff, 0xf, 0xff, 0xff, 0xff, 0xff, 0xff, 0x3, 0xff, 0xff, 0xff, 0xff, 0xff, 0x1, 0xff, 0xff, 0xff, 0xff, 0xff, 0x0, 0xff, 0xff, 0xff, 0xff, 0xff, 0x0, 0x3f, 0xfe, 0x0, 0x0, 0x0, 0x0, 0x1f, 0xfe, 0x0, 0x0, 0x0, 0x0, 0xf, 0xfe, 0x0, 0x0, 0x0, 0x0, 0x3, 0xfe, 0x0, 0x0, 0x0, 0x0, 0x1, 0xfe, 0x0, 0x0, 0x0, 0x0, 0x0, 0x7e, 0x0, 0x0, 0x0, 0x0, 0x0, 0x3e, 0x0, 0x0, 0x0, 0x0, 0x0, 0x1e, 0x0, 0x0, 0x0, 0x0, 0x0, 0xe, 0x0, 0x0, 0x0, 0x0, 0x0, 0x6, 0x0, 0x0, 0x0, 0x0, 0x0, 0x2, 0x0, 0x0, 0x0, 0x0, 0x0, 0x0, 0x0, 0x0, 0x0, 0x0, 0x0, 0x0, 0x0, 0x0, 0x0 };
static const uint8_t gas[]@ ".rodata" = { 0x1f, 0xff, 0xf8, 0x0, 0x3f, 0xff, 0xfc, 0x0, 0x60, 0x0, 0x6, 0x0, 0xcf, 0xff, 0xf3, 0x0, 0xd3, 0x18, 0xcb, 0x0, 0xd4, 0xa5, 0x2b, 0x80, 0xd4, 0xa5, 0x2b, 0xc6, 0xd4, 0xa5, 0x2b, 0x6f, 0xd3, 0x18, 0xcb, 0x3d, 0xcf, 0xff, 0xf3, 0x19, 0xc0, 0x0, 0x3, 0xd, 0xc0, 0x0, 0x3, 0xf, 0xc3, 0xff, 0xc3, 0x87, 0xc7, 0xff, 0xe3, 0x41, 0xcf, 0x0, 0xf3, 0x21, 0xce, 0x0, 0x73, 0x11, 0xcc, 0x0, 0x33, 0x11, 0xcc, 0x0, 0x33, 0x11, 0xcc, 0x0, 0x33, 0x11, 0xcc, 0x0, 0x33, 0x11, 0xcc, 0x0, 0x33, 0x11, 0xcc, 0x0, 0x33, 0x11, 0xcc, 0x0, 0x33, 0x11, 0xcc, 0x0, 0x33, 0x11, 0xcc, 0x0, 0x33, 0x11, 0xce, 0x0, 0x73, 0x21, 0xcf, 0x0, 0xf3, 0x61, 0xc7, 0xff, 0xe3, 0x41, 0x63, 0xff, 0xc6, 0x41, 0x30, 0x0, 0xc, 0x41, 0x1f, 0xff, 0xf8, 0x72, 0xf, 0xff, 0xf0, 0x1c };

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
  HAL_ADCEx_Calibration_Start(&hadc1);
  Flash_read();
  uint8_t sequence_trig=0;
  uint16_t hold_cnt_prev;
  /* Infinite loop */
  for(;;)
  {
    HAL_ADC_Stop_DMA(&hadc1);
    HAL_ADC_Start_DMA(&hadc1,(uint32_t*)&ADC_dma,3);
    osDelay(10);
    hold_cnt_prev=CAL.hold_counter;
    ADC_ProcessNewSamples();
    AnalogSensor_Update(&Level1_ai,R_From_ADC(adc_filtered[0]));                //TODO Вот тут надо продебажить и снять кривую для АЦП
    AnalogSensor_Update(&Level2_ai,R_From_ADC(adc_filtered[1]));
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
      Level1_ai.adc_max=Level1_ai.adc_max+10;
      if(Level1_ai.adc_min>10){
        Level1_ai.adc_min=Level1_ai.adc_min-10;
      }else{
        Level1_ai.adc_min=0;
      }
      Level2_ai.adc_max=Level2_ai.adc_max+10;
      if(Level2_ai.adc_min>10){
        Level2_ai.adc_min=Level2_ai.adc_min-10;
      }else{
        Level2_ai.adc_min=0;
      }        
      Flash_write();
    }
    if(CAL.pos_out==GPIO_PIN_RESET){
      sequence_trig=1;
    }   
    
    if((CAL.hold_counter>200)&&(sequence_trig)){
      sequence_trig=0;
      reverse_in_sequence++;
    }
       
    if(CAL.hold_counter>400){
      reverse_in_sequence=0;
      sequence_trig=0;
      analog_reverse=0;
    }
    if((reverse_in_sequence==2)&&(CAL.pos_out==GPIO_PIN_RESET)&&(hold_cnt_prev>10)&&(hold_cnt_prev<100)){
      analog_reverse^=0x01;
    }
    if((reverse_in_sequence==3)&&(CAL.pos_out==GPIO_PIN_RESET)&&(hold_cnt_prev>10)&&(hold_cnt_prev<100)){
      analog_reverse^=0x02;
    }
    if((reverse_in_sequence==4)&&(CAL.pos_out==GPIO_PIN_RESET)){
      int16_t temp;
      if(analog_reverse&0x01){
        temp=Level1_ai.value_min;
        Level1_ai.value_min=Level1_ai.value_max;
        Level1_ai.value_max=temp;
      }
      if(analog_reverse&0x02){
        temp=Level2_ai.value_min;
        Level2_ai.value_min=Level2_ai.value_max;
        Level2_ai.value_max=temp;
      }
      reverse_in_sequence=0;
      analog_reverse=0;
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
  uint8_t buffer[240];
  //----------Horse---------------

  
  ssd1306_HardResetAndReinit();
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
    char buf[20];
    ssd1306_Fill(Black);

    // Линия 1
    ssd1306_SetCursor(0, 9);
    if (cal_ongoing_flag)
    {
      ssd1306_WriteString("L1:", Font_7x10, White);
      ssd1306_SetCursor(21, 7);
      ssd1306_WriteString("CAL", Font_11x18, White);
    }
    else
    {
      int perc = Level1_ai.value;  // проценты уже есть
      ssd1306_WriteString("L1:", Font_7x10, White);
      if (perc < 0){
        ssd1306_SetCursor(21, 7);
        ssd1306_WriteString("err", Font_11x18, White);
        if(Fuel_level1_low.pos_out==GPIO_PIN_SET){
          memcpy(buffer,gas,128);
          SSD1306.CurrentY-=7;
          DrawBitmap_32(buffer,White);
        }
      }else if (perc > 109){
        ssd1306_SetCursor(21, 7);
        ssd1306_WriteString("err", Font_11x18, White);
        if(Fuel_level1_low.pos_out==GPIO_PIN_SET){
          memcpy(buffer,gas,128);
          SSD1306.CurrentY-=7;
          DrawBitmap_32(buffer,White);
        }
      }else{
        ssd1306_SetCursor(21, 0);
        char numbuf[5];
        sprintf(numbuf, "%d", perc);
        if((Fuel_level1_low.pos_out==GPIO_PIN_RESET)||(reverse_in_sequence==2)){
          if(reverse_in_sequence==2){
            ssd1306_WriteString(numbuf, Font_16x26, Black);
            SSD1306.CurrentY+=4;
            ssd1306_WriteString("%", Font_11x18, Black);
          }else{
            ssd1306_WriteString(numbuf, Font_16x26, White);
            SSD1306.CurrentY+=4;
            ssd1306_WriteString("%", Font_11x18, White);
          }
        }else{
          ssd1306_WriteString(numbuf, Font_16x26, Black);
          memcpy(buffer,gas,128);
          SSD1306.CurrentY=0;
          DrawBitmap_32(buffer,White);
          //ssd1306_WriteString("%", Font_11x18, Black);
        }
      }
      if((reverse_in_sequence==2)||(reverse_in_sequence==3)){
        if (analog_reverse & 0x01){
            ssd1306_WriteString("REV", Font_7x10, White);
        } else {
            ssd1306_WriteString("NORM", Font_7x10, White);
        }
      }
    }

    
     
    
    // Линия 2
    ssd1306_SetCursor(0, 51);
    if (cal_ongoing_flag)
    {
      ssd1306_WriteString("L2:", Font_7x10, White);
       ssd1306_SetCursor(21, 45);
      ssd1306_WriteString("CAL", Font_11x18, White);
    }
    else
    {
      int perc = Level2_ai.value;
      ssd1306_WriteString("L2:", Font_7x10, White);
      if (perc < 0){
        ssd1306_SetCursor(21, 45);
        ssd1306_WriteString("err", Font_11x18, White);
        if(Fuel_level2_low.pos_out==GPIO_PIN_SET){
          memcpy(buffer,gas,128);
          SSD1306.CurrentY-=14;
          DrawBitmap_32(buffer,White);
          
        }
      } else if (perc > 109){
        ssd1306_SetCursor(21, 45);
        ssd1306_WriteString("err", Font_11x18, White);
        if(Fuel_level2_low.pos_out==GPIO_PIN_SET){
          memcpy(buffer,gas,128);
          SSD1306.CurrentY-=14;
          DrawBitmap_32(buffer,White);
          
        }
      } else {
        char numbuf[5];
        ssd1306_SetCursor(21, 37);
        sprintf(numbuf, "%d", perc);
        if((Fuel_level2_low.pos_out==GPIO_PIN_RESET)||(reverse_in_sequence==3)){
          if(reverse_in_sequence==3){
            ssd1306_WriteString(numbuf, Font_16x26, Black);
            SSD1306.CurrentY+=4;
            ssd1306_WriteString("%", Font_11x18, Black);
          }else{
            ssd1306_WriteString(numbuf, Font_16x26, White);
            SSD1306.CurrentY+=4;
            ssd1306_WriteString("%", Font_11x18, White);
          }
        }else{
          ssd1306_WriteString(numbuf, Font_16x26, Black);
          memcpy(buffer,gas,128);
          SSD1306.CurrentY-=6;
          DrawBitmap_32(buffer,White);
          //ssd1306_WriteString("%", Font_11x18, Black);
        }
      }
      if((reverse_in_sequence==2)||(reverse_in_sequence==3)){
        if (analog_reverse & 0x02){
          ssd1306_WriteString("REV", Font_7x10, White);
        } else {
          ssd1306_WriteString("NORM", Font_7x10, White);
        }
      }
    }

    
    // Справа — напряжение или иконка поворотника
    if (current_blynk_flag == Blynk_off)
    {
      int volt_perc = Voltage.value;
      if (volt_perc < Voltage.value_min) volt_perc = Voltage.value_min;
      if (volt_perc > Voltage.value_max) volt_perc = Voltage.value_max;
      uint8_t xxx=volt_perc/10;
      uint8_t yyy=volt_perc%10;
      sprintf(buf, "%d", xxx);
      ssd1306_SetCursor(87, 20);
      ssd1306_WriteString(buf, Font_11x18, White);
      SSD1306.CurrentX-=2;
      ssd1306_WriteString(".", Font_11x18, White);
      SSD1306.CurrentX-=2;
      sprintf(buf, "%d", yyy);
      ssd1306_WriteString(buf, Font_11x18, White);
    }
    else
    {
      switch (current_blynk_flag)
      {
      case Blynk_right:
        if(blynk_output){
          memcpy(buffer,arrow_right,240);
        }else{
          memcpy(buffer,arrow_right_fill,240);
        }
        break;
      case Blynk_left:
        if(blynk_output){
          memcpy(buffer,arrow_left,240);
        }else{
          memcpy(buffer,arrow_left_fill,240);
        }
        break;
      case Blynk_warning:
        if(blynk_output){
          memcpy(buffer,warning_triangle,240);
        }else{
          memcpy(buffer,warning_triangle_fill,240);
        }
        break;
      default:
        break;
      }
    ssd1306_SetCursor(80, 8);
    DrawBitmap(buffer,White);   
    }
    if(ssd1306_UpdateScreen()==-1){
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
  
  /* Infinite loop */
  for(;;)
  {
    osDelay(500); 
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

//    // Ограничим в пределах value_min и value_max
//    if (scaled < sensor->value_min) scaled = sensor->value_min;
//    if (scaled > sensor->value_max) scaled = sensor->value_max;

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
  if(sensor->adc_min<5000){
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
        Flash_write();
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
    Voltage.adc_min   = 1;
    Voltage.adc_max   = 4096;
    Voltage.value_min = 0;
    Voltage.value_max = 331;
    Voltage.adc_raw   = 0;
    Voltage.value     = 0;

    Level1_ai.adc_min   = 20;
    Level1_ai.adc_max   = 6000;
    Level1_ai.value_min =0;
    Level1_ai.value_max = 100;
    Level1_ai.adc_raw   = 0;
    Level1_ai.value     = 0;

    Level2_ai.adc_min   = 20;
    Level2_ai.adc_max   = 6000;
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



uint16_t R_From_ADC(uint16_t adc)
{
    /* Границы */
    if (adc >= adc_table[0])
    {
        return r20_table[0];
    }

    if (adc <= adc_table[10])
    {
        return r20_table[10];
    }

    /* Поиск сегмента */
    for (uint8_t i = 0; i < 10; i++)
    {
        if ((adc <= adc_table[i]) && (adc >= adc_table[i + 1]))
        {
            uint16_t x0 = adc_table[i];
            uint16_t x1 = adc_table[i + 1];

            uint16_t y0 = r20_table[i];
            uint16_t y1 = r20_table[i + 1];

            /* Интерполяция:
               y = y0 + (x0 - adc) * (y1 - y0) / (x0 - x1)
            */
            uint32_t num = (uint32_t)(x0 - adc) * (y1 - y0);
            uint32_t den = (uint32_t)(x0 - x1);

            return (uint16_t)(y0 + num / den);
        }
    }

    return 0; // не должно сюда попасть
}

/* USER CODE END Application */

