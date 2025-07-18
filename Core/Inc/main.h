/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file           : main.h
  * @brief          : Header for main.c file.
  *                   This file contains the common defines of the application.
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

/* Define to prevent recursive inclusion -------------------------------------*/
#ifndef __MAIN_H
#define __MAIN_H

#ifdef __cplusplus
extern "C" {
#endif

/* Includes ------------------------------------------------------------------*/
#include "stm32f1xx_hal.h"

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */

/* USER CODE END Includes */

/* Exported types ------------------------------------------------------------*/
/* USER CODE BEGIN ET */

/* USER CODE END ET */

/* Exported constants --------------------------------------------------------*/
/* USER CODE BEGIN EC */

/* USER CODE END EC */

/* Exported macro ------------------------------------------------------------*/
/* USER CODE BEGIN EM */

/* USER CODE END EM */

/* Exported functions prototypes ---------------------------------------------*/
void Error_Handler(void);

/* USER CODE BEGIN EFP */

/* USER CODE END EFP */

/* Private defines -----------------------------------------------------------*/
#define Fuel_level1_ADC_Pin GPIO_PIN_0
#define Fuel_level1_ADC_GPIO_Port GPIOA
#define Fuel_level2_ADC_Pin GPIO_PIN_1
#define Fuel_level2_ADC_GPIO_Port GPIOA
#define V_to_ADC_Pin GPIO_PIN_2
#define V_to_ADC_GPIO_Port GPIOA
#define Warning_in_Pin GPIO_PIN_3
#define Warning_in_GPIO_Port GPIOA
#define Right_in_Pin GPIO_PIN_4
#define Right_in_GPIO_Port GPIOA
#define Left_in_Pin GPIO_PIN_5
#define Left_in_GPIO_Port GPIOA
#define CAL_Pin GPIO_PIN_6
#define CAL_GPIO_Port GPIOA
#define Fuel_level1_low_Pin GPIO_PIN_7
#define Fuel_level1_low_GPIO_Port GPIOA
#define Fuel_level2_low_Pin GPIO_PIN_0
#define Fuel_level2_low_GPIO_Port GPIOB
#define Left_out_GPIO_Pin GPIO_PIN_1
#define Left_out_GPIO_GPIO_Port GPIOB
#define Right_out_gpio_Pin GPIO_PIN_2
#define Right_out_gpio_GPIO_Port GPIOB
#define Reboot_LCD_Pin GPIO_PIN_8
#define Reboot_LCD_GPIO_Port GPIOA

/* USER CODE BEGIN Private defines */
static const int   User_Page_Adress[]={
  0x0800FC00,0x0800FC04,0x0800FC08,0x0800FC0C,0x0800FC10,0x0800FC14,0x0800FC18,0x0800FC1C,
  0x0800FC20,0x0800FC24,0x0800FC28,0x0800FC2C,0x0800FC30,0x0800FC34,0x0800FC38,0x0800FC3C,
  0x0800FC40,0x0800FC44,0x0800FC48,0x0800FC4C,0x0800FC50,0x0800FC54,0x0800FC58,0x0800FC5C,
  0x0800FC60,0x0800FC64};

/* USER CODE END Private defines */

#ifdef __cplusplus
}
#endif

#endif /* __MAIN_H */
