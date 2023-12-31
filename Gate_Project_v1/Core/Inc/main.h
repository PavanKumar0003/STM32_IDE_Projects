/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file           : main.h
  * @brief          : Header for main.c file.
  *                   This file contains the common defines of the application.
  ******************************************************************************
  * @attention
  *
  * Copyright (c) 2023 STMicroelectronics.
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
#include "stm32f4xx_hal.h"

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
#define LCD_D0_Pin GPIO_PIN_0
#define LCD_D0_GPIO_Port GPIOB
#define LCD_D1_Pin GPIO_PIN_1
#define LCD_D1_GPIO_Port GPIOB
#define LCD_D2_Pin GPIO_PIN_2
#define LCD_D2_GPIO_Port GPIOB
#define IR_Sensor_Pin GPIO_PIN_10
#define IR_Sensor_GPIO_Port GPIOB
#define Buzzer_Pin GPIO_PIN_12
#define Buzzer_GPIO_Port GPIOB
#define PIR_Sensor_Pin GPIO_PIN_15
#define PIR_Sensor_GPIO_Port GPIOB
#define UP_BTN_Pin GPIO_PIN_8
#define UP_BTN_GPIO_Port GPIOC
#define DWN_BTN_Pin GPIO_PIN_9
#define DWN_BTN_GPIO_Port GPIOC
#define LCD_D3_Pin GPIO_PIN_3
#define LCD_D3_GPIO_Port GPIOB
#define LCD_RS_Pin GPIO_PIN_4
#define LCD_RS_GPIO_Port GPIOB
#define LCD_RW_Pin GPIO_PIN_5
#define LCD_RW_GPIO_Port GPIOB
#define LCD_EW_Pin GPIO_PIN_8
#define LCD_EW_GPIO_Port GPIOB
/* USER CODE BEGIN Private defines */

/* USER CODE END Private defines */

#ifdef __cplusplus
}
#endif

#endif /* __MAIN_H */
