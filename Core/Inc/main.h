/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file           : main.h
  * @brief          : Header for main.c file.
  *                   This file contains the common defines of the application.
  ******************************************************************************
  * @attention
  *
  * <h2><center>&copy; Copyright (c) 2024 STMicroelectronics.
  * All rights reserved.</center></h2>
  *
  * This software component is licensed by ST under BSD 3-Clause license,
  * the "License"; You may not use this file except in compliance with the
  * License. You may obtain a copy of the License at:
  *                        opensource.org/licenses/BSD-3-Clause
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
typedef struct dataMain {
	uint8_t timeToEntance;
	uint8_t timeToExit;
	uint8_t gateMode; //0 - выкл, 1 - авто, 2- только въезд, 3-только выезд
	uint8_t startDelay;
	int8_t coolerTemp;
	int8_t heaterTemp;
};
/* USER CODE END ET */

/* Exported constants --------------------------------------------------------*/
/* USER CODE BEGIN EC */

/* USER CODE END EC */

/* Exported macro ------------------------------------------------------------*/
/* USER CODE BEGIN EM */

/* USER CODE END EM */

void HAL_TIM_MspPostInit(TIM_HandleTypeDef *htim);

/* Exported functions prototypes ---------------------------------------------*/
void Error_Handler(void);

/* USER CODE BEGIN EFP */

/* USER CODE END EFP */

/* Private defines -----------------------------------------------------------*/
#define AIN2_Pin GPIO_PIN_0
#define AIN2_GPIO_Port GPIOB
#define AIN1_Pin GPIO_PIN_1
#define AIN1_GPIO_Port GPIOB
#define BUZZER_Pin GPIO_PIN_8
#define BUZZER_GPIO_Port GPIOE
#define QOUT1_Pin GPIO_PIN_9
#define QOUT1_GPIO_Port GPIOE
#define DCOK_Pin GPIO_PIN_11
#define DCOK_GPIO_Port GPIOE
#define OUT1_Pin GPIO_PIN_12
#define OUT1_GPIO_Port GPIOE
#define OUT2_Pin GPIO_PIN_13
#define OUT2_GPIO_Port GPIOE
#define OUT3_Pin GPIO_PIN_14
#define OUT3_GPIO_Port GPIOE
#define OUT4_Pin GPIO_PIN_15
#define OUT4_GPIO_Port GPIOE
#define OUT5_Pin GPIO_PIN_10
#define OUT5_GPIO_Port GPIOB
#define OUT6_Pin GPIO_PIN_11
#define OUT6_GPIO_Port GPIOB
#define IN8_Pin GPIO_PIN_12
#define IN8_GPIO_Port GPIOB
#define IN7_Pin GPIO_PIN_13
#define IN7_GPIO_Port GPIOB
#define D1_Pin GPIO_PIN_14
#define D1_GPIO_Port GPIOB
#define D1_EXTI_IRQn EXTI15_10_IRQn
#define D0_Pin GPIO_PIN_15
#define D0_GPIO_Port GPIOB
#define D0_EXTI_IRQn EXTI15_10_IRQn
#define RE2_Pin GPIO_PIN_10
#define RE2_GPIO_Port GPIOD
#define LED5_Pin GPIO_PIN_12
#define LED5_GPIO_Port GPIOD
#define IN4_Pin GPIO_PIN_6
#define IN4_GPIO_Port GPIOC
#define IN3_Pin GPIO_PIN_7
#define IN3_GPIO_Port GPIOC
#define IN2_Pin GPIO_PIN_8
#define IN2_GPIO_Port GPIOC
#define IN1_Pin GPIO_PIN_9
#define IN1_GPIO_Port GPIOC
#define RE1_Pin GPIO_PIN_8
#define RE1_GPIO_Port GPIOA
#define LED4_Pin GPIO_PIN_3
#define LED4_GPIO_Port GPIOD
#define LED3_Pin GPIO_PIN_4
#define LED3_GPIO_Port GPIOD
#define LED2_Pin GPIO_PIN_5
#define LED2_GPIO_Port GPIOD
#define LED1_Pin GPIO_PIN_6
#define LED1_GPIO_Port GPIOD
#define PWRLED_Pin GPIO_PIN_7
#define PWRLED_GPIO_Port GPIOD
#define VUSB_Pin GPIO_PIN_4
#define VUSB_GPIO_Port GPIOB
#define WP_Pin GPIO_PIN_5
#define WP_GPIO_Port GPIOB

/* USER CODE BEGIN Private defines */

/* USER CODE END Private defines */

#ifdef __cplusplus
}
#endif

#endif /* __MAIN_H */
