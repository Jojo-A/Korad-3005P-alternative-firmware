/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file           : main.h
  * @brief          : Header for main.c file.
  *                   This file contains the common defines of the application.
  ******************************************************************************
  * @attention
  *
  * Copyright (c) 2022 STMicroelectronics.
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
#define DO_BUZZER_Pin GPIO_PIN_13
#define DO_BUZZER_GPIO_Port GPIOC
#define DO_REL1_Pin GPIO_PIN_14
#define DO_REL1_GPIO_Port GPIOC
#define DO_REL2_Pin GPIO_PIN_15
#define DO_REL2_GPIO_Port GPIOC
#define DO_REL_OUT_Pin GPIO_PIN_0
#define DO_REL_OUT_GPIO_Port GPIOD
#define DI_OCP_Pin GPIO_PIN_1
#define DI_OCP_GPIO_Port GPIOD
#define AI_VOUT_Pin GPIO_PIN_0
#define AI_VOUT_GPIO_Port GPIOA
#define AI_IOUT_Pin GPIO_PIN_1
#define AI_IOUT_GPIO_Port GPIOA
#define AI_NTC_Pin GPIO_PIN_2
#define AI_NTC_GPIO_Port GPIOA
#define DO_KEY1_Pin GPIO_PIN_3
#define DO_KEY1_GPIO_Port GPIOA
#define DO_KEY2_Pin GPIO_PIN_4
#define DO_KEY2_GPIO_Port GPIOA
#define SPI_DISP_SCK_Pin GPIO_PIN_5
#define SPI_DISP_SCK_GPIO_Port GPIOA
#define DO_DISP_LATCH_Pin GPIO_PIN_6
#define DO_DISP_LATCH_GPIO_Port GPIOA
#define SPI_DISP_MOSI_Pin GPIO_PIN_7
#define SPI_DISP_MOSI_GPIO_Port GPIOA
#define DO_KEY3_Pin GPIO_PIN_0
#define DO_KEY3_GPIO_Port GPIOB
#define DO_KEY4_Pin GPIO_PIN_1
#define DO_KEY4_GPIO_Port GPIOB
#define DO_DISP_COL0_Pin GPIO_PIN_2
#define DO_DISP_COL0_GPIO_Port GPIOB
#define DO_DISP_COL1_Pin GPIO_PIN_10
#define DO_DISP_COL1_GPIO_Port GPIOB
#define DO_DISP_COL2_Pin GPIO_PIN_11
#define DO_DISP_COL2_GPIO_Port GPIOB
#define DO_DISP_COL3_Pin GPIO_PIN_12
#define DO_DISP_COL3_GPIO_Port GPIOB
#define SPI_DAC_SCK_Pin GPIO_PIN_13
#define SPI_DAC_SCK_GPIO_Port GPIOB
#define DO_DAC_LATCH_Pin GPIO_PIN_14
#define DO_DAC_LATCH_GPIO_Port GPIOB
#define SPI_DAC_MOSI_Pin GPIO_PIN_15
#define SPI_DAC_MOSI_GPIO_Port GPIOB
#define TIM1_ENCA_Pin GPIO_PIN_8
#define TIM1_ENCA_GPIO_Port GPIOA
#define TIM1_ENCB_Pin GPIO_PIN_9
#define TIM1_ENCB_GPIO_Port GPIOA
#define DI_KEY1_Pin GPIO_PIN_10
#define DI_KEY1_GPIO_Port GPIOA
#define DI_KEY2_Pin GPIO_PIN_11
#define DI_KEY2_GPIO_Port GPIOA
#define DI_KEY3_Pin GPIO_PIN_12
#define DI_KEY3_GPIO_Port GPIOA
#define DO_LEDS_Pin GPIO_PIN_15
#define DO_LEDS_GPIO_Port GPIOA
#define PWM_FAN_Pin GPIO_PIN_4
#define PWM_FAN_GPIO_Port GPIOB
/* USER CODE BEGIN Private defines */

/* USER CODE END Private defines */

#ifdef __cplusplus
}
#endif

#endif /* __MAIN_H */
