/* USER CODE BEGIN Header */
/**
 ******************************************************************************
 * @file           : main.h
 * @brief          : Header for main.c file.
 *                   This file contains the common defines of the application.
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

/* Define to prevent recursive inclusion -------------------------------------*/
#ifndef __MAIN_H
#define __MAIN_H

#ifdef __cplusplus
extern "C" {
#endif

/* Includes ------------------------------------------------------------------*/
#include "stm32h7xx_hal.h"

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
#define STP3_DIR_Pin GPIO_PIN_0
#define STP3_DIR_GPIO_Port GPIOI
#define STP4_EN_Pin GPIO_PIN_2
#define STP4_EN_GPIO_Port GPIOI
#define MP3_UART5_TX_Pin GPIO_PIN_12
#define MP3_UART5_TX_GPIO_Port GPIOC
#define STP4_DIR_Pin GPIO_PIN_3
#define STP4_DIR_GPIO_Port GPIOI
#define MP3_UART5_RX_Pin GPIO_PIN_2
#define MP3_UART5_RX_GPIO_Port GPIOD
#define STP2_EN_Pin GPIO_PIN_11
#define STP2_EN_GPIO_Port GPIOH
#define STP1_DIR_Pin GPIO_PIN_9
#define STP1_DIR_GPIO_Port GPIOH
#define STP2_DIR_Pin GPIO_PIN_12
#define STP2_DIR_GPIO_Port GPIOH
#define STP1_EN_Pin GPIO_PIN_8
#define STP1_EN_GPIO_Port GPIOH
#define STP3_EN_Pin GPIO_PIN_7
#define STP3_EN_GPIO_Port GPIOH

/* USER CODE BEGIN Private defines */

/* USER CODE END Private defines */

#ifdef __cplusplus
}
#endif

#endif /* __MAIN_H */
