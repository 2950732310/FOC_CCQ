/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file           : main.h
  * @brief          : Header for main.c file.
  *                   This file contains the common defines of the application.
  ******************************************************************************
  * @attention
  *
  * Copyright (c) 2026 STMicroelectronics.
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
#include "stm32g4xx_hal.h"

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
#define MCPWM_CLOCK_HZ 168000000
#define MCPWM_DEADTIME_CLOCKS 20
#define MCPWM_TGRO_TIME MCPWM_PERIOD_CLOCKS-10
#define MCPWM_RCR 0
#define MCPWM_FREQ 20000
#define MCPWM_PERIOD_CLOCKS MCPWM_CLOCK_HZ/2/MCPWM_FREQ
#define SAMPL_CURRENT_IC_Pin GPIO_PIN_0
#define SAMPL_CURRENT_IC_GPIO_Port GPIOA
#define SAMPL_CURRENT_IB_Pin GPIO_PIN_1
#define SAMPL_CURRENT_IB_GPIO_Port GPIOA
#define SAMPL_CURRENT_IA_Pin GPIO_PIN_2
#define SAMPL_CURRENT_IA_GPIO_Port GPIOA
#define VOL_SAMPL_Pin GPIO_PIN_3
#define VOL_SAMPL_GPIO_Port GPIOA
#define WS2812_Pin GPIO_PIN_4
#define WS2812_GPIO_Port GPIOA
#define SPI1_SCK_Pin GPIO_PIN_5
#define SPI1_SCK_GPIO_Port GPIOA
#define SPI1_MISO_Pin GPIO_PIN_6
#define SPI1_MISO_GPIO_Port GPIOA
#define SPI1_MOSI_Pin GPIO_PIN_7
#define SPI1_MOSI_GPIO_Port GPIOA
#define SPI1_CS_Pin GPIO_PIN_4
#define SPI1_CS_GPIO_Port GPIOC
#define LIN3_Pin GPIO_PIN_13
#define LIN3_GPIO_Port GPIOB
#define LIN2_Pin GPIO_PIN_14
#define LIN2_GPIO_Port GPIOB
#define LIN1_Pin GPIO_PIN_15
#define LIN1_GPIO_Port GPIOB
#define HIN3_Pin GPIO_PIN_8
#define HIN3_GPIO_Port GPIOA
#define HIN2_Pin GPIO_PIN_9
#define HIN2_GPIO_Port GPIOA
#define HIN1_Pin GPIO_PIN_10
#define HIN1_GPIO_Port GPIOA
#define UART1_TX_Pin GPIO_PIN_6
#define UART1_TX_GPIO_Port GPIOB
#define UART1_RX_Pin GPIO_PIN_7
#define UART1_RX_GPIO_Port GPIOB

/* USER CODE BEGIN Private defines */
#define FLASH_ADDR 0x0801F800   //  数据存储区域地址
/* USER CODE END Private defines */

#ifdef __cplusplus
}
#endif

#endif /* __MAIN_H */
