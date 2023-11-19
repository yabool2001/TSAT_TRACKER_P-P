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
#include "stm32g0xx_hal.h"

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */
#include <string.h>
#include <stdio.h>
#include <stdbool.h>
#include "my_lis2dw12.h"
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
#define MCO_Pin GPIO_PIN_0
#define MCO_GPIO_Port GPIOF
#define ACC_INT1_Pin GPIO_PIN_0
#define ACC_INT1_GPIO_Port GPIOA
#define ACC_INT1_EXTI_IRQn EXTI0_1_IRQn
#define ACC_INT2_Pin GPIO_PIN_1
#define ACC_INT2_GPIO_Port GPIOA
#define ACC_INT2_EXTI_IRQn EXTI0_1_IRQn
#define DBG_TX_Pin GPIO_PIN_2
#define DBG_TX_GPIO_Port GPIOA
#define DBG_RX_Pin GPIO_PIN_3
#define DBG_RX_GPIO_Port GPIOA
#define ACC_CS_Pin GPIO_PIN_4
#define ACC_CS_GPIO_Port GPIOA
#define LDG_Pin GPIO_PIN_5
#define LDG_GPIO_Port GPIOA
#define ACC_MISO_Pin GPIO_PIN_6
#define ACC_MISO_GPIO_Port GPIOA
#define ACC_MOSI_Pin GPIO_PIN_7
#define ACC_MOSI_GPIO_Port GPIOA
#define ASTRO_WKUP_Pin GPIO_PIN_0
#define ASTRO_WKUP_GPIO_Port GPIOB
#define ASTRO_RST_Pin GPIO_PIN_1
#define ASTRO_RST_GPIO_Port GPIOB
#define ASTRO_EVT_Pin GPIO_PIN_2
#define ASTRO_EVT_GPIO_Port GPIOB
#define GNSS_GEOF_Pin GPIO_PIN_10
#define GNSS_GEOF_GPIO_Port GPIOB
#define GNSS_RST_Pin GPIO_PIN_13
#define GNSS_RST_GPIO_Port GPIOB
#define GNSS_PWR_SW_Pin GPIO_PIN_15
#define GNSS_PWR_SW_GPIO_Port GPIOB
#define ACC_SCK_Pin GPIO_PIN_8
#define ACC_SCK_GPIO_Port GPIOD
#define TMS_Pin GPIO_PIN_13
#define TMS_GPIO_Port GPIOA
#define TCK_Pin GPIO_PIN_14
#define TCK_GPIO_Port GPIOA
#define GNSS_TX_Pin GPIO_PIN_3
#define GNSS_TX_GPIO_Port GPIOB
#define GNSS_RX_Pin GPIO_PIN_4
#define GNSS_RX_GPIO_Port GPIOB
#define GNSS_3DFIX_Pin GPIO_PIN_5
#define GNSS_3DFIX_GPIO_Port GPIOB
#define GNSS_JAM_Pin GPIO_PIN_6
#define GNSS_JAM_GPIO_Port GPIOB
#define ASTRO_TX_Pin GPIO_PIN_8
#define ASTRO_TX_GPIO_Port GPIOB
#define ASTRO_RX_Pin GPIO_PIN_9
#define ASTRO_RX_GPIO_Port GPIOB

/* USER CODE BEGIN Private defines */
#define HUART_DBG						&huart2
#define HUART_GNSS						&huart5
#define HACC							&hspi1
#define UART_TIMEOUT 					1000
#define UART_TX_MAX_BUFF_SIZE			250
#define TIM_SECONDS_THS_SYSTEM_RESET	120
/* USER CODE END Private defines */

#ifdef __cplusplus
}
#endif

#endif /* __MAIN_H */
