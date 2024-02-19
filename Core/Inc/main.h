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
#include "stm32f3xx_hal.h"

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
#define PWM_MAX 0xFF
#define ENC2_SIGB_Pin GPIO_PIN_0
#define ENC2_SIGB_GPIO_Port GPIOC
#define ENC2_SIGA_Pin GPIO_PIN_1
#define ENC2_SIGA_GPIO_Port GPIOC
#define ENC1_SIGA_Pin GPIO_PIN_0
#define ENC1_SIGA_GPIO_Port GPIOA
#define ENC1_SIGB_Pin GPIO_PIN_1
#define ENC1_SIGB_GPIO_Port GPIOA
#define USART_TX_Pin GPIO_PIN_2
#define USART_TX_GPIO_Port GPIOA
#define USART_RX_Pin GPIO_PIN_3
#define USART_RX_GPIO_Port GPIOA
#define BRAKE_Pin GPIO_PIN_4
#define BRAKE_GPIO_Port GPIOA
#define DIR_B_LD2_Pin GPIO_PIN_5
#define DIR_B_LD2_GPIO_Port GPIOA
#define DIR_A_Pin GPIO_PIN_6
#define DIR_A_GPIO_Port GPIOA
#define SPI_CS_Pin GPIO_PIN_7
#define SPI_CS_GPIO_Port GPIOA
#define TEMP_ALERT_Pin GPIO_PIN_0
#define TEMP_ALERT_GPIO_Port GPIOB
#define PWM_A_Pin GPIO_PIN_1
#define PWM_A_GPIO_Port GPIOB
#define PWM_A_passive_Pin GPIO_PIN_10
#define PWM_A_passive_GPIO_Port GPIOB
#define SPI_MISO_Pin GPIO_PIN_7
#define SPI_MISO_GPIO_Port GPIOC
#define SPI_CLK_Pin GPIO_PIN_9
#define SPI_CLK_GPIO_Port GPIOA
#define ENC1_SIGA_PASSIVE_Pin GPIO_PIN_10
#define ENC1_SIGA_PASSIVE_GPIO_Port GPIOA
#define TMS_Pin GPIO_PIN_13
#define TMS_GPIO_Port GPIOA
#define TCK_Pin GPIO_PIN_14
#define TCK_GPIO_Port GPIOA
#define SWO_Pin GPIO_PIN_3
#define SWO_GPIO_Port GPIOB
#define PWM_B_Pin GPIO_PIN_4
#define PWM_B_GPIO_Port GPIOB
#define SPI_MOSI_Pin GPIO_PIN_6
#define SPI_MOSI_GPIO_Port GPIOB
#define DBG1_Pin GPIO_PIN_7
#define DBG1_GPIO_Port GPIOB

/* USER CODE BEGIN Private defines */
#define PWM_MAX 0xFF
#define ENC2_SIGB_Pin GPIO_PIN_0
#define ENC2_SIGB_GPIO_Port GPIOC
#define ENC2_SIGA_Pin GPIO_PIN_1
#define ENC2_SIGA_GPIO_Port GPIOC
#define ENC1_SIGA_Pin GPIO_PIN_0
#define ENC1_SIGA_GPIO_Port GPIOA
#define ENC1_SIGB_Pin GPIO_PIN_1
#define ENC1_SIGB_GPIO_Port GPIOA
#define BRAKE_Pin GPIO_PIN_4
#define BRAKE_GPIO_Port GPIOA
#define DIR_B_Pin GPIO_PIN_5
#define DIR_B_GPIO_Port GPIOA
#define DIR_A_Pin GPIO_PIN_6
#define DIR_A_GPIO_Port GPIOA
#define SPI_CS_Pin GPIO_PIN_7
#define SPI_CS_GPIO_Port GPIOA
#define TEMP_ALERT_Pin GPIO_PIN_0
#define TEMP_ALERT_GPIO_Port GPIOB
#define PWM_A_Pin GPIO_PIN_1
#define PWM_A_GPIO_Port GPIOB
#define PWM_A_passive_Pin GPIO_PIN_10
#define PWM_A_passive_GPIO_Port GPIOB
#define SPI_MISO_Pin GPIO_PIN_7
#define SPI_MISO_GPIO_Port GPIOC
#define SPI_CLK_Pin GPIO_PIN_9
#define SPI_CLK_GPIO_Port GPIOA
#define ENC1_SIGA_PASSIVE_Pin GPIO_PIN_10
#define ENC1_SIGA_PASSIVE_GPIO_Port GPIOA
#define DBG2_Pin GPIO_PIN_15
#define DBG2_GPIO_Port GPIOA
#define PWM_B_Pin GPIO_PIN_4
#define PWM_B_GPIO_Port GPIOB
#define SPI_MOSI_Pin GPIO_PIN_6
#define SPI_MOSI_GPIO_Port GPIOB
#define DBG1_Pin GPIO_PIN_7
#define DBG1_GPIO_Port GPIOB
/* USER CODE END Private defines */

#ifdef __cplusplus
}
#endif

#endif /* __MAIN_H */
