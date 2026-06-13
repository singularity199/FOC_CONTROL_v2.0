// clang-format off
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
extern "C"
{
#endif

/* Includes ------------------------------------------------------------------*/
#include "stm32g4xx_hal.h"

#include "stm32g4xx_ll_adc.h"
#include "stm32g4xx_ll_cordic.h"
#include "stm32g4xx_ll_fmac.h"
#include "stm32g4xx_ll_i2c.h"
#include "stm32g4xx_ll_opamp.h"
#include "stm32g4xx_ll_rcc.h"
#include "stm32g4xx_ll_bus.h"
#include "stm32g4xx_ll_crs.h"
#include "stm32g4xx_ll_system.h"
#include "stm32g4xx_ll_exti.h"
#include "stm32g4xx_ll_cortex.h"
#include "stm32g4xx_ll_utils.h"
#include "stm32g4xx_ll_pwr.h"
#include "stm32g4xx_ll_dma.h"
#include "stm32g4xx_ll_spi.h"
#include "stm32g4xx_ll_tim.h"
#include "stm32g4xx_ll_usart.h"
#include "stm32g4xx_ll_gpio.h"

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */

/* USER CODE END Includes */

/* Exported types ------------------------------------------------------------*/
/* USER CODE BEGIN ET */
extern uint32_t debug_it;
/* USER CODE END ET */

/* Exported constants --------------------------------------------------------*/
/* USER CODE BEGIN EC */

/* USER CODE END EC */

/* Exported macro ------------------------------------------------------------*/
/* USER CODE BEGIN EM */

/* USER CODE END EM */

/* Exported functions prototypes ---------------------------------------------*/
void Error_Handler(void);
void DWT_Init();
/* USER CODE BEGIN EFP */

/* USER CODE END EFP */

/* Private defines -----------------------------------------------------------*/
#define TIM_PWM_ARR 280
#define LED_PWM_ARR 102
#define OLED_I2C_ADDR 0x78
#define U_P_Pin                 LL_GPIO_PIN_1
#define U_P_GPIO_Port           GPIOA
#define U_OUT_Pin               LL_GPIO_PIN_2
#define U_OUT_GPIO_Port         GPIOA
#define U_M_Pin                 LL_GPIO_PIN_3
#define U_M_GPIO_Port           GPIOA
#define V_P_Pin                 LL_GPIO_PIN_5
#define V_P_GPIO_Port           GPIOA
#define V_OUT_Pin               LL_GPIO_PIN_6
#define V_OUT_GPIO_Port         GPIOA
#define V_M_Pin                 LL_GPIO_PIN_7
#define V_M_GPIO_Port           GPIOA
#define W_P_Pin                 LL_GPIO_PIN_0
#define W_P_GPIO_Port           GPIOB
#define W_OUT_Pin               LL_GPIO_PIN_1
#define W_OUT_GPIO_Port         GPIOB
#define W_M_Pin                 LL_GPIO_PIN_2
#define W_M_GPIO_Port           GPIOB
#define SPI_SS1_Pin             LL_GPIO_PIN_12
#define SPI_SS1_GPIO_Port       GPIOB
#define PWM_CH1N_Pin            LL_GPIO_PIN_13
#define PWM_CH1N_GPIO_Port      GPIOB
#define PWM_CH2N_Pin            LL_GPIO_PIN_14
#define PWM_CH2N_GPIO_Port      GPIOB
#define PWM_CH3N_Pin            LL_GPIO_PIN_15
#define PWM_CH3N_GPIO_Port      GPIOB
#define PWM_CH1_Pin             LL_GPIO_PIN_8
#define PWM_CH1_GPIO_Port       GPIOA
#define PWM_CH2_Pin             LL_GPIO_PIN_9
#define PWM_CH2_GPIO_Port       GPIOA
#define PWM_CH3_Pin             LL_GPIO_PIN_10
#define PWM_CH3_GPIO_Port       GPIOA
#define LED_STATUS_Pin          LL_GPIO_PIN_6
#define LED_STATUS_GPIO_Port    GPIOB

/* USER CODE BEGIN Private defines */
#define SQRT3 1.73205081f
#define POLE_PAIRS 7
#define V_BUS 25.0f
#define PI 3.14159265358979323846f
#define ADC_Coefficient 0.0537109375f
#define ADC_Offset 1425
#define ADC_FILTER_ALPHA 0.005f
#define USE_TIM_UPDATE_IT 0
#define USE_ADC_INJECT_IT 1
#define DETECT_RPM 0


#define DEBUG 0
#define RUN 1
/* USER CODE END Private defines */

#ifdef __cplusplus
}
#endif

#endif /* __MAIN_H */
