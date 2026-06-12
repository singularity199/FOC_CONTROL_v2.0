/* USER CODE BEGIN Header */
/**
 ******************************************************************************
 * @file    cordic.c
 * @brief   This file provides code for the configuration
 *          of the CORDIC instances.
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
/* Includes ------------------------------------------------------------------*/
#include "cordic.h"

/* USER CODE BEGIN 0 */

/* USER CODE END 0 */

/* CORDIC init function */
void MX_CORDIC_Init(void)
{

  /* USER CODE BEGIN CORDIC_Init 0 */

  /* USER CODE END CORDIC_Init 0 */

  /* Peripheral clock enable */
  LL_AHB1_GRP1_EnableClock(LL_AHB1_GRP1_PERIPH_CORDIC);

  /* USER CODE BEGIN CORDIC_Init 1 */
  LL_CORDIC_SetFunction(CORDIC, LL_CORDIC_FUNCTION_SINE);
  LL_CORDIC_SetPrecision(CORDIC, LL_CORDIC_PRECISION_6CYCLES);
  LL_CORDIC_SetScale(CORDIC, LL_CORDIC_SCALE_0);
  LL_CORDIC_SetNbWrite(CORDIC, LL_CORDIC_NBWRITE_1);
  LL_CORDIC_SetNbRead(CORDIC, LL_CORDIC_NBREAD_2);
  LL_CORDIC_SetInSize(CORDIC, LL_CORDIC_INSIZE_32BITS);
  LL_CORDIC_SetOutSize(CORDIC, LL_CORDIC_OUTSIZE_32BITS);
  /* USER CODE END CORDIC_Init 1 */

  /* nothing else to be configured */

  /* USER CODE BEGIN CORDIC_Init 2 */

  /* USER CODE END CORDIC_Init 2 */
}

/* USER CODE BEGIN 1 */

/* USER CODE END 1 */
