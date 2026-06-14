/* USER CODE BEGIN Header */
/**
 ******************************************************************************
 * @file           : main.c
 * @brief          : Main program body
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
#include "main.h"
#include "adc.h"
#include "cordic.h"
#include "fdcan.h"
#include "fmac.h"
#include "i2c.h"
#include "opamp.h"
#include "spi.h"
#include "tim.h"
#include "usart.h"
#include "usb_device.h"
#include "gpio.h"

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */
#include "oled.h"
#include "motor_control.h"
#include "usbd_cdc_if.h"
/* USER CODE END Includes */

/* Private typedef -----------------------------------------------------------*/
/* USER CODE BEGIN PTD */

/* USER CODE END PTD */

/* Private define ------------------------------------------------------------*/
/* USER CODE BEGIN PD */

/* USER CODE END PD */

/* Private macro -------------------------------------------------------------*/
/* USER CODE BEGIN PM */

/* USER CODE END PM */

/* Private variables ---------------------------------------------------------*/

/* USER CODE BEGIN PV */
uint32_t debug_it = 0;
float usb_array[14];
/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
/* USER CODE BEGIN PFP */

/* USER CODE END PFP */

/* Private user code ---------------------------------------------------------*/
/* USER CODE BEGIN 0 */
uint32_t it_count = 0;
/* USER CODE END 0 */

/**
 * @brief  The application entry point.
 * @retval int
 */
#if RUN
int main(void)
{

    /* USER CODE BEGIN 1 */

    /* USER CODE END 1 */

    /* MCU Configuration--------------------------------------------------------*/

    /* Reset of all peripherals, Initializes the Flash interface and the Systick. */
    HAL_Init();

    /* USER CODE BEGIN Init */

    /* USER CODE END Init */

    /* Configure the system clock */
    SystemClock_Config();

    /* USER CODE BEGIN SysInit */
    LL_APB2_GRP1_EnableClock(LL_APB2_GRP1_PERIPH_SYSCFG);
    /* USER CODE END SysInit */

    /* Initialize all configured peripherals */
    MX_GPIO_Init();
    MX_ADC1_Init();
    MX_ADC2_Init();
    MX_FDCAN1_Init();
    MX_I2C1_Init();
    MX_OPAMP1_Init();
    MX_OPAMP2_Init();
    MX_OPAMP3_Init();
    MX_SPI1_Init();
    MX_TIM1_Init();
    MX_TIM4_Init();
    MX_USART3_UART_Init();
    MX_USB_Device_Init();
    MX_CORDIC_Init();
    MX_FMAC_Init();
    /* USER CODE BEGIN 2 */
    OLED_Init();
    SPEED_LOOP_TIM_Init();
    LL_mDelay(100);
    OLED_Printf(0, 0, "SYSTEM:INIT!");
    LL_SPI_Enable(SPI1);
    LL_OPAMP_Enable(OPAMP1);
    LL_OPAMP_Enable(OPAMP2);
    LL_OPAMP_Enable(OPAMP3);
    LL_mDelay(100);
    OLED_Update();

    OLED_Printf(0, 2, "OPAMP:START!");
    LL_mDelay(10);
    DWT_Init();
    FOC_ADC_Start();
    LL_mDelay(10);
    OLED_Update();

    OLED_Printf(0, 4, "ADC:START!");
    OLED_Printf(0, 6, "READY!");
    OLED_Update();
    LL_mDelay(2000);
    OLED_Clear();
    OLED_Update();

    ((uint32_t *)&(usb_array))[13] = 0X7F800000;
    /* USER CODE END 2 */

    /* Infinite loop */
    /* USER CODE BEGIN WHILE */

    uint16_t last_angle_machanine = 0;
    uint32_t last_tick = 0;

    foc.last_tick = 0;
    while (1)
    {
        /* USER CODE END WHILE */

        /* USER CODE BEGIN 3 */
        // foc.start_tick = DWT->CYCCNT;
        // LL_mDelay(100);
        // foc.end_tick = DWT->CYCCNT;
        // OLED_Printf(0, 0, "start:%d", foc.start_tick);
        // OLED_Printf(0, 1, "end:%d", foc.end_tick);

        // uint8_t tail[4] = {0x00, 0x00, 0x80, 0x7f};
        // UART_Write((uint8_t *)&angle_el, 4);
        // UART_Write(tail, 4);

        // uint16_t last_angle_machanine = SPI1_read(0xffff) & 0x3fff;
        /************************************************************************************** */

        // uint16_t new_angle_machanine = foc.angle_machanine << 2;
        // uint32_t new_tick = DWT->CYCCNT;
        // usb_array[0] = (1 - ADC_FILTER_ALPHA) * usb_array[0] + ADC_FILTER_ALPHA * ((float)((int16_t)(new_angle_machanine - last_angle_machanine)) / 65535.0f) / ((float)(new_tick - last_tick) / 170000000.0f);
        // last_angle_machanine = foc.angle_machanine << 2;
        // last_tick = DWT->CYCCNT;
        usb_array[0] = foc.real_speed;
        /*************************************************************************************** */

        // OLED_Printf(0, 0, "%3d", it_count);
        // OLED_Update();

        // foc.start_tick = DWT->CYCCNT;
        // OLED_Printf(0, 2, "%8x", foc.adc.v_a);
        // foc.end_tick = DWT->CYCCNT;
        // OLED_Printf(0, 4, "%8x", foc.adc.v_c);
        // OLED_Printf(0, 6, "%8x", foc.adc.v_b);
        // // OLED_Printf(0, 0, "tick:%x", foc.end_tick - foc.start_tick);
        // OLED_Update();
        /*************************************************************************************** */

        // usb_array[0] = ((float)((foc.angle_machanine + 0x3fff - foc.angle_error) & 0x3fff)) / 16383.0f;

        // usb_array[0].i_u = foc.adc.v_a;
        // usb_array[1].i_v = foc.adc.v_b;
        // usb_array[2].i_w = foc.adc.v_c;
        // usb_array[3] = foc.adc.v_a + foc.adc.v_b + foc.adc.v_c;
        usb_array[1] = foc.pid_d.error;
        usb_array[2] = foc.pid_q.error;
        usb_array[3] = foc.pid_d.error_sum;
        usb_array[4] = foc.pid_q.error_sum;
        usb_array[5] = foc.pid_d.output;
        usb_array[6] = foc.pid_q.output;
        // usb_array[5] = foc.pid_d.output * foc.real_park.i_d;
        // usb_array[6] = foc.pid_q.output * foc.real_park.i_q;

        // usb_array[3] = foc.target_clark.v_alpha;
        // usb_array[4] = foc.target_clark.v_beta;

        // usb_array[5] = foc.real_clark.v_alpha;
        // usb_array[6] = foc.real_clark.v_beta;

        usb_array[7] = foc.real_park_i.d;
        usb_array[8] = foc.real_park_i.q;
        usb_array[9] = foc.real_park_i.q * foc.real_park_i.q + foc.real_park_i.d * foc.real_park_i.d;
        // usb_array[10] = foc.adc.i_a;
        // usb_array[11] = foc.adc.i_b;
        // usb_array[12] = foc.adc.i_c;
        usb_array[10] = foc.pid_speed.error;
        usb_array[11] = foc.pid_speed.error_sum;
        usb_array[12] = foc.pid_speed.output;

        while (CDC_Transmit_FS((uint8_t *)&usb_array, sizeof(usb_array)) != USBD_OK)
            ;
        // CDC_Transmit_FS((uint8_t *)&usb_array, sizeof(usb_array));
    }
    /* USER CODE END 3 */
}
#endif
#if DEBUG
int main(void)
{

    HAL_Init();

    SystemClock_Config();

    LL_APB2_GRP1_EnableClock(LL_APB2_GRP1_PERIPH_SYSCFG);
    MX_GPIO_Init();
    MX_ADC1_Init();
    MX_ADC2_Init();
    MX_FDCAN1_Init();
    MX_I2C1_Init();
    OLED_Init();
    LL_mDelay(100);
    OLED_Clear();
    OLED_Printf(0, 0, "ready");
    LL_mDelay(1000);
    // MX_OPAMP1_Init();
    // MX_OPAMP2_Init();
    // MX_OPAMP3_Init();

    // LL_SPI_Enable(SPI1);
    // LL_OPAMP_Enable(OPAMP1);
    // LL_OPAMP_Enable(OPAMP2);
    // LL_OPAMP_Enable(OPAMP3);
    volatile uint16_t v_a = 0;
    while (1)
    {
        LL_ADC_INJ_StartConversion(ADC1);

        // 3. 等待注入组转换完成标志位 (JEOS - Injected End of Sequence)
        // 注意：不同系列的 STM32 标志位宏可能略有不同，如 JEOC。
        while (LL_ADC_IsActiveFlag_JEOS(ADC1) == 0)
        {
            // 等待转换结束
        }

        // 4. 清除标志位，为下一次转换做准备
        LL_ADC_ClearFlag_JEOS(ADC1);

        // 5. 读取转换数据
        v_a = LL_ADC_INJ_ReadConversionData12(ADC1, LL_ADC_INJ_RANK_1);
        OLED_Printf(0, 0, "%d", v_a);
    }
}
#endif

/**
 * @brief System Clock Configuration
 * @retval None
 */
void SystemClock_Config(void)
{
    LL_FLASH_SetLatency(LL_FLASH_LATENCY_4);
    while (LL_FLASH_GetLatency() != LL_FLASH_LATENCY_4)
    {
    }
    LL_PWR_EnableRange1BoostMode();
    LL_RCC_HSE_Enable();
    /* Wait till HSE is ready */
    while (LL_RCC_HSE_IsReady() != 1)
    {
    }

    LL_RCC_HSI48_Enable();
    /* Wait till HSI48 is ready */
    while (LL_RCC_HSI48_IsReady() != 1)
    {
    }

    LL_RCC_PLL_ConfigDomain_SYS(LL_RCC_PLLSOURCE_HSE, LL_RCC_PLLM_DIV_3, 85, LL_RCC_PLLR_DIV_2);
    LL_RCC_PLL_EnableDomain_SYS();
    LL_RCC_PLL_Enable();
    /* Wait till PLL is ready */
    while (LL_RCC_PLL_IsReady() != 1)
    {
    }

    LL_RCC_SetSysClkSource(LL_RCC_SYS_CLKSOURCE_PLL);
    LL_RCC_SetAHBPrescaler(LL_RCC_SYSCLK_DIV_1);
    /* Wait till System clock is ready */
    while (LL_RCC_GetSysClkSource() != LL_RCC_SYS_CLKSOURCE_STATUS_PLL)
    {
    }

    /* Insure 1us transition state at intermediate medium speed clock*/
    for (__IO uint32_t i = (170 >> 1); i != 0; i--)
        ;

    /* Set AHB prescaler*/
    LL_RCC_SetAHBPrescaler(LL_RCC_SYSCLK_DIV_1);
    LL_RCC_SetAPB1Prescaler(LL_RCC_APB1_DIV_1);
    LL_RCC_SetAPB2Prescaler(LL_RCC_APB2_DIV_1);
    LL_SetSystemCoreClock(170000000);

    /* Update the time base */
    if (HAL_InitTick(TICK_INT_PRIORITY) != HAL_OK)
    {
        Error_Handler();
    }
    LL_CRS_SetSyncDivider(LL_CRS_SYNC_DIV_1);
    LL_CRS_SetSyncPolarity(LL_CRS_SYNC_POLARITY_RISING);
    LL_CRS_SetSyncSignalSource(LL_CRS_SYNC_SOURCE_USB);
    LL_CRS_SetReloadCounter(__LL_CRS_CALC_CALCULATE_RELOADVALUE(48000000, 1000));
    LL_CRS_SetFreqErrorLimit(34);
    LL_CRS_SetHSI48SmoothTrimming(32);
}

/* USER CODE BEGIN 4 */
void DWT_Init()
{
    CoreDebug->DEMCR |= CoreDebug_DEMCR_TRCENA_Msk;
    DWT->CYCCNT = 0;
    DWT->CTRL |= DWT_CTRL_CYCCNTENA_Msk;
}
/* USER CODE END 4 */

/**
 * @brief  This function is executed in case of error occurrence.
 * @retval None
 */
void Error_Handler(void)
{
    /* USER CODE BEGIN Error_Handler_Debug */
    /* User can add his own implementation to report the HAL error return state */
    __disable_irq();
    while (1)
    {
    }
    /* USER CODE END Error_Handler_Debug */
}
#ifdef USE_FULL_ASSERT
/**
 * @brief  Reports the name of the source file and the source line number
 *         where the assert_param error has occurred.
 * @param  file: pointer to the source file name
 * @param  line: assert_param error line source number
 * @retval None
 */
void assert_failed(uint8_t *file, uint32_t line)
{
    /* USER CODE BEGIN 6 */
    /* User can add his own implementation to report the file name and line number,
       ex: printf("Wrong parameters value: file %s on line %d\r\n", file, line) */
    /* USER CODE END 6 */
}
#endif /* USE_FULL_ASSERT */
