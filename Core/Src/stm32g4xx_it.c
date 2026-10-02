/* USER CODE BEGIN Header */
/**
 ******************************************************************************
 * @file    stm32g4xx_it.c
 * @brief   Interrupt Service Routines.
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
#include "stm32g4xx_it.h"
/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */
#include "spi.h"
#include "oled.h"
#include "motor_control.h"
#include "s_curve.h"
#include "remote_control.h"
#include "nonlinear_flux_observer.h"
#include "math.h"
#include "Sguan_NLFO.h"
/* USER CODE END Includes */

/* Private typedef -----------------------------------------------------------*/
/* USER CODE BEGIN TD */

/* USER CODE END TD */

/* Private define ------------------------------------------------------------*/
/* USER CODE BEGIN PD */

/* USER CODE END PD */

/* Private macro -------------------------------------------------------------*/
/* USER CODE BEGIN PM */

/* USER CODE END PM */

/* Private variables ---------------------------------------------------------*/
/* USER CODE BEGIN PV */

uint16_t last_angle_machanine = 0;
uint32_t last_tick = 0;
extern FDCAN_HandleTypeDef hfdcan1;
extern float usb_array[23];
extern NLFO_STRUCT nlfo;

typedef struct
{
    uint8_t initialized;
    uint8_t ready;

    uint16_t last_theta_q16;
    int8_t last_dir;

    uint32_t travel_q16;
    uint16_t sample_count;

    float weight_sum;
    float pm_alpha_sum;
    float pm_beta_sum;

    float pm_alpha_avg;
    float pm_beta_avg;
} PM_CycleAvg_t;

static PM_CycleAvg_t pm_cycle_avg = {0};

/* 给 VOFA / USB 用的调试量 */
volatile float nfo_pm_alpha_avg_dbg = 0.0f;
volatile float nfo_pm_beta_avg_dbg = 0.0f;

volatile float nfo_pm_alpha_bias_norm_dbg = 0.0f;
volatile float nfo_pm_beta_bias_norm_dbg = 0.0f;

volatile float nfo_pm_enc_raw_deg_dbg = 0.0f;
volatile float nfo_pm_enc_centered_deg_dbg = 0.0f;

volatile float nfo_pm_centered_alpha_dbg = 0.0f;
volatile float nfo_pm_centered_beta_dbg = 0.0f;
/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
/* USER CODE BEGIN PFP */

/* USER CODE END PFP */

/* Private user code ---------------------------------------------------------*/
/* USER CODE BEGIN 0 */
static inline uint16_t ABS_I16_TO_U16(int16_t x)
{
    return (x >= 0) ? (uint16_t)x : (uint16_t)(-x);
}

/*
 * 用当前磁编码器机械角 + angle_error 算 14bit 电角度。
 * 不用 foc.angle_el 的 float，避免浮点比例误差。
 */
static inline uint16_t FOC_GetElecAngle14_Debug(void)
{
    uint16_t mech = foc.angle_machanine & 0x3FFFu;
    uint16_t zero = foc.angle_error & 0x3FFFu;

    uint16_t mech_delta = (uint16_t)((mech + 0x4000u - zero) & 0x3FFFu);

    return (uint16_t)(((uint32_t)mech_delta * POLE_PAIRS) & 0x3FFFu);
}

static void PM_CycleAvg_ResetAccum(PM_CycleAvg_t *avg)
{
    avg->travel_q16 = 0u;
    avg->sample_count = 0u;
    avg->weight_sum = 0.0f;
    avg->pm_alpha_sum = 0.0f;
    avg->pm_beta_sum = 0.0f;
}

/*
 * 每走过 1 个电角周期，更新一次 pm_alpha_avg / pm_beta_avg。
 *
 * theta_e_14: 14bit 电角度，0~16383
 * pm_alpha / pm_beta: NFO 当前估计的永磁磁链矢量
 */
static void PM_CycleAvg_Update(PM_CycleAvg_t *avg,
                               uint16_t theta_e_14,
                               float pm_alpha,
                               float pm_beta)
{
    uint16_t theta_q16 = (uint16_t)(theta_e_14 << 2); // 14bit -> 16bit angle

    if (avg->initialized == 0u)
    {
        avg->initialized = 1u;
        avg->ready = 0u;
        avg->last_theta_q16 = theta_q16;
        avg->last_dir = 0;
        PM_CycleAvg_ResetAccum(avg);
        return;
    }

    /*
     * 16bit 环形角度差。
     * 正转为正，反转为负。
     */
    int16_t dtheta_q16 = (int16_t)(theta_q16 - avg->last_theta_q16);
    avg->last_theta_q16 = theta_q16;

    uint16_t step_q16 = ABS_I16_TO_U16(dtheta_q16);

    /*
     * 低速或重复采样，角度没变化，跳过。
     */
    if (step_q16 == 0u)
    {
        return;
    }

    int8_t dir = (dtheta_q16 >= 0) ? 1 : -1;

    /*
     * 如果中途反向，当前周期平均没有意义，清掉重来。
     */
    if ((avg->last_dir != 0) && (dir != avg->last_dir))
    {
        PM_CycleAvg_ResetAccum(avg);
        avg->last_dir = dir;
        return;
    }

    avg->last_dir = dir;

    /*
     * 用角度增量作为权重。
     * 这样是按电角度平均，不是按时间平均。
     */
    float w = (float)step_q16;

    avg->pm_alpha_sum += pm_alpha * w;
    avg->pm_beta_sum += pm_beta * w;
    avg->weight_sum += w;
    avg->travel_q16 += step_q16;
    avg->sample_count++;

    /*
     * 一个完整电角周期 = 65536 q16 angle count。
     * sample_count > 16 防止异常跳变导致假更新。
     */
    if ((avg->travel_q16 >= 65536u) && (avg->sample_count > 16u))
    {
        float inv_w = 1.0f / avg->weight_sum;

        avg->pm_alpha_avg = avg->pm_alpha_sum * inv_w;
        avg->pm_beta_avg = avg->pm_beta_sum * inv_w;
        avg->ready = 1u;

        PM_CycleAvg_ResetAccum(avg);
    }
}

/*
 * 不用 atan2，直接用叉积看 pm 相对编码器角度的误差。
 * 输出单位近似 degree。
 */
static void NFO_PM_Debug_Update(uint16_t theta_enc_14)
{
    uint32_t theta_enc_q32 = ((uint32_t)theta_enc_14) << 18;
    cordic_angle enc = cor_q32(theta_enc_q32);

    /*
     * 原始 pm，使用 psi_f 归一化。
     * 如果 pm_abs 波动很大，也可以改成用 nfo.pm_abs 归一化。
     */
    float pm_raw_cos = nfo.pm_alpha * nfo.inv_psi_f;
    float pm_raw_sin = nfo.pm_beta * nfo.inv_psi_f;

    /*
     * raw: sin(theta_pm - theta_enc)
     * >0 表示 pm 超前编码器电角度。
     */
    float pm_enc_raw_sin =
        pm_raw_sin * enc.cos_angle - pm_raw_cos * enc.sin_angle;

    nfo_pm_enc_raw_deg_dbg = pm_enc_raw_sin * 57.2958f;

    /*
     * 去掉一个电角周期内的 pm 圆心偏置。
     */
    float pm_alpha_centered = nfo.pm_alpha;
    float pm_beta_centered = nfo.pm_beta;

    if (pm_cycle_avg.ready != 0u)
    {
        pm_alpha_centered -= pm_cycle_avg.pm_alpha_avg;
        pm_beta_centered -= pm_cycle_avg.pm_beta_avg;
    }

    nfo_pm_centered_alpha_dbg = pm_alpha_centered;
    nfo_pm_centered_beta_dbg = pm_beta_centered;

    float pm_c_cos = pm_alpha_centered * nfo.inv_psi_f;
    float pm_c_sin = pm_beta_centered * nfo.inv_psi_f;

    float pm_enc_centered_sin =
        pm_c_sin * enc.cos_angle - pm_c_cos * enc.sin_angle;

    nfo_pm_enc_centered_deg_dbg = pm_enc_centered_sin * 57.2958f;

    nfo_pm_alpha_avg_dbg = pm_cycle_avg.pm_alpha_avg;
    nfo_pm_beta_avg_dbg = pm_cycle_avg.pm_beta_avg;

    nfo_pm_alpha_bias_norm_dbg = pm_cycle_avg.pm_alpha_avg * nfo.inv_psi_f;
    nfo_pm_beta_bias_norm_dbg = pm_cycle_avg.pm_beta_avg * nfo.inv_psi_f;
}
/* USER CODE END 0 */

/* External variables --------------------------------------------------------*/
extern PCD_HandleTypeDef hpcd_USB_FS;
/* USER CODE BEGIN EV */

/* USER CODE END EV */

/******************************************************************************/
/*           Cortex-M4 Processor Interruption and Exception Handlers          */
/******************************************************************************/
/**
 * @brief This function handles Non maskable interrupt.
 */
void NMI_Handler(void)
{
    /* USER CODE BEGIN NonMaskableInt_IRQn 0 */

    /* USER CODE END NonMaskableInt_IRQn 0 */
    /* USER CODE BEGIN NonMaskableInt_IRQn 1 */
    while (1)
    {
    }
    /* USER CODE END NonMaskableInt_IRQn 1 */
}

/**
 * @brief This function handles Hard fault interrupt.
 */
void HardFault_Handler(void)
{
    /* USER CODE BEGIN HardFault_IRQn 0 */

    /* USER CODE END HardFault_IRQn 0 */
    while (1)
    {
        /* USER CODE BEGIN W1_HardFault_IRQn 0 */
        /* USER CODE END W1_HardFault_IRQn 0 */
    }
}

/**
 * @brief This function handles Memory management fault.
 */
void MemManage_Handler(void)
{
    /* USER CODE BEGIN MemoryManagement_IRQn 0 */

    /* USER CODE END MemoryManagement_IRQn 0 */
    while (1)
    {
        /* USER CODE BEGIN W1_MemoryManagement_IRQn 0 */
        /* USER CODE END W1_MemoryManagement_IRQn 0 */
    }
}

/**
 * @brief This function handles Prefetch fault, memory access fault.
 */
void BusFault_Handler(void)
{
    /* USER CODE BEGIN BusFault_IRQn 0 */

    /* USER CODE END BusFault_IRQn 0 */
    while (1)
    {
        /* USER CODE BEGIN W1_BusFault_IRQn 0 */
        /* USER CODE END W1_BusFault_IRQn 0 */
    }
}

/**
 * @brief This function handles Undefined instruction or illegal state.
 */
void UsageFault_Handler(void)
{
    /* USER CODE BEGIN UsageFault_IRQn 0 */

    /* USER CODE END UsageFault_IRQn 0 */
    while (1)
    {
        /* USER CODE BEGIN W1_UsageFault_IRQn 0 */
        /* USER CODE END W1_UsageFault_IRQn 0 */
    }
}

/**
 * @brief This function handles System service call via SWI instruction.
 */
void SVC_Handler(void)
{
    /* USER CODE BEGIN SVCall_IRQn 0 */

    /* USER CODE END SVCall_IRQn 0 */
    /* USER CODE BEGIN SVCall_IRQn 1 */

    /* USER CODE END SVCall_IRQn 1 */
}

/**
 * @brief This function handles Debug monitor.
 */
void DebugMon_Handler(void)
{
    /* USER CODE BEGIN DebugMonitor_IRQn 0 */

    /* USER CODE END DebugMonitor_IRQn 0 */
    /* USER CODE BEGIN DebugMonitor_IRQn 1 */

    /* USER CODE END DebugMonitor_IRQn 1 */
}

/**
 * @brief This function handles Pendable request for system service.
 */
void PendSV_Handler(void)
{
    /* USER CODE BEGIN PendSV_IRQn 0 */

    /* USER CODE END PendSV_IRQn 0 */
    /* USER CODE BEGIN PendSV_IRQn 1 */

    /* USER CODE END PendSV_IRQn 1 */
}

/**
 * @brief This function handles System tick timer.
 */
void SysTick_Handler(void)
{
    /* USER CODE BEGIN SysTick_IRQn 0 */

    /* USER CODE END SysTick_IRQn 0 */
    HAL_IncTick();
    /* USER CODE BEGIN SysTick_IRQn 1 */

    /* USER CODE END SysTick_IRQn 1 */
}

/******************************************************************************/
/* STM32G4xx Peripheral Interrupt Handlers                                    */
/* Add here the Interrupt Handlers for the used peripherals.                  */
/* For the available peripheral interrupt handler names,                      */
/* please refer to the startup file (startup_stm32g4xx.s).                    */
/******************************************************************************/

/**
 * @brief This function handles ADC1 and ADC2 global interrupt.
 */
void ADC1_2_IRQHandler(void)
{
    /* USER CODE BEGIN ADC1_2_IRQn 0 */
    if (LL_ADC_IsActiveFlag_JEOS(ADC1) != 0)
    {
        LL_ADC_ClearFlag_JEOS(ADC1);
// foc_current_loop_control(foc.target_park_i.d, foc.target_park_i.q);
#define NFO_VOLTAGE_SCALE 1.0f

        float v_alpha_obs =
            foc.target_clark_v.alpha *
            NFO_VOLTAGE_SCALE;

        float v_beta_obs =
            foc.target_clark_v.beta *
            NFO_VOLTAGE_SCALE;

        foc_current_loop_control(0.0f, foc.target_park_i.q);

        NFO_Update(
            &nfo,
            v_alpha_obs,
            v_beta_obs,
            foc.real_clark_i.alpha,
            foc.real_clark_i.beta);
        nlfo.go.Input_Ualpha = v_alpha_obs;
        nlfo.go.Input_Ubeta = v_beta_obs;
        nlfo.go.Input_Ialpha = foc.real_clark_i.alpha;
        nlfo.go.Input_Ibeta = foc.real_clark_i.beta;
        NLFO_Loop(&nlfo);

        SMO_Update(
            &smo,
            v_alpha_obs,
            v_beta_obs,
            foc.real_clark_i.alpha,
            foc.real_clark_i.beta);
        usb_array[14] = nfo.theta_e;
        usb_array[15] = atan2f(nlfo.go.Output_Sine, nlfo.go.Output_Cosine);
        usb_array[16] = smo.theta_compensated;
    }
    /* USER CODE END ADC1_2_IRQn 0 */
    /* USER CODE BEGIN ADC1_2_IRQn 1 */

    /* USER CODE END ADC1_2_IRQn 1 */
}
/**
 * @brief This function handles FDCAN1 interrupt 0.
 */
void FDCAN1_IT0_IRQHandler(void)
{
    /* USER CODE BEGIN FDCAN1_IT0_IRQn 0 */

    /* USER CODE END FDCAN1_IT0_IRQn 0 */
    HAL_FDCAN_IRQHandler(&hfdcan1);
    /* USER CODE BEGIN FDCAN1_IT0_IRQn 1 */

    /* USER CODE END FDCAN1_IT0_IRQn 1 */
}
/**
 * @brief This function handles USB low priority interrupt remap.
 */
void USB_LP_IRQHandler(void)
{
    /* USER CODE BEGIN USB_LP_IRQn 0 */

    /* USER CODE END USB_LP_IRQn 0 */
    HAL_PCD_IRQHandler(&hpcd_USB_FS);
    /* USER CODE BEGIN USB_LP_IRQn 1 */

    /* USER CODE END USB_LP_IRQn 1 */
}

/**
 * @brief This function handles EXTI line[15:10] interrupts.
 */
void EXTI15_10_IRQHandler(void)
{
    /* USER CODE BEGIN EXTI15_10_IRQn 0 */

    /* USER CODE END EXTI15_10_IRQn 0 */
    if (LL_EXTI_IsActiveFlag_0_31(LL_EXTI_LINE_13) != RESET)
    {
        LL_EXTI_ClearFlag_0_31(LL_EXTI_LINE_13);
        /* USER CODE BEGIN LL_EXTI_LINE_13 */

        motor_init();
        LL_TIM_EnableIT_UPDATE(TIM2);
        LL_TIM_EnableCounter(TIM2);
        /* USER CODE END LL_EXTI_LINE_13 */

        /* USER CODE BEGIN EXTI15_10_IRQn 1 */

        /* USER CODE END EXTI15_10_IRQn 1 */
    }
}
/* USER CODE BEGIN 1 */
void TIM1_UP_TIM16_IRQHandler(void)
{
    /* USER CODE BEGIN TIM1_UP_TIM16_IRQn 0 */
    if (LL_TIM_IsActiveFlag_UPDATE(TIM1) != RESET)
    {
        LL_TIM_ClearFlag_UPDATE(TIM1);
        /* USER CODE BEGIN TIM1_UP_TIM16 */

        // foc_open_loop_voltage_control();
        /* USER CODE END TIM1_UP_TIM16 */
    }
    /* USER CODE BEGIN TIM1_UP_TIM16_IRQn 1 */

    /* USER CODE END TIM1_UP_TIM16_IRQn 1 */
    /* USER CODE END TIM1_UP_TIM16_IRQn 0 */
}
void TIM2_IRQHandler(void)
{
    /* USER CODE BEGIN TIM2_IRQn 0 */
    if (LL_TIM_IsActiveFlag_UPDATE(TIM2) != RESET)
    {
        LL_TIM_ClearFlag_UPDATE(TIM2);
        /* USER CODE BEGIN TIM2 */
        it_count++;
        uint16_t new_angle_machanine = foc.angle_machanine << 2;
        foc.new_tick = DWT->CYCCNT;
        // foc.real_speed = (1 - ADC_FILTER_ALPHA) * foc.real_speed + ADC_FILTER_ALPHA * ((float)((int16_t)(new_angle_machanine - last_angle_machanine)) / 65535.0f) / ((float)(foc.new_tick - last_tick) / 170000000.0f);
        FILTER(0.1f, ((float)((int16_t)(new_angle_machanine - last_angle_machanine)) / 65535.0f) / ((float)(foc.new_tick - last_tick) / 170000000.0f), foc.real_speed);
        last_angle_machanine = foc.angle_machanine << 2;
        last_tick = foc.new_tick;
        if (speed_control_flag == 1)
        {
            SCurve_SetTarget(&speed_curve, foc.target_speed);
            iq_control_flag = 0;
        }
        float scurve_speed = SCurve_Update(&speed_curve);
        if (iq_control_flag == 0)
        {
            foc.target_park_i.q = PI_Calc(&foc.pid_speed, scurve_speed, foc.real_speed);
        }
        /* USER CODE END TIM2 */
    }
    /* USER CODE BEGIN TIM2_IRQn 1 */

    /* USER CODE END TIM2_IRQn 1 */
    /* USER CODE END TIM2_IRQn 0 */
}
/* USER CODE END 1 */
