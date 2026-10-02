#include "motor_control.h"

foc_state foc;
NLFO_STRUCT nlfo;
NFO_Handle_t nfo;
SMO_t smo;
uint8_t throttle;
/*****************************************************
 * @brief 设置高组态输出————高低相皆输出0
 *
 *****************************************************/
void set_high_phase(uint8_t phase)
{
    switch (phase)
    {

    case 1:
        TIM1->CCER = TIM1->CCER & (~(1 << 2)); // reset CC1NE
        TIM1->CCER = TIM1->CCER | ((1 << 0));  // set CC1E
        TIM1->CCR1 = 0;
        break;
    case 2:
        TIM1->CCER = TIM1->CCER & (~(1 << 6)); // reset CC2NE
        TIM1->CCER = TIM1->CCER | ((1 << 4));  // set CC2E
        TIM1->CCR2 = 0;
        break;
    case 3:
        TIM1->CCER = TIM1->CCER & (~(1 << 10)); // reset CC3NE
        TIM1->CCER = TIM1->CCER | ((1 << 8));   // set CC3E
        TIM1->CCR3 = 0;
        break;
    default:

        break;
    }
}

/****************************************************
 * @brief 设置低电平相（相线拉低接地，下桥导通）
 *****************************************************/
void set_low_phase(uint8_t phase)
{
    switch (phase)
    {
    case 1:
        TIM1->CCER |= (1 << 0); // set CC1E
        TIM1->CCER |= (1 << 2); // set CC1NE
        TIM1->CCR1 = 0;
        break;
    case 2:
        TIM1->CCER |= (1 << 4); // set CC2E
        TIM1->CCER |= (1 << 6); // set CC2NE
        TIM1->CCR2 = 0;
        break;
    case 3:
        TIM1->CCER |= (1 << 8);  // set CC3E
        TIM1->CCER |= (1 << 10); // set CC3NE
        TIM1->CCR3 = 0;
        break;
    }
}

/****************************************************
 * @brief 设置pwm相
 *
 *****************************************************/
void set_pwm_phase(uint8_t phase, uint16_t throttle)
{
    switch (phase)
    {
    case 1:
        TIM1->CCER = TIM1->CCER | ((1 << 0)); // set cc1e
        TIM1->CCER = TIM1->CCER | ((1 << 2)); // set cc1ne
        TIM1->CCR1 = throttle;
        break;
    case 2:
        TIM1->CCER = TIM1->CCER | ((1 << 4)); // set cc2e
        TIM1->CCER = TIM1->CCER | ((1 << 6)); // set cc2ne
        TIM1->CCR2 = throttle;
        break;
    case 3:
        TIM1->CCER = TIM1->CCER | ((1 << 8));  // set cc3e
        TIM1->CCER = TIM1->CCER | ((1 << 10)); // set cc3ne
        TIM1->CCR3 = throttle;
        break;
    default:
        break;
    }
}

void motor_stop_all(void)
{
    disable_phase_output(1);
    disable_phase_output(2);
    disable_phase_output(3);
    TIM1->EGR |= TIM_EGR_COMG;
}

/**
 * @brief  完全关闭某一相的输出 (CCxE = 0, CCxNE = 0)
 * @note   在你的硬件配置下，建议先将 CCR 设为 0，再清除使能位。
 */
void disable_phase_output(uint8_t phase)
{
    switch (phase)
    {
    case 1:
        TIM1->CCR1 = 0; // 先确保占空比为0
        // 清除 CC1E (bit 0) 和 CC1NE (bit 2)
        TIM1->CCER &= ~(TIM_CCER_CC1E | TIM_CCER_CC1NE);
        break;
    case 2:
        TIM1->CCR2 = 0;
        // 清除 CC2E (bit 4) 和 CC2NE (bit 6)
        TIM1->CCER &= ~(TIM_CCER_CC2E | TIM_CCER_CC2NE);
        break;
    case 3:
        TIM1->CCR3 = 0;
        // 清除 CC3E (bit 8) 和 CC3NE (bit 10)
        TIM1->CCER &= ~(TIM_CCER_CC3E | TIM_CCER_CC3NE);
        break;
    }
}

/*****************************************************
 * @brief 六部换向步骤
 *
 *****************************************************/
void motor_step(uint8_t step)
{
    TIM1->CCER = 0;
    uint16_t actual_ccr = (uint16_t)(((uint32_t)throttle * TIM_PWM_ARR) / 1024);
    switch (step)
    {
    case 0:
        set_high_phase(3);
        set_low_phase(2);
        set_pwm_phase(1, actual_ccr);
        break;
    case 1:
        set_high_phase(2);
        set_low_phase(3);
        set_pwm_phase(1, actual_ccr);
        break;
    case 2:
        set_high_phase(1);
        set_low_phase(3);
        set_pwm_phase(2, actual_ccr);
        break;
    case 3:
        set_high_phase(3);
        set_low_phase(1);
        set_pwm_phase(2, actual_ccr);
        break;
    case 4:
        set_high_phase(2);
        set_low_phase(1);
        set_pwm_phase(3, actual_ccr);
        break;
    case 5:
        set_high_phase(1);
        set_low_phase(2);
        set_pwm_phase(3, actual_ccr);
        break;
    default:
        break;
    }
    TIM1->EGR |= TIM_EGR_COMG;
}

float get_angle_el(void)
{
    // AS5047P

    foc.angle_machanine = SPI1_read(0xffff) & 0x3fff;
    float angle_el = ((float)(((foc.angle_machanine + 0x3fff - foc.angle_error) * POLE_PAIRS) & 0x3fff)) * 2 * PI / 16383.0f;

    return angle_el;
    // open loop
}
void motor_smo_init(void)
{
    smo.config.rs_ohm = BLDC_PHASE_Rs;
    smo.config.ls_h = BLDC_PHASE_L;
    smo.config.ts_s = FOC_CONTROL_Ts;
    smo.config.k_slide_v = 15.0f;
    smo.config.boundary_a = 0.5f;
    smo.config.emf_lpf_hz = 1500.0f;
    smo.config.speed_lpf_hz = 100.0f;
    smo.config.emf_valid_v = 0.5f;
    smo.config.direction = 1;
    smo.config.theta_offset_rad = 0.0f;
    SMO_Init(&smo, &smo.config);
}
void motor_init(void)
{
    // 初始化电流 PID
    // Kp Ki 需要根据电机电感电阻实际整定
    // limit_out = 12.0f 假设母线电压12V，最大输出不能超过母线电压
    // limit_int = 10.0f; 积分限幅稍微比输出限幅小一点或相等
    PI_Init(&foc.pid_d, 0.3927f, 0.03608f, 13.0f, 13.0f);
    PI_Init(&foc.pid_q, 0.3927f, 0.03608f, 13.0f, 13.0f);
    PI_Init(&foc.pid_speed, 2.0f, 0.062f, 3.0f, 3.0f);

    /* --- ADC 零点校准 --- */
    // 确保此时三相 PWM 还没输出，或者占空比为 0，保证电机没有电流
    uint32_t offset_sum_u = 0;
    uint32_t offset_sum_w = 0;
    uint32_t offset_sum_v = 0;
    int sample_counts = 2000;

    // 先开启 ADC，确保它可以工作，但先不要开中断
    LL_ADC_DisableIT_JEOS(ADC1);
    LL_ADC_ClearFlag_JEOS(ADC1);
    LL_TIM_CC_EnableChannel(TIM1, LL_TIM_CHANNEL_CH4);
    LL_TIM_EnableCounter(TIM1);
    for (int i = 0; i < sample_counts; i++)
    {

        // 2. 等待采样完成（JEOS 标志位置 1）
        while (LL_ADC_IsActiveFlag_JEOS(ADC1) == 0)
            ;
        LL_ADC_ClearFlag_JEOS(ADC1); // 清除标志位

        offset_sum_u += LL_ADC_INJ_ReadConversionData12(ADC1, LL_ADC_INJ_RANK_1);
        offset_sum_w += LL_ADC_INJ_ReadConversionData12(ADC1, LL_ADC_INJ_RANK_2);
        offset_sum_v += LL_ADC_INJ_ReadConversionData12(ADC2, LL_ADC_INJ_RANK_1);
    }

    foc.adc.i_a_offset = offset_sum_u / sample_counts;
    foc.adc.i_c_offset = offset_sum_w / sample_counts;
    foc.adc.i_b_offset = offset_sum_v / sample_counts;
    LL_TIM_DisableCounter(TIM1);
    LL_TIM_DisableAllOutputs(TIM1);
    LL_TIM_CC_EnableChannel(TIM1,
                            LL_TIM_CHANNEL_CH1 | LL_TIM_CHANNEL_CH1N |
                                LL_TIM_CHANNEL_CH2 | LL_TIM_CHANNEL_CH2N |
                                LL_TIM_CHANNEL_CH3 | LL_TIM_CHANNEL_CH3N);
    TIM1->EGR |= TIM_EGR_COMG;
    LL_TIM_EnableCounter(TIM1);
    LL_TIM_EnableAllOutputs(TIM1);

#if USE_TIM_UPDATE_IT
    LL_TIM_DisableIT_UPDATE(TIM1);
    svpwm_modulation(0.7f, 0);
    LL_mDelay(1000);
    for (int i = 0; i < 4; i++)
        foc.angle_error = SPI1_read(0xffff) & 0x3fff;
    svpwm_modulation(0, 0);
    // LL_TIM_EnableIT_UPDATE(TIM1);
#endif

#if USE_ADC_INJECT_IT
    LL_ADC_DisableIT_JEOS(ADC1);
    LL_TIM_CC_DisableChannel(TIM1, LL_TIM_CHANNEL_CH4);

    svpwm_modulation(1.0f, 0.0f);
    LL_mDelay(1000);
    for (int i = 0; i < 4; i++)
        foc.angle_error = SPI1_read(0xffff) & 0x3fff;
    NFO_ResetAngle(&nfo, 0.0f, 0.0f, 0.0f);
    svpwm_modulation(0.0f, 0.0f);
    LL_ADC_ClearFlag_JEOS(ADC1);
    LL_ADC_ClearFlag_JEOS(ADC2);
    LL_TIM_CC_EnableChannel(TIM1, LL_TIM_CHANNEL_CH4);
    LL_ADC_EnableIT_JEOS(ADC1);
#endif
}
