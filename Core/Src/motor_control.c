#include "motor_control.h"
#include "pid.h"
#include "spi.h"
#include "usart.h"
#include "oled.h"
foc_state foc;
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

// a相 b相 c相
clark_frame clark_trans(float a, float b, float c)
{
    clark_frame res;
    res.alpha = (2.0f * a - b - c) / 3.0f;
    res.beta = (b - c) / 1.7320508f;

    return res;
}

adc_curr clark_reverse(float alpha, float beta)
{
    adc_curr res;
    res.i_a = alpha;
    res.i_b = (beta * SQRT3 - alpha) / 2;
    res.i_c = -(alpha + beta * SQRT3) / 2;
    return res;
}

park_frame park_trans(float alpha, float beta, float angle_el)
{
    park_frame res;
    cordic_angle angle = cor(angle_el);
    res.d = alpha * angle.cos_angle + beta * angle.sin_angle;
    res.q = -alpha * angle.sin_angle + beta * angle.cos_angle;

    return res;
}

clark_frame park_reverse(float q, float d, float angle_el)
{
    clark_frame res;
    cordic_angle angle = cor(angle_el);
    res.alpha = d * angle.cos_angle - q * angle.sin_angle;
    res.beta = d * angle.sin_angle + q * angle.cos_angle;
    return res;
}

cordic_angle cor(float angle_rad)
{
    cordic_angle res;

    // 1. 将弧度 [0, 2PI] 映射到 [-PI, PI]
    // 这是 STM32 CORDIC q31 输入的标准要求
    if (angle_rad > PI)
    {
        angle_rad -= (2.0f * PI);
    }

    // 2. 映射到 q31 格式: [-1, 1) 对应 [-PI, PI)
    // 使用 2147483647.0f 防止正向溢出
    int32_t angle_int = (int32_t)((angle_rad / PI) * 2147483647.0f);

    // 3. 写入数据触发计算
    LL_CORDIC_WriteData(CORDIC, angle_int);

    // 4. 连续读取 Sine 和 Cosine
    // CORDIC 算出的 q31 结果直接除以 2^31 转回 [-1.0, 1.0] 的浮点数
    // 先强转为有符号的 int32_t，告诉编译器这是有正负的 q31，然后再转 float！
    res.sin_angle = ((float)((int32_t)LL_CORDIC_ReadData(CORDIC))) / 2147483648.0f;
    res.cos_angle = ((float)((int32_t)LL_CORDIC_ReadData(CORDIC))) / 2147483648.0f;

    return res;
}

float get_angle_el(void)
{
    // AS5047P

    foc.angle_machanine = SPI1_read(0xffff) & 0x3fff;
    float angle_el = ((float)(((foc.angle_machanine + 0x3fff - foc.angle_error) * POLE_PAIRS) & 0x3fff)) * 2 * PI / 16383.0f;

    return angle_el;
    // open loop
}

uint8_t sector_judge(float v_alpha, float v_beta)
{
    uint8_t a = 0, b = 0, c = 0, N;
    if (v_beta > 0.0f)
    {
        a = 1;
    }
    if ((0.8660254038f * v_alpha) - (v_beta / 2.0f) > 0.0f)
    {
        b = 1;
    }
    if (-(0.8660254038f * v_alpha) - (v_beta / 2.0f) > 0.0f)
    {
        c = 1;
    }
    N = 4 * c + 2 * b + a;
    return N;
}

T4T6_timeTypeDef T4T6_time(float v_alpha, float v_beta)
{
    T4T6_timeTypeDef res;

    // 1. 预计算常数（编译器会优化为固定数值）
    const float MAX_T = 2.0f * (float)TIM_PWM_ARR;
    const float K1 = (SQRT3 * MAX_T) / (float)V_BUS;
    const float K2 = K1 * 0.5f;

    // 2. 核心数学计算：去除了所有除法，全部转为单周期乘法
    float X = K1 * v_beta;
    float Y = K2 * (SQRT3 * v_alpha + v_beta);
    float Z = K2 * (-SQRT3 * v_alpha + v_beta);

    // 3. 扇区赋值
    switch (foc.svpwm.sector_N)
    {
    case 1:
        res.T4 = Z;
        res.T6 = Y;
        break;
    case 2:
        res.T4 = Y;
        res.T6 = -X;
        break;
    case 3:
        res.T4 = -Z;
        res.T6 = X;
        break;
    case 4:
        res.T4 = -X;
        res.T6 = Z;
        break;
    case 5:
        res.T4 = X;
        res.T6 = -Y;
        break;
    case 6:
        res.T4 = -Y;
        res.T6 = -Z;
        break;
    default:
        res.T4 = 0.0f;
        res.T6 = 0.0f;
        break;
    }

    // 4. 过调制限幅优化：提取比例系数，只做一次除法
    float sum = res.T4 + res.T6;
    if (sum > MAX_T)
    {
        float scale = MAX_T / sum; // 仅在此处消耗一次除法周期
        res.T4 *= scale;
        res.T6 *= scale;
    }

    return res;
}

void svpwm_modulation(float v_alpha, float v_beta)
{
    foc.svpwm.sector_N = sector_judge(v_alpha, v_beta);
    foc.svpwm.T4T6 = T4T6_time(v_alpha, v_beta);
    foc.svpwm.tabc_time.ta = (TIM_PWM_ARR - ((uint32_t)foc.svpwm.T4T6.T4) / 2 - ((uint32_t)foc.svpwm.T4T6.T6) / 2) / 2;
    foc.svpwm.tabc_time.tb = foc.svpwm.tabc_time.ta + ((uint32_t)foc.svpwm.T4T6.T4) / 2;
    foc.svpwm.tabc_time.tc = foc.svpwm.tabc_time.tb + ((uint32_t)foc.svpwm.T4T6.T6) / 2;
    switch (foc.svpwm.sector_N)
    {
    case 1:
        foc.svpwm.change_tick.tcm1 = foc.svpwm.tabc_time.tb;
        foc.svpwm.change_tick.tcm2 = foc.svpwm.tabc_time.ta;
        foc.svpwm.change_tick.tcm3 = foc.svpwm.tabc_time.tc;
        break;
    case 2:
        foc.svpwm.change_tick.tcm1 = foc.svpwm.tabc_time.ta;
        foc.svpwm.change_tick.tcm2 = foc.svpwm.tabc_time.tc;
        foc.svpwm.change_tick.tcm3 = foc.svpwm.tabc_time.tb;
        break;
    case 3:
        foc.svpwm.change_tick.tcm1 = foc.svpwm.tabc_time.ta;
        foc.svpwm.change_tick.tcm2 = foc.svpwm.tabc_time.tb;
        foc.svpwm.change_tick.tcm3 = foc.svpwm.tabc_time.tc;
        break;
    case 4:
        foc.svpwm.change_tick.tcm1 = foc.svpwm.tabc_time.tc;
        foc.svpwm.change_tick.tcm2 = foc.svpwm.tabc_time.tb;
        foc.svpwm.change_tick.tcm3 = foc.svpwm.tabc_time.ta;
        break;
    case 5:
        foc.svpwm.change_tick.tcm1 = foc.svpwm.tabc_time.tc;
        foc.svpwm.change_tick.tcm2 = foc.svpwm.tabc_time.ta;
        foc.svpwm.change_tick.tcm3 = foc.svpwm.tabc_time.tb;
        break;
    case 6:
        foc.svpwm.change_tick.tcm1 = foc.svpwm.tabc_time.tb;
        foc.svpwm.change_tick.tcm2 = foc.svpwm.tabc_time.tc;
        foc.svpwm.change_tick.tcm3 = foc.svpwm.tabc_time.ta;
        break;
    default:
        foc.svpwm.change_tick.tcm1 = TIM_PWM_ARR;
        foc.svpwm.change_tick.tcm2 = TIM_PWM_ARR;
        foc.svpwm.change_tick.tcm3 = TIM_PWM_ARR;
        break;
    }
    TIM1->CCR1 = foc.svpwm.change_tick.tcm1;
    TIM1->CCR2 = foc.svpwm.change_tick.tcm2;
    TIM1->CCR3 = foc.svpwm.change_tick.tcm3;
}

void foc_current_loop_control(float i_d, float i_q)
{

    // 1. 获取真实电角度（保持磁场定向跟随转子位置）
    float angle_el = get_angle_el();

    // foc.adc.i_a = (1 - ADC_FILTER_ALPHA) * foc.adc.i_a + ADC_FILTER_ALPHA * ((float)LL_ADC_INJ_ReadConversionData12(ADC1, LL_ADC_INJ_RANK_1) - foc.adc.i_a_offset) * ADC_Coefficient;
    // foc.adc.i_c = (1 - ADC_FILTER_ALPHA) * foc.adc.i_c + ADC_FILTER_ALPHA * ((float)LL_ADC_INJ_ReadConversionData12(ADC1, LL_ADC_INJ_RANK_2) - foc.adc.i_c_offset) * ADC_Coefficient;
    // foc.adc.i_b = (1 - ADC_FILTER_ALPHA) * foc.adc.i_b + ADC_FILTER_ALPHA * ((float)LL_ADC_INJ_ReadConversionData12(ADC2, LL_ADC_INJ_RANK_2) - foc.adc.i_b_offset) * ADC_Coefficient;

    FILTER(ADC_FILTER_ALPHA, ((float)LL_ADC_INJ_ReadConversionData12(ADC1, LL_ADC_INJ_RANK_1) - foc.adc.i_a_offset) * ADC_Coefficient, foc.adc.i_a);
    FILTER(ADC_FILTER_ALPHA, ((float)LL_ADC_INJ_ReadConversionData12(ADC1, LL_ADC_INJ_RANK_2) - foc.adc.i_c_offset) * ADC_Coefficient, foc.adc.i_c);
    FILTER(ADC_FILTER_ALPHA, ((float)LL_ADC_INJ_ReadConversionData12(ADC2, LL_ADC_INJ_RANK_2) - foc.adc.i_b_offset) * ADC_Coefficient, foc.adc.i_b);

    foc.real_clark_i = clark_trans(-foc.adc.i_c, -foc.adc.i_b, -foc.adc.i_a);
    foc.real_park_i = park_trans(foc.real_clark_i.alpha, foc.real_clark_i.beta, angle_el);

    // park_frame curr_park_i = park_trans(foc.real_clark_i.alpha, foc.real_clark_i.beta, angle_el);
    // FILTER(0.9f, curr_park_i.d, foc.real_park_i.d);
    // FILTER(0.9f, curr_park_i.q, foc.real_park_i.q);

    // PI 控制器计算目标电压
    float target_v_d = PI_Calc(&foc.pid_d, i_d, foc.real_park_i.d);
    float target_v_q = PI_Calc(&foc.pid_q, i_q, foc.real_park_i.q);

    foc.target_clark_v = park_reverse(target_v_q, target_v_d, angle_el);
    svpwm_modulation(foc.target_clark_v.alpha, foc.target_clark_v.beta);
}

void motor_init(void)
{
    // 初始化电流 PID
    // Kp Ki 需要根据电机电感电阻实际整定
    // limit_out = 12.0f 假设母线电压12V，最大输出不能超过母线电压
    // limit_int = 10.0f; 积分限幅稍微比输出限幅小一点或相等
    PI_Init(&foc.pid_d, 10.0f, 1e-2f, 6.0f, 6.0f);
    PI_Init(&foc.pid_q, 10.0f, 1e-2f, 6.0f, 6.0f);
    PI_Init(&foc.pid_speed, 1e-3f, 1e-2f, 0.5f, 0.5f);

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
    svpwm_modulation(0.0f, 0.0f);

    LL_ADC_ClearFlag_JEOS(ADC1);
    LL_ADC_ClearFlag_JEOS(ADC2);
    LL_TIM_CC_EnableChannel(TIM1, LL_TIM_CHANNEL_CH4);
    LL_ADC_EnableIT_JEOS(ADC1);
#endif
}
