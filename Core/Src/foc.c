#include "foc.h"
extern foc_state foc;
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
cordic_angle cor_q32(uint32_t theta_q32)
{
    cordic_angle res;

    int32_t angle_int = (int32_t)theta_q32;

    LL_CORDIC_WriteData(CORDIC, angle_int);

    /*
     * 当前 CORDIC 配置是 LL_CORDIC_FUNCTION_SINE，
     * 所以 NBREAD_2 下先读 sin，再读 cos。
     */
    res.sin_angle = ((float)((int32_t)LL_CORDIC_ReadData(CORDIC))) * (1.0f / 2147483648.0f);
    res.cos_angle = ((float)((int32_t)LL_CORDIC_ReadData(CORDIC))) * (1.0f / 2147483648.0f);

    return res;
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
    foc.angle_el = angle_el;
    foc.adc.i_a = ((float)LL_ADC_INJ_ReadConversionData12(ADC1, LL_ADC_INJ_RANK_1) - foc.adc.i_a_offset) * ADC_Coefficient;
    foc.adc.i_c = ((float)LL_ADC_INJ_ReadConversionData12(ADC1, LL_ADC_INJ_RANK_2) - foc.adc.i_c_offset) * ADC_Coefficient;
    foc.adc.i_b = ((float)LL_ADC_INJ_ReadConversionData12(ADC2, LL_ADC_INJ_RANK_2) - foc.adc.i_b_offset) * ADC_Coefficient;

    // FILTER(ADC_FILTER_ALPHA, ((float)LL_ADC_INJ_ReadConversionData12(ADC1, LL_ADC_INJ_RANK_1) - foc.adc.i_a_offset) * ADC_Coefficient, foc.adc.i_a);
    // FILTER(ADC_FILTER_ALPHA, ((float)LL_ADC_INJ_ReadConversionData12(ADC1, LL_ADC_INJ_RANK_2) - foc.adc.i_c_offset) * ADC_Coefficient, foc.adc.i_c);
    // FILTER(ADC_FILTER_ALPHA, ((float)LL_ADC_INJ_ReadConversionData12(ADC2, LL_ADC_INJ_RANK_2) - foc.adc.i_b_offset) * ADC_Coefficient, foc.adc.i_b);

    foc.real_clark_i = clark_trans(-foc.adc.i_c, -foc.adc.i_b, -foc.adc.i_a);
    park_frame curr_park_i = park_trans(foc.real_clark_i.alpha, foc.real_clark_i.beta, angle_el);

    // park_frame curr_park_i = park_trans(foc.real_clark_i.alpha, foc.real_clark_i.beta, angle_el);
    FILTER(0.9f, curr_park_i.d, foc.real_park_i.d);
    FILTER(0.9f, curr_park_i.q, foc.real_park_i.q);

    // PI 控制器计算目标电压
    float target_v_d = PI_Calc(&foc.pid_d, i_d, foc.real_park_i.d);
    float target_v_q = PI_Calc(&foc.pid_q, i_q, foc.real_park_i.q);

    foc.target_clark_v = park_reverse(target_v_q, target_v_d, angle_el);
    svpwm_modulation(foc.target_clark_v.alpha, foc.target_clark_v.beta);
}
