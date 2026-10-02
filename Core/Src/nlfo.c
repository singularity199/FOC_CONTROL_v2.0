// clang-format off
/**
 ******************************************************************************
 * @file    nlfo.c
 * @brief   NLFO（NonLinear Flux Observer，非线性磁链观测器）实现。
 *
 *          观测方程（αβ 静止坐标系）:
 *              λ̂̇   = v − Rs·i − ℓ·(ρ̂ − ψf)·η̂          （磁链积分 + 非线性径向修正）
 *              ψ̂   = λ̂ − Ls·i                          （转子磁链）
 *              ρ̂   = |ψ̂|,  η̂ = ψ̂/ρ̂
 *              θ̂   = PLL( atan2(ψ̂β, ψ̂α) )             （角度 + 速度锁相）
 *
 *          本文件为【新增独立模块】，不修改任何现有源码，不依赖数学库：
 *          sin/cos 与 atan2 为自实现单精度多项式（float32 下最大误差分别
 *          < 2e-7 rad 与 < 2e-6 rad），sqrt 为牛顿迭代（相对误差 < 1e-7）。
 *
 ******************************************************************************
 */

#include "nlfo.h"

/* 自采集模式需要读取 ADC 注入结果与 TIM1 比较寄存器 */
#include "main.h"

#include <string.h>

#define NLFO_PI         3.14159265358979323846f
#define NLFO_TWO_PI     6.28318530717958647692f
#define NLFO_HALF_PI    1.57079632679489661923f
#define NLFO_SQRT3      1.73205080756887729353f
#define NLFO_RHO_MIN    1e-9f  /* 磁链幅值下限，低于此值视为未激励 */
#define NLFO_CONV_RATIO 0.2f   /* 收敛判定: ρ̂ > 0.2·ψf */

/* ============================ 数学工具（自实现） ============================ */

/** @brief 角度归一到 [−π, π] */
static float nlfo_wrap_pi(float a)
{
    while (a > NLFO_PI)
    {
        a -= NLFO_TWO_PI;
    }
    while (a < -NLFO_PI)
    {
        a += NLFO_TWO_PI;
    }
    return a;
}

/** @brief 限幅 */
static float nlfo_clampf(float x, float lo, float hi)
{
    if (x < lo)
    {
        return lo;
    }
    if (x > hi)
    {
        return hi;
    }
    return x;
}

/**
 * @brief  快速 sqrtf：IEEE754 指数折半猜值 + 3 次牛顿迭代。
 *         float32 下相对误差 < 1e-7（x > 0）。
 */
static float nlfo_sqrtf(float x)
{
    union
    {
        float f;
        uint32_t u;
    } conv;

    conv.f = x;
    /* 指数域折半: (i >> 1) + (0x3F800000 >> 1)，得到 ±6% 以内的初值 */
    conv.u = (conv.u >> 1) + 0x1FC00000u;
    float y = conv.f;
    y = 0.5f * (y + x / y);
    y = 0.5f * (y + x / y);
    y = 0.5f * (y + x / y);
    return y;
}

/**
 * @brief  自实现 atan2(y, x)，返回 [−π, π]。
 *         对 r ∈ [0,1] 使用 atan(r) ≈ r·P(r²) 的 7 阶极小化多项式
 *         （float32 实测最大误差 1.8e-6 rad），配合八分圆折叠。
 */
static float nlfo_atan2(float y, float x)
{
    const float c0 = 0.9999994908f;
    const float c1 = -0.3332770742f;
    const float c2 = 0.1989716105f;
    const float c3 = -0.1356127725f;
    const float c4 = 0.08543323111f;
    const float c5 = -0.0385157449f;
    const float c6 = 0.008401147275f;

    float ax = x < 0.0f ? -x : x;
    float ay = y < 0.0f ? -y : y;
    float r;
    float a;

    if (ax >= ay)
    {
        r = ay / ax; /* 第一八分圆: a ∈ [0, π/4] */
    }
    else
    {
        r = ax / ay; /* 其余: a ∈ (π/4, π/2]，用 π/2 − atan(r) */
    }

    float u = r * r;
    a = c0 + u * (c1 + u * (c2 + u * (c3 + u * (c4 + u * (c5 + u * c6)))));
    a *= r;

    if (ax < ay)
    {
        a = NLFO_HALF_PI - a;
    }
    if (x < 0.0f)
    {
        a = NLFO_PI - a;
    }
    if (y < 0.0f)
    {
        a = -a;
    }
    return a;
}

/**
 * @brief  自实现 sin/cos（输入 x ∈ [−π, π]）。
 *         sin(x) ≈ x·P(x²)，P 为 5 阶极小化多项式（float32 实测最大误差
 *         1.3e-7），先折叠到 [0, π/2] 再做象限符号处理。
 */
static void nlfo_sincos(float x, float *s, float *c)
{
    const float p0 = 0.999999997f;
    const float p1 = -0.1666665996f;
    const float p2 = 0.008333097227f;
    const float p3 = -0.0001981245976f;
    const float p4 = 2.612840168e-06f;

    float s_sign = 1.0f; /* sin 符号 */
    float c_sign = 1.0f; /* cos 符号 */

    if (x < 0.0f)
    {
        x = -x;
        s_sign = -1.0f; /* sin(−x) = −sin(x), cos(−x) = cos(x) */
    }
    if (x > NLFO_HALF_PI)
    {
        x = NLFO_PI - x;
        c_sign = -1.0f; /* sin(π−x) = sin(x), cos(π−x) = −cos(x) */
    }

    float u = x * x;
    float poly = p0 + u * (p1 + u * (p2 + u * (p3 + u * p4)));
    float sin_x = poly * x;

    float xc = NLFO_HALF_PI - x; /* cos(x) = sin(π/2 − x) */
    float uc = xc * xc;
    float polyc = p0 + uc * (p1 + uc * (p2 + uc * (p3 + uc * p4)));

    *s = s_sign * sin_x;
    *c = c_sign * (polyc * xc);
}

/* ============================ 初始化 / 复位 ============================ */

void NLFO_Init(NLFO_Handle_t *h, float Rs, float Ls, float flux_pm, float Ts)
{
    memset(h, 0, sizeof(*h));

    h->Rs = Rs;
    h->Ls = Ls;
    h->flux_pm = flux_pm;
    h->Ts = Ts;

    /* 默认观测器参数 */
    h->gain = 200.0f;      /* 磁链幅值收敛时间常数 ≈ 1/ℓ = 5 ms */
    h->pll_kp = 1000.0f;   /* ωn = √ki ≈ 775 rad/s (123 Hz), ζ = kp/(2ωn) ≈ 0.65 */
    h->pll_ki = 600000.0f;
    h->omega_max = 2000.0f;
    h->omega_lpf = 0.1f;
    h->omega_min = 30.0f;

    /* 自采集默认值（与 main.h 宏一致，零点仍需用 NLFO_ConfigSampler 单独配置） */
    h->adc_gain = ADC_Coefficient;
    h->v_bus = V_BUS;
    h->pwm_arr = (float)TIM_PWM_ARR;
}

void NLFO_Reset(NLFO_Handle_t *h, float theta0_el)
{
    float theta = nlfo_wrap_pi(theta0_el);
    float sin_t;
    float cos_t;

    nlfo_sincos(theta, &sin_t, &cos_t);

    /* λ̂ = Ls·i + ψf·η(θ0)，用最近一次采样电流，静止时即 ψf·η(θ0) */
    h->lambda_alpha = h->Ls * h->i_alpha + h->flux_pm * cos_t;
    h->lambda_beta = h->Ls * h->i_beta + h->flux_pm * sin_t;
    h->psi_alpha = h->flux_pm * cos_t;
    h->psi_beta = h->flux_pm * sin_t;
    h->rho = h->flux_pm;
    h->theta_raw = theta;
    h->theta_el = theta;
    h->omega_el = 0.0f;
    h->omega_el_f = 0.0f;
    h->pll_err = 0.0f;
    h->pll_int = 0.0f;
    h->converged = 0u;
}

/* ============================ 周期调用 ============================ */

void NLFO_Update(NLFO_Handle_t *h, float v_alpha, float v_beta, float i_alpha, float i_beta)
{
    const float Ts = h->Ts;

    h->v_alpha = v_alpha;
    h->v_beta = v_beta;
    h->i_alpha = i_alpha;
    h->i_beta = i_beta;
    h->tick++;

    /* 1. 磁链积分: λ̂̇ = v − Rs·i */
    h->lambda_alpha += Ts * (v_alpha - h->Rs * i_alpha);
    h->lambda_beta += Ts * (v_beta - h->Rs * i_beta);

    /* 2. 转子磁链: ψ̂ = λ̂ − Ls·i */
    float psi_a = h->lambda_alpha - h->Ls * i_alpha;
    float psi_b = h->lambda_beta - h->Ls * i_beta;
    float rho = nlfo_sqrtf(psi_a * psi_a + psi_b * psi_b);
    h->rho = rho;

    if (rho < NLFO_RHO_MIN)
    {
        /* 磁链幅值过小（静止或未激励），冻结角度输出 */
        h->pll_err = 0.0f;
        h->omega_el = 0.0f;
        h->omega_el_f = 0.0f;
        h->converged = 0u;
        return;
    }

    /* 3. 非线性磁链反馈（径向修正）: λ̂̇ += −ℓ·(ρ̂ − ψf)·η̂, η̂ = ψ̂/ρ̂
     *    把 |ψ̂| 拉向 ψf，抑制纯积分漂移，保证全局收敛 */
    float corr = Ts * h->gain * (rho - h->flux_pm) / rho;
    h->lambda_alpha -= corr * psi_a;
    h->lambda_beta -= corr * psi_b;

    /* 修正后重算转子磁链，保证存储状态自洽 */
    psi_a = h->lambda_alpha - h->Ls * i_alpha;
    psi_b = h->lambda_beta - h->Ls * i_beta;
    rho = nlfo_sqrtf(psi_a * psi_a + psi_b * psi_b);
    h->psi_alpha = psi_a;
    h->psi_beta = psi_b;
    h->rho = rho;

    /* 4. 原始角度 + PLL 锁相 */
    h->theta_raw = nlfo_atan2(psi_b, psi_a);

    float sin_t;
    float cos_t;
    nlfo_sincos(h->theta_el, &sin_t, &cos_t);

    /* 相位误差 = sin(θraw − θpll) = (ψβ·cosθpll − ψα·sinθpll)/ρ */
    float err = (psi_b * cos_t - psi_a * sin_t) / rho;
    err = nlfo_clampf(err, -2.0f, 2.0f);
    h->pll_err = err;

    h->pll_int += Ts * h->pll_ki * err;
    h->pll_int = nlfo_clampf(h->pll_int, -h->omega_max, h->omega_max);
    float omega = h->pll_int + h->pll_kp * err;
    omega = nlfo_clampf(omega, -h->omega_max, h->omega_max);
    h->omega_el = omega;

    h->theta_el = nlfo_wrap_pi(h->theta_el + Ts * omega);
    h->omega_el_f += h->omega_lpf * (omega - h->omega_el_f);

    /* 5. 收敛判定 */
    float omega_abs = omega < 0.0f ? -omega : omega;
    h->converged = (rho > NLFO_CONV_RATIO * h->flux_pm && omega_abs > h->omega_min) ? 1u : 0u;
}

void NLFO_UpdateInputs(NLFO_Handle_t *h, const NLFO_Input_t *in)
{
    NLFO_Update(h, in->v_alpha, in->v_beta, in->i_alpha, in->i_beta);
}

/* ============================ 参数设置 ============================ */

void NLFO_SetGain(NLFO_Handle_t *h, float gain)
{
    h->gain = gain;
}

void NLFO_SetPll(NLFO_Handle_t *h, float kp, float ki, float omega_max)
{
    h->pll_kp = kp;
    h->pll_ki = ki;
    h->omega_max = omega_max;
}

void NLFO_SetOmegaFilter(NLFO_Handle_t *h, float lpf_alpha, float omega_min)
{
    h->omega_lpf = lpf_alpha;
    h->omega_min = omega_min;
}

void NLFO_ConfigSampler(NLFO_Handle_t *h, uint32_t off_a, uint32_t off_b, uint32_t off_c,
                        float adc_gain, float v_bus, float pwm_arr)
{
    h->adc_off_a = off_a;
    h->adc_off_b = off_b;
    h->adc_off_c = off_c;
    h->adc_gain = adc_gain;
    h->v_bus = v_bus;
    h->pwm_arr = pwm_arr;
}

/* ============================ 自采集 ============================ */

void NLFO_SampleInputs(NLFO_Handle_t *h, NLFO_Input_t *in)
{
    /* 电流: ADC 通道映射与 foc_current_loop_control 完全一致
     *   i_a = ADC1 注入 Rank1, i_c = ADC1 注入 Rank2, i_b = ADC2 注入 Rank2 */
    float i_a = ((float)LL_ADC_INJ_ReadConversionData12(ADC1, LL_ADC_INJ_RANK_1) - (float)h->adc_off_a) * h->adc_gain;
    float i_c = ((float)LL_ADC_INJ_ReadConversionData12(ADC1, LL_ADC_INJ_RANK_2) - (float)h->adc_off_c) * h->adc_gain;
    float i_b = ((float)LL_ADC_INJ_ReadConversionData12(ADC2, LL_ADC_INJ_RANK_2) - (float)h->adc_off_b) * h->adc_gain;

    /* αβ 变换与现有 Clarke 定义一致: clark_trans(−i_c, −i_b, −i_a) */
    in->i_alpha = (i_a + i_b - 2.0f * i_c) / 3.0f;
    in->i_beta = (i_a - i_b) / NLFO_SQRT3;

    /* 电压: 由 TIM1 三相比较值按本项目 SVPWM 公式精确反解。
     * 对 svpwm_modulation 的 6 个扇区做最小二乘反解得到（残差 < 1e-13，
     * 即线性调制区内的精确逆映射；含过调制时给出实际输出伏秒）:
     *   v_alpha = V_BUS/(6·ARR)·(CCR2 + CCR3 − 2·CCR1)
     *   v_beta  = V_BUS/(√3·ARR)·(CCR3 − CCR2)
     * 常数项(ARR/2)因系数和为零自动消去，无需占空比极性假设。 */
    float c1 = (float)TIM1->CCR1;
    float c2 = (float)TIM1->CCR2;
    float c3 = (float)TIM1->CCR3;

    in->v_alpha = (h->v_bus / (6.0f * h->pwm_arr)) * (c2 + c3 - 2.0f * c1);
    in->v_beta = (h->v_bus / (NLFO_SQRT3 * h->pwm_arr)) * (c3 - c2);
}

/* ============================ 结果读取 ============================ */

float NLFO_GetThetaEl(const NLFO_Handle_t *h)
{
    return h->theta_el;
}

float NLFO_GetOmegaEl(const NLFO_Handle_t *h)
{
    return h->omega_el_f;
}

float NLFO_GetRho(const NLFO_Handle_t *h)
{
    return h->rho;
}

float NLFO_GetAngleError(const NLFO_Handle_t *h, float ref_theta_el)
{
    return nlfo_wrap_pi(h->theta_el - ref_theta_el);
}

uint32_t NLFO_IsConverged(const NLFO_Handle_t *h)
{
    return h->converged;
}
