// clang-format off
/**
 ******************************************************************************
 * @file    motor_identify.c
 * @brief   电机参数自辨识模块实现（Rs 电阻 / Ls 电感 / ψf 磁链）。
 *
 *          【电阻 Rs】多电平直流注入: 施加 v_alpha = V（多级），稳态电流
 *                    i_alpha = (V − V_off)/R，对 (V, I) 做最小二乘得到 R，
 *                    死区/器件压降等常数偏移 V_off 自动剔除。
 *
 *          【电感 Ls】方波电压 ±V 注入，稳态电流峰峰值满足分段指数模型
 *                    （含死区/压降偏移 v_off = v_offset，即电流反向时电压
 *                    误差 ±v_off）:
 *                      i_p = (K−1)(V²−v²)/(R·(V−v+K(V+v))), K = e^(h_eff/τ)
 *                    反解 τ = h_eff/ln(K) 即得 Ls = τ·R。h_eff 为实际施加的
 *                    半周期（含 PWM 预装载延迟，见函数内注释）。
 *                    转子所受力矩为注入频率交变、平均为零，不会转动。
 *
 *          【磁链 ψf】在线运行中按 dq 电压方程
 *                    v_q = Rs·i_q + Ls·di_q/dt + ωe·ψf
 *                    对 ψf 做最小二乘（恒速、轻载段最准，内部降采样到 ~1kHz）。
 *
 *          本文件为【新增独立模块】，不修改任何现有源码：
 *          - 阻感辨识期间临时关闭 ADC1 JEOS 中断（结束自动恢复），
 *            复用现有 svpwm_modulation()（motor_control.c 中定义）输出测试电压，
 *            电流采样通道与 foc_current_loop_control 完全一致。
 *          - 磁链辨识只读取 foc 全局变量，不写任何外设。
 ******************************************************************************
 */

#include "motor_identify.h"

#include "main.h"           /* LL ADC/TIM 寄存器访问、V_BUS/ADC_Coefficient/POLE_PAIRS/SPEED_LOOP_TS */
#include "motor_control.h"  /* foc 全局变量（FluxUpdateFromFoc） */

#include <string.h>

/* svpwm_modulation 在 motor_control.c 中定义但未在其头文件导出；
   此处仅声明复用，不改动任何现有文件 */
extern void svpwm_modulation(float v_alpha, float v_beta);

#define MOTORID_SQRT3         1.73205080756887729353f
#define MOTORID_TWO_PI        6.28318530717958647692f
#define MOTORID_LN2           0.69314718055994530942f
#define MOTORID_FS_FALLBACK   19318.0f /* TIM1 170MHz/(20·2·220) 的中心对齐频率 */
#define MOTORID_FLUX_RENORM_SAMPLES 65536u
#define MOTORID_MIN_SAMPLES_AVG 8u

/* ============================ 数学工具（自实现） ============================ */

/**
 * @brief  ln(x), x > 0：归约到 m ∈ [1,2)，ln(x) = ln(m) + e·ln2。
 *         多项式为 [1,2] 上 7 阶拟合（float32 实测最大误差 < 4e-6）。
 */
static float motorid_ln(float x)
{
    union
    {
        float f;
        uint32_t u;
    } conv;

    const float c0 = 3.473009368e-06f;
    const float c1 = 0.9997931695f;
    const float c2 = -0.4969826976f;
    const float c3 = 0.3146040977f;
    const float c4 = -0.1888011021f;
    const float c5 = 0.081738391f;
    const float c6 = -0.01721066802f;

    conv.f = x;
    int32_t e = (int32_t)((conv.u >> 23) & 0xFFu) - 127;
    conv.u = (conv.u & 0x007FFFFFu) | (127u << 23);
    float m = conv.f - 1.0f; /* m ∈ [0, 1) */
    float p = c0 + m * (c1 + m * (c2 + m * (c3 + m * (c4 + m * (c5 + m * c6)))));
    return p + (float)e * MOTORID_LN2;
}

/* ============================ 采样底层 ============================ */

/**
 * @brief  等待一次 ADC 注入组转换完成并读取三相电流 → αβ。
 *         通道映射与 foc_current_loop_control 一致：
 *         i_a = ADC1 注入 Rank1, i_c = ADC1 注入 Rank2, i_b = ADC2 注入 Rank2
 *         αβ 与现有 Clarke 定义一致：i_alpha = (i_a+i_b−2·i_c)/3, i_beta = (i_a−i_b)/√3
 * @return 1 成功；0 ADC 无触发（超时）
 */
static int motorid_sample(MotorID_Handle_t *h, float *i_alpha, float *i_beta)
{
    uint32_t timeout = 200000u;
    while (LL_ADC_IsActiveFlag_JEOS(ADC1) == 0u)
    {
        if (--timeout == 0u)
        {
            return 0;
        }
    }
    LL_ADC_ClearFlag_JEOS(ADC1);

    float i_a = ((float)LL_ADC_INJ_ReadConversionData12(ADC1, LL_ADC_INJ_RANK_1) - (float)h->adc_off_a) * h->adc_gain;
    float i_c = ((float)LL_ADC_INJ_ReadConversionData12(ADC1, LL_ADC_INJ_RANK_2) - (float)h->adc_off_c) * h->adc_gain;
    float i_b = ((float)LL_ADC_INJ_ReadConversionData12(ADC2, LL_ADC_INJ_RANK_2) - (float)h->adc_off_b) * h->adc_gain;

    *i_alpha = (i_a + i_b - 2.0f * i_c) / 3.0f;
    *i_beta = (i_a - i_b) / MOTORID_SQRT3;
    return 1;
}

/**
 * @brief  采集 n 个样本并平均 i_alpha，同时检测过流。
 * @param  over 输出：1 表示出现过 |i_alpha| > i_limit
 * @return 1 成功；0 ADC 超时
 */
static int motorid_probe(MotorID_Handle_t *h, uint32_t n, float *out, int *over)
{
    if (n < MOTORID_MIN_SAMPLES_AVG)
    {
        n = MOTORID_MIN_SAMPLES_AVG;
    }
    float sum = 0.0f;
    uint32_t cnt = 0u;
    *over = 0;
    for (uint32_t i = 0u; i < n; i++)
    {
        float ia;
        float ib;
        if (!motorid_sample(h, &ia, &ib))
        {
            return 0;
        }
        if (ia > h->i_limit || ia < -h->i_limit)
        {
            *over = 1;
        }
        sum += ia;
        cnt++;
    }
    *out = sum / (float)cnt;
    return 1;
}

/**
 * @brief  实测 ADC 注入触发频率（64 个周期计 CYCCNT）。
 *         DWT 未初始化或 TIM1 未运行时回退默认值 19318 Hz。
 */
static float motorid_measure_fs(void)
{
    const uint32_t n = 64u;
    uint32_t t0;
    uint32_t t1;

    for (uint32_t i = 0u; i < 4u; i++) /* 丢弃前几个周期等稳定 */
    {
        uint32_t timeout = 200000u;
        while (LL_ADC_IsActiveFlag_JEOS(ADC1) == 0u)
        {
            if (--timeout == 0u)
            {
                return MOTORID_FS_FALLBACK;
            }
        }
        LL_ADC_ClearFlag_JEOS(ADC1);
    }

    t0 = DWT->CYCCNT;
    for (uint32_t i = 0u; i < n; i++)
    {
        uint32_t timeout = 200000u;
        while (LL_ADC_IsActiveFlag_JEOS(ADC1) == 0u)
        {
            if (--timeout == 0u)
            {
                return MOTORID_FS_FALLBACK;
            }
        }
        LL_ADC_ClearFlag_JEOS(ADC1);
    }
    t1 = DWT->CYCCNT;

    if (t1 <= t0)
    {
        return MOTORID_FS_FALLBACK; /* DWT 未初始化 */
    }
    float fs = (float)n * 170000000.0f / (float)(t1 - t0);
    if (fs < 500.0f || fs > 200000.0f)
    {
        return MOTORID_FS_FALLBACK;
    }
    return fs;
}

/* ============================ 初始化 / 配置 ============================ */

void MotorID_Init(MotorID_Handle_t *h)
{
    memset(h, 0, sizeof(*h));

    h->v_dc_start = 1.0f;
    h->settle_first_ms = 400.0f;
    h->settle_ms = 100.0f;
    h->avg_samples = 2000u;
    h->i_limit = 2.0f;

    h->v_induct = 3.0f;
    h->f_induct = 1000.0f;
    h->cycles = 100u;
    h->settle_cycles = 100u;

    h->omega_min = 100.0f;
    h->v_q_max = 15.0f;

    h->adc_gain = ADC_Coefficient;
    h->v_bus = V_BUS;

    h->fs = MOTORID_FS_FALLBACK;
}

void MotorID_ConfigAdc(MotorID_Handle_t *h, uint32_t off_a, uint32_t off_b, uint32_t off_c,
                       float adc_gain, float v_bus)
{
    h->adc_off_a = off_a;
    h->adc_off_b = off_b;
    h->adc_off_c = off_c;
    h->adc_gain = adc_gain;
    h->v_bus = v_bus;
}

void MotorID_SetParams(MotorID_Handle_t *h, float Rs, float Ls)
{
    h->Rs = Rs;
    h->Ls = Ls;
}

/* ============================ 阻感辨识（阻塞式） ============================ */

MotorID_Status_t MotorID_Begin(MotorID_Handle_t *h)
{
    if (h->busy != 0u)
    {
        return MOTORID_ERR_BUSY;
    }

    /* 保存并关闭电流环中断（阻感辨识期间接管 PWM 输出） */
    h->it_was_enabled = LL_ADC_IsEnabledIT_JEOS(ADC1);
    LL_ADC_DisableIT_JEOS(ADC1);
    LL_ADC_ClearFlag_JEOS(ADC1);

    /* 确保 TIM1 输出与 ADC 注入触发就绪（与 motor_init 尾部一致，幂等） */
    LL_TIM_CC_EnableChannel(TIM1, LL_TIM_CHANNEL_CH4);
    LL_TIM_CC_EnableChannel(TIM1,
                            LL_TIM_CHANNEL_CH1 | LL_TIM_CHANNEL_CH1N |
                                LL_TIM_CHANNEL_CH2 | LL_TIM_CHANNEL_CH2N |
                                LL_TIM_CHANNEL_CH3 | LL_TIM_CHANNEL_CH3N);
    LL_TIM_EnableAllOutputs(TIM1);
    LL_TIM_EnableCounter(TIM1);
    LL_ADC_INJ_StartConversion(ADC1);

    h->fs = motorid_measure_fs();

    svpwm_modulation(0.0f, 0.0f); /* 零电压状态 */
    h->busy = 1u;
    return MOTORID_OK;
}

MotorID_Status_t MotorID_IdentifyResistance(MotorID_Handle_t *h)
{
    if (h->busy == 0u)
    {
        return MOTORID_ERR_BUSY;
    }

    /* 1. 自适应确定起始电压: 使稳态电流落在可测范围 [0.15A, i_limit] */
    float V = h->v_dc_start;
    float I = 0.0f;
    int over = 0;
    int ok = 0;
    for (int attempt = 0; attempt < 6; attempt++)
    {
        svpwm_modulation(V, 0.0f);
        float settle = (attempt == 0) ? h->settle_first_ms : h->settle_ms;
        uint32_t settle_n = (uint32_t)(settle * h->fs / 1000.0f);
        if (!motorid_probe(h, settle_n, &I, &over))
        {
            return MOTORID_ERR_ADC_TIMEOUT;
        }
        if (!motorid_probe(h, 500u, &I, &over)) /* 快速探测 */
        {
            return MOTORID_ERR_ADC_TIMEOUT;
        }
        if (over || I > h->i_limit)
        {
            /* 电流过大: 按比例降压重试 */
            float ref = I > 0.05f ? I : 0.05f;
            V *= (0.8f * h->i_limit) / ref;
            if (V < 0.05f)
            {
                V = 0.05f;
            }
            continue;
        }
        if (I < 0.15f)
        {
            /* 电流太小: 升压重试 */
            V *= 2.0f;
            float v_max = h->v_bus / 3.0f;
            if (V > v_max)
            {
                V = v_max;
            }
            continue;
        }
        ok = 1;
        break;
    }
    if (!ok)
    {
        svpwm_modulation(0.0f, 0.0f);
        return MOTORID_ERR_CURRENT_TOO_SMALL;
    }

    /* 2. 电流细化到约 0.4·i_limit（保证后续多点在限幅内且间隔足够） */
    {
        float v_max = h->v_bus / 3.0f;
        float target = 0.4f * h->i_limit;
        float Vr = V * (target / I);
        if (Vr > 0.85f * v_max)
        {
            Vr = 0.85f * v_max;
        }
        if (Vr < 0.05f)
        {
            Vr = 0.05f;
        }
        if (Vr > 1.1f * V || Vr < 0.9f * V)
        {
            svpwm_modulation(Vr, 0.0f);
            uint32_t settle_n = (uint32_t)(h->settle_ms * h->fs / 1000.0f);
            if (!motorid_probe(h, settle_n, &I, &over))
            {
                return MOTORID_ERR_ADC_TIMEOUT;
            }
            if (!motorid_probe(h, 500u, &I, &over))
            {
                return MOTORID_ERR_ADC_TIMEOUT;
            }
            if (!over && I > 0.15f && I < h->i_limit)
            {
                V = Vr;
            }
        }
    }

    /* 3. 多点测量: (V, 1.4V, 1.8V)，超限/电压间隔不足时自动跳过，最小二乘拟合
     *    I = (V − V_off)/R（至少需要 2 个有效点） */
    float Vs[3];
    float Is[3];
    int n = 0;
    for (int k = 0; k < 3; k++)
    {
        float v_max = h->v_bus / 3.0f;
        float Vk = V * (1.0f + 0.4f * (float)k);
        if (Vk > v_max)
        {
            Vk = v_max;
        }
        if (n > 0 && (Vk - Vs[n - 1]) < 0.05f * V)
        {
            /* 与上一点电压太近: 尝试取剩余电压区间中点，仍不足则放弃该点 */
            Vk = Vs[n - 1] + (v_max - Vs[n - 1]) * 0.5f;
            if ((Vk - Vs[n - 1]) < 0.05f * V)
            {
                continue;
            }
        }
        svpwm_modulation(Vk, 0.0f);
        uint32_t settle_n = (uint32_t)(h->settle_ms * h->fs / 1000.0f);
        if (!motorid_probe(h, settle_n, &I, &over))
        {
            return MOTORID_ERR_ADC_TIMEOUT;
        }
        if (!motorid_probe(h, h->avg_samples, &I, &over))
        {
            return MOTORID_ERR_ADC_TIMEOUT;
        }
        if (over || I > h->i_limit)
        {
            continue; /* 跳过超限点 */
        }
        Vs[n] = Vk;
        Is[n] = I;
        n++;
    }
    svpwm_modulation(0.0f, 0.0f);

    if (n < 2)
    {
        return MOTORID_ERR_NO_DATA; /* 单点拟合会被死区偏移污染，拒绝 */
    }

    float vmean = 0.0f;
    float imean = 0.0f;
    for (int k = 0; k < n; k++)
    {
        vmean += Vs[k];
        imean += Is[k];
    }
    vmean /= (float)n;
    imean /= (float)n;
    float num = 0.0f;
    float den = 0.0f;
    for (int k = 0; k < n; k++)
    {
        float dv = Vs[k] - vmean;
        float di = Is[k] - imean;
        num += dv * di;
        den += dv * dv;
    }
    h->Rs = den / num; /* R = dV/dI */
    h->v_offset = vmean - h->Rs * imean;
    return MOTORID_OK;
}

MotorID_Status_t MotorID_IdentifyInductance(MotorID_Handle_t *h)
{
    if (h->busy == 0u)
    {
        return MOTORID_ERR_BUSY;
    }
    if (h->Rs <= 0.0f)
    {
        return MOTORID_ERR_RS_UNKNOWN;
    }

    float V = h->v_induct;
    float f = h->f_induct;
    uint32_t half_samples = (uint32_t)(h->fs / (2.0f * f) + 0.5f);
    if (half_samples < 4u)
    {
        return MOTORID_ERR_FREQ_TOO_HIGH;
    }

    /* 峰值窗口 = 半周期采样点 + 1（W·Ts 即实际施加的半周期）:
     * 电压切换指令在窗口末发出，经 PWM 预装载约 1 个采样周期后生效；
     * 真峰值出现在切换瞬间，恰好落在下一窗口的首个样本上（该样本触发时
     * 电压刚切换约 1µs，电流尚未变化），故记账推迟到下一窗口首样本时进行 */
    uint32_t W = half_samples + 1u;

    /* 死区/压降偏移（α 轴），来自电阻辨识的 v_offset，用于精确反解 */
    float v_off = h->v_offset < 0.0f ? -h->v_offset : h->v_offset;

    uint32_t total_halves = (h->settle_cycles + h->cycles) * 2u;
    float sum_max = 0.0f;
    float sum_min = 0.0f;
    uint32_t n_max = 0u;
    uint32_t n_min = 0u;
    float cur_max = -1e30f;
    float cur_min = 1e30f;
    int32_t half_idx = 0;
    uint32_t phase = 0u; /* 0: +V 半周, 1: −V 半周 */
    uint32_t half_cnt = 0u;
    uint32_t finalize_pending = 0u;
    uint32_t pend_acc = 0u;
    uint32_t pend_phase = 0u;
    uint32_t retry = 0u;

retry_ls:
    sum_max = 0.0f;
    sum_min = 0.0f;
    n_max = 0u;
    n_min = 0u;
    cur_max = -1e30f;
    cur_min = 1e30f;
    half_idx = 0;
    phase = 0u;
    half_cnt = 0u;
    finalize_pending = 0u;
    pend_acc = 0u;
    pend_phase = 0u;

    svpwm_modulation(V, 0.0f);
    for (uint32_t s = 0u; s < total_halves * W; s++)
    {
        float ia;
        float ib;
        if (!motorid_sample(h, &ia, &ib))
        {
            svpwm_modulation(0.0f, 0.0f);
            return MOTORID_ERR_ADC_TIMEOUT;
        }
        if (ia > h->i_limit || ia < -h->i_limit)
        {
            /* 稳定阶段过流: 自动降压重试（峰值近似正比于 V） */
            if (half_idx < (int32_t)(h->settle_cycles * 2u) && retry < 3u)
            {
                float peak = ia > 0.0f ? ia : -ia;
                if (peak > 0.05f)
                {
                    V *= (0.7f * h->i_limit) / peak;
                    if (V < 0.05f)
                    {
                        V = 0.05f;
                    }
                }
                retry++;
                goto retry_ls;
            }
            svpwm_modulation(0.0f, 0.0f);
            return MOTORID_ERR_OVERCURRENT;
        }

        /* 1. 上一个窗口的记账: 本样本仍反映上一半的电压（真峰/真谷） */
        if (finalize_pending)
        {
            if (pend_phase == 0u)
            {
                if (ia > cur_max)
                {
                    cur_max = ia;
                }
                if (pend_acc)
                {
                    sum_max += cur_max;
                    n_max++;
                }
                cur_min = 1e30f;
            }
            else
            {
                if (ia < cur_min)
                {
                    cur_min = ia;
                }
                if (pend_acc)
                {
                    sum_min += cur_min;
                    n_min++;
                }
                cur_max = -1e30f;
            }
            finalize_pending = 0u;
        }

        /* 2. 当前窗口峰值/谷值跟踪（含本样本） */
        if (phase == 0u)
        {
            if (ia > cur_max)
            {
                cur_max = ia;
            }
        }
        else
        {
            if (ia < cur_min)
            {
                cur_min = ia;
            }
        }

        /* 3. 窗口边界: 发出切换指令（下一样本生效），记账推迟 */
        if (++half_cnt >= W)
        {
            half_cnt = 0u;
            pend_phase = phase;
            pend_acc = half_idx >= (int32_t)(h->settle_cycles * 2u) ? 1u : 0u;
            phase ^= 1u;
            finalize_pending = 1u;
            svpwm_modulation(phase == 0u ? V : -V, 0.0f);
            half_idx++;
        }
    }
    svpwm_modulation(0.0f, 0.0f);

    if (n_max == 0u || n_min == 0u)
    {
        return MOTORID_ERR_NO_DATA;
    }

    /* 精确反解（含死区偏移 v_off 的分段指数模型）:
     *   i_p = (K−1)(V²−v²)/(R·(V−v+K(V+v))), K = e^(h_eff/τ)
     *   ⇒ K = (V−v)(A+V+v) / ((V+v)(V−v−A)), A = R·i_p
     *   τ = h_eff / ln(K)，L = τ·R。v_off = 0 时退化为 atanh 模型。 */
    float ipp = sum_max / (float)n_max - sum_min / (float)n_min;
    float A = h->Rs * ipp * 0.5f; /* R·i_p */
    float den = (V + v_off) * (V - v_off - A);
    if (A <= 0.0f)
    {
        return MOTORID_ERR_CURRENT_TOO_SMALL;
    }
    if (A < 0.02f * V)
    {
        return MOTORID_ERR_CURRENT_TOO_SMALL; /* 电流纹波过小, ADC 分辨率不足 */
    }
    if (A >= 0.90f * (V - v_off))
    {
        return MOTORID_ERR_SATURATED; /* 纹波已接近饱和, 提高 f_induct 或减小 v_induct */
    }
    float K = (V - v_off) * (A + V + v_off) / den;
    if (K <= 1.0f)
    {
        return MOTORID_ERR_NO_DATA;
    }
    float h_eff = (float)W / h->fs;
    float tau = h_eff / motorid_ln(K);
    h->Ls = tau * h->Rs;
    return MOTORID_OK;
}

void MotorID_End(MotorID_Handle_t *h)
{
    svpwm_modulation(0.0f, 0.0f); /* 占空比归零(CCR = ARR 即零电压) */
    LL_ADC_ClearFlag_JEOS(ADC1);
    if (h->it_was_enabled)
    {
        LL_ADC_EnableIT_JEOS(ADC1); /* 恢复电流环中断 */
    }
    h->busy = 0u;
}

MotorID_Status_t MotorID_IdentifyAll(MotorID_Handle_t *h)
{
    MotorID_Status_t st = MotorID_Begin(h);
    if (st == MOTORID_OK)
    {
        st = MotorID_IdentifyResistance(h);
    }
    if (st == MOTORID_OK)
    {
        st = MotorID_IdentifyInductance(h);
    }
    MotorID_End(h);
    return st;
}

/* ============================ 磁链辨识（在线，非阻塞） ============================ */

void MotorID_FluxReset(MotorID_Handle_t *h)
{
    h->sum_wy = 0.0f;
    h->sum_w2 = 0.0f;
    h->flux_samples = 0u;
    h->i_q_prev = 0.0f;
    h->ds_cnt = 0u;
    h->ds_ratio = 0u;
}

void MotorID_FluxUpdate(MotorID_Handle_t *h, float v_q, float i_q, float omega_el, float Ts)
{
    if (h->Rs <= 0.0f || h->Ls <= 0.0f)
    {
        return; /* 需要已知 Rs/Ls */
    }

    /* 内部降采样到 ~1kHz（降低 di_q 数值微分噪声） */
    if (h->ds_ratio == 0u)
    {
        h->ds_ratio = (uint32_t)(0.001f / Ts + 0.5f);
        if (h->ds_ratio < 1u)
        {
            h->ds_ratio = 1u;
        }
        h->ds_Ts = Ts * (float)h->ds_ratio;
        h->ds_cnt = 0u;
    }
    if (++h->ds_cnt < h->ds_ratio)
    {
        return;
    }
    h->ds_cnt = 0u;

    float w_abs = omega_el < 0.0f ? -omega_el : omega_el;
    float v_abs = v_q < 0.0f ? -v_q : v_q;
    float i_abs = i_q < 0.0f ? -i_q : i_q;
    if (w_abs < h->omega_min)
    {
        return;
    }
    if (v_abs > h->v_q_max)
    {
        return; /* 电流环输出饱和，v_q 参考 ≠ 实际电压，跳过 */
    }
    if (i_abs > h->i_limit)
    {
        return;
    }

    float di_q = (i_q - h->i_q_prev) / h->ds_Ts;
    h->i_q_prev = i_q;
    float y = v_q - h->Rs * i_q - h->Ls * di_q; /* = ωe·ψf */

    h->sum_wy += omega_el * y;
    h->sum_w2 += omega_el * omega_el;
    h->flux_samples++;
    if (h->flux_samples >= MOTORID_FLUX_RENORM_SAMPLES)
    {
        /* 防止 float 累加溢出：整体缩放 */
        h->sum_wy *= 0.5f;
        h->sum_w2 *= 0.5f;
        h->flux_samples = MOTORID_FLUX_RENORM_SAMPLES / 2u;
    }
}

void MotorID_FluxUpdateFromFoc(MotorID_Handle_t *h)
{
    /* foc.real_speed 为机械转速 [rev/s]，乘 2π·极对数得电角速度 */
    float omega_el = foc.real_speed * MOTORID_TWO_PI * (float)POLE_PAIRS;
    MotorID_FluxUpdate(h, foc.target_park_v.q, foc.real_park_i.q, omega_el, SPEED_LOOP_TS);
}

float MotorID_FluxGetResult(MotorID_Handle_t *h)
{
    if (h->flux_samples < 100u || h->sum_w2 < 1.0f)
    {
        return 0.0f; /* 样本不足 */
    }
    h->flux_pm = h->sum_wy / h->sum_w2;
    return h->flux_pm;
}

/* ============================ 与 NLFO 联动 ============================ */

void MotorID_ApplyToNlfo(const MotorID_Handle_t *h, NLFO_Handle_t *nlfo)
{
    nlfo->Rs = h->Rs;
    nlfo->Ls = h->Ls;
    nlfo->flux_pm = h->flux_pm < 0.0f ? -h->flux_pm : h->flux_pm; /* 取幅值 */
}
