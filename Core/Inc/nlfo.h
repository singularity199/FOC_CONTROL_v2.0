// clang-format off
/**
 ******************************************************************************
 * @file    nlfo.h
 * @brief   NLFO（NonLinear Flux Observer，非线性磁链观测器）
 *          三线无刷电机无传感器转子位置 / 电角速度观测模块。
 *
 * @note    本文件为【新增独立模块】，不修改任何现有源码。
 *          核心算法不依赖外设与数学库（sin/cos/atan2/sqrt 均为自实现
 *          单精度多项式，已在 float32 下验证：atan2 误差 < 2e-6 rad，
 *          sin/cos 误差 < 2e-7）。可选的自采集模式只读取 ADC/TIM 寄存器。
 *
 * 原理（αβ 静止坐标系，参考 Lee/Hong/Nam/Ortega 等提出的 NLFO）：
 *          定子磁链:        λ = Ls·i + ψf·[cosθe, sinθe]^T
 *          电压方程:        dλ/dt = v − Rs·i
 *          观测器:          dλ̂/dt = v − Rs·i − ℓ·(ρ̂ − ψf)·η̂
 *                           其中 ρ̂ = |λ̂ − Ls·i|, η̂ = (λ̂ − Ls·i)/ρ̂
 *          转子角度:        θ̂_raw = atan2(λ̂β − Ls·iβ, λ̂α − Ls·iα)
 *          角度/速度:       θ̂_raw 经 PLL 锁相环平滑输出 θ̂、ω̂
 *
 * @attention  零速 / 极低速时反电动势为零，观测器不收敛；本模块仅输出
 *             观测值，不改变任何控制量。接入前建议先与编码器对比验证。
 *
 ******************************************************************************
 */
/* USER CODE END Header */

#ifndef _NLFO_H
#define _NLFO_H

#include <stdint.h>

#ifdef __cplusplus
extern "C"
{
#endif

/* ===================== 使用说明（二选一，均只需加 1 行调用） =====================
 *
 * 准备（main() 中，任何位置）:
 *      NLFO_Handle_t nlfo;
 *      NLFO_Init(&nlfo, Rs, Ls, flux_pm, 1.0f/19318.0f);
 *          Rs/Ls    : 相电阻[Ω]/相电感[H]，用电桥或 LCR 表测量
 *          flux_pm  : 永磁磁链幅值 ψf[Wb] = 反电动势幅值 / 电角速度；
 *                     由 Kv 粗估可用 ψf ≈ 5.51/Kv（量级正确即可）
 *          Ts       : 电流环周期。本项目 TIM1 中心对齐, 170MHz/(20*2*220)
 *                     ≈ 19318 Hz，即 Ts = 1/19318 s
 *
 * 模式 A（推荐，直接复用 FOC 环内变量，坐标系与现有 FOC 完全一致）:
 *      在 stm32g4xx_it.c 的 ADC1_2_IRQHandler 中，
 *      foc_current_loop_control(...) 这一行【之后】加入:
 *          NLFO_Update(&nlfo, foc.target_clark_v.alpha, foc.target_clark_v.beta,
 *                             foc.real_clark_i.alpha, foc.real_clark_i.beta);
 *      （可选）启动/复位后用编码器角度对齐一次:
 *          NLFO_Reset(&nlfo, get_angle_el());
 *
 * 模式 B（完全自采集，不依赖 foc 结构体，同样只需在 ISR 内加一行）:
 *      先配置采样器（ADC 零点直接复用 foc 已校准好的值）:
 *          NLFO_ConfigSampler(&nlfo, foc.adc.i_a_offset, foc.adc.i_b_offset,
 *                                     foc.adc.i_c_offset, ADC_Coefficient,
 *                                     V_BUS, TIM_PWM_ARR);
 *      ISR 内（JEOS 中断里，保证电流与 CCR 同一拍读取）:
 *          NLFO_Input_t in;
 *          NLFO_SampleInputs(&nlfo, &in);
 *          NLFO_UpdateInputs(&nlfo, &in);
 *
 * 读取结果（主循环里，例如送 USB 画图）:
 *      float theta_obs  = NLFO_GetThetaEl(&nlfo);            // 观测电角度 [−π, π]
 *      float omega_obs  = NLFO_GetOmegaEl(&nlfo);            // 观测电角速度 [rad/s]
 *      float err_obs    = NLFO_GetAngleError(&nlfo, get_angle_el()); // 与编码器误差
 *      uint32_t ok      = NLFO_IsConverged(&nlfo);           // 收敛标志
 * ============================================================================ */

/* 观测器一次更新的输入量（αβ 坐标系，与现有 FOC 的 Clarke 定义一致） */
typedef struct
{
    float i_alpha; /* α 轴电流 [A] */
    float i_beta;  /* β 轴电流 [A] */
    float v_alpha; /* α 轴电压 [V] */
    float v_beta;  /* β 轴电压 [V] */
} NLFO_Input_t;

typedef struct
{
    /* ---------- 电机参数（按实际电机填写，Init 后可用 Setter 修改） ---------- */
    float Rs;      /* 相电阻 [Ω] */
    float Ls;      /* 相电感 [H]，表贴式/无刷 Ld≈Lq */
    float flux_pm; /* 永磁磁链幅值 ψf [Wb]，必须 > 0 */

    /* ---------- 观测器参数 ---------- */
    float Ts;        /* 观测周期 [s] = 1/电流环频率 */
    float gain;      /* 非线性磁链反馈增益 ℓ [1/s]，默认 200（建议 50~500） */
    float pll_kp;    /* PLL 比例增益，默认 1000 */
    float pll_ki;    /* PLL 积分增益，默认 600000（ωn≈775 rad/s, ζ≈0.65） */
    float omega_max; /* PLL 输出电角速度限幅 [rad/s]，默认 2000 */
    float omega_lpf; /* 输出速度低通系数 α (0~1]，默认 0.1 */
    float omega_min; /* 收敛判定的最小电角速度 [rad/s]，默认 30 */

    /* ---------- 观测状态（只读，每拍 NLFO_Update 刷新） ---------- */
    float lambda_alpha; /* 定子磁链估计 λ̂α [Wb] */
    float lambda_beta;  /* 定子磁链估计 λ̂β [Wb] */
    float psi_alpha;    /* 转子磁链估计 λ̂α − Ls·iα [Wb] */
    float psi_beta;     /* 转子磁链估计 λ̂β − Ls·iβ [Wb] */
    float rho;          /* 转子磁链幅值 |ψ̂| [Wb] */
    float theta_raw;    /* atan2 原始电角度 [rad]，范围 [−π, π] */
    float theta_el;     /* PLL 输出电角度 [rad]，范围 [−π, π]（可直接给 park_trans） */
    float omega_el;     /* PLL 输出电角速度 [rad/s] */
    float omega_el_f;   /* 低通滤波后电角速度 [rad/s] */
    float pll_err;      /* PLL 相位误差 sin(θraw − θpll)（诊断用） */
    float pll_int;      /* PLL 积分器状态 */
    float i_alpha;      /* 最近一次输入 α 电流 */
    float i_beta;       /* 最近一次输入 β 电流 */
    float v_alpha;      /* 最近一次输入 α 电压 */
    float v_beta;       /* 最近一次输入 β 电压 */
    uint32_t converged; /* 收敛标志：ρ > 0.2·ψf 且 |ω| > omega_min */
    uint32_t tick;      /* 累计更新次数 */

    /* ---------- 自采集配置（模式 B，用 NLFO_ConfigSampler 设置） ---------- */
    uint32_t adc_off_a; /* A 相 ADC 零点（LSB），复用 foc.adc.i_a_offset */
    uint32_t adc_off_b; /* B 相 ADC 零点，复用 foc.adc.i_b_offset */
    uint32_t adc_off_c; /* C 相 ADC 零点，复用 foc.adc.i_c_offset */
    float adc_gain;     /* 电流系数 [A/LSB]，默认 ADC_Coefficient */
    float v_bus;        /* 母线电压 [V]，默认 V_BUS */
    float pwm_arr;      /* TIM1 自动重载值，默认 TIM_PWM_ARR */
} NLFO_Handle_t;

/* ------------------------------ 初始化 / 复位 ------------------------------ */

/**
 * @brief  初始化观测器，载入电机参数并复位全部状态。
 * @param  h        观测器句柄
 * @param  Rs       相电阻 [Ω]
 * @param  Ls       相电感 [H]
 * @param  flux_pm  永磁磁链幅值 ψf [Wb]，必须 > 0
 * @param  Ts       观测周期 [s]
 */
void NLFO_Init(NLFO_Handle_t *h, float Rs, float Ls, float flux_pm, float Ts);

/**
 * @brief  用已知电角度复位观测器（启动/失步恢复时对齐，可配合编码器使用）。
 *         将 λ̂ 置为 Ls·i + ψf·[cosθ0, sinθ0]^T，θ̂ = θ0，ω̂ = 0。
 * @param  h         观测器句柄
 * @param  theta0_el 对齐电角度 [rad]（内部自动归一到 [−π, π]）
 */
void NLFO_Reset(NLFO_Handle_t *h, float theta0_el);

/* ------------------------------ 周期调用 ------------------------------ */

/**
 * @brief  观测器主更新函数，每个电流环周期调用一次（与 FOC 同拍，见文件头示例）。
 * @param  h       观测器句柄
 * @param  v_alpha α 轴电压 [V]（与 foc.target_clark_v.alpha 同一坐标系）
 * @param  v_beta  β 轴电压 [V]
 * @param  i_alpha α 轴电流 [A]（与 foc.real_clark_i.alpha 同一坐标系）
 * @param  i_beta  β 轴电流 [A]
 */
void NLFO_Update(NLFO_Handle_t *h, float v_alpha, float v_beta, float i_alpha, float i_beta);

/**
 * @brief  NLFO_Update 的结构体版本，配合 NLFO_SampleInputs 使用。
 */
void NLFO_UpdateInputs(NLFO_Handle_t *h, const NLFO_Input_t *in);

/* ------------------------------ 参数设置 ------------------------------ */

/** @brief 设置非线性磁链反馈增益 ℓ [1/s] */
void NLFO_SetGain(NLFO_Handle_t *h, float gain);

/** @brief 设置 PLL 参数：kp、ki 及输出速度限幅 omega_max [rad/s] */
void NLFO_SetPll(NLFO_Handle_t *h, float kp, float ki, float omega_max);

/** @brief 设置输出速度滤波器系数 alpha 与收敛速度阈值 omega_min [rad/s] */
void NLFO_SetOmegaFilter(NLFO_Handle_t *h, float lpf_alpha, float omega_min);

/**
 * @brief  配置自采集（模式 B）参数。
 * @param  h        观测器句柄
 * @param  off_a    A 相 ADC 零点（直接传 foc.adc.i_a_offset）
 * @param  off_b    B 相 ADC 零点（foc.adc.i_b_offset）
 * @param  off_c    C 相 ADC 零点（foc.adc.i_c_offset）
 * @param  adc_gain 电流系数 [A/LSB]（传 ADC_Coefficient）
 * @param  v_bus    母线电压 [V]（传 V_BUS）
 * @param  pwm_arr  TIM1 ARR（传 TIM_PWM_ARR）
 */
void NLFO_ConfigSampler(NLFO_Handle_t *h, uint32_t off_a, uint32_t off_b, uint32_t off_c,
                        float adc_gain, float v_bus, float pwm_arr);

/* ------------------------------ 自采集 ------------------------------ */

/**
 * @brief  自采集一次输入量（模式 B，在 JEOS 中断内调用，与电流环同拍）。
 *         电流通道映射与 foc_current_loop_control 完全一致：
 *             i_a = ADC1 注入 Rank1, i_c = ADC1 注入 Rank2, i_b = ADC2 注入 Rank2
 *         αβ 变换与现有 Clarke 定义一致：
 *             i_alpha = (i_a + i_b − 2·i_c)/3,   i_beta = (i_a − i_b)/√3
 *         电压由 TIM1 三相比较值按本项目 SVPWM 公式精确反解（线性调制区内为
 *         精确逆映射，残差 < 1e-13，含过调制时给出实际输出伏秒）：
 *             v_alpha = V_BUS/(6·ARR)·(CCR2 + CCR3 − 2·CCR1)
 *             v_beta  = V_BUS/(√3·ARR)·(CCR3 − CCR2)
 * @param  h   观测器句柄（需已用 NLFO_ConfigSampler 配置零点/增益/母线/ARR）
 * @param  in  输出：采集到的 iαβ / vαβ
 */
void NLFO_SampleInputs(NLFO_Handle_t *h, NLFO_Input_t *in);

/* ------------------------------ 结果读取 ------------------------------ */

/** @brief 读取 PLL 输出的观测电角度 [rad]，范围 [−π, π] */
float NLFO_GetThetaEl(const NLFO_Handle_t *h);

/** @brief 读取滤波后的观测电角速度 [rad/s] */
float NLFO_GetOmegaEl(const NLFO_Handle_t *h);

/** @brief 读取转子磁链幅值 |ψ̂| [Wb]（诊断用） */
float NLFO_GetRho(const NLFO_Handle_t *h);

/** @brief 计算观测角度与参考角度的误差 [rad]（归一到 [−π, π]，可用编码器角度作参考） */
float NLFO_GetAngleError(const NLFO_Handle_t *h, float ref_theta_el);

/** @brief 是否收敛（磁链幅值足够且转速高于 omega_min） */
uint32_t NLFO_IsConverged(const NLFO_Handle_t *h);

#ifdef __cplusplus
}
#endif

#endif /* _NLFO_H */
