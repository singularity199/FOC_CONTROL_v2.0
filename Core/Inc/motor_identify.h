// clang-format off
/**
 ******************************************************************************
 * @file    motor_identify.h
 * @brief   电机参数自辨识模块（MotorID）：
 *             - 相电阻 Rs：多电平直流注入 + 最小二乘拟合（自动剔除死区/压降偏移）
 *             - 相电感 Ls：方波电压注入 + 稳态电流峰峰值 + 解析反解
 *             - 永磁磁链 ψf：在线运行中由 dq 电压方程最小二乘估计
 *           辨识结果可直接写入 NLFO 观测器（MotorID_ApplyToNlfo）。
 *
 * @note    本文件为【新增独立模块】，不修改任何现有源码。
 *           阻感辨识为阻塞式：临时接管 PWM 输出并暂停电流环中断（结束后自动
 *           恢复），电机必须【静止】；磁链辨识为非阻塞在线估计，与 FOC 同时
 *           运行，电机【正常旋转】即可。
 *
 * 典型用法：
 *   1) main() 初始化处：
 *          MotorID_Handle_t mid;
 *          MotorID_Init(&mid);
 *          MotorID_ConfigAdc(&mid, foc.adc.i_a_offset, foc.adc.i_b_offset,
 *                                  foc.adc.i_c_offset, ADC_Coefficient, V_BUS);
 *
 *   2) 阻感辨识（阻塞约 1~2 s，调用前电机必须静止）：
 *          MotorID_Status_t st = MotorID_IdentifyAll(&mid);  // Begin + Rs + Ls + End
 *          成功后 mid.Rs、mid.Ls 即辨识结果。
 *
 *   3) 磁链辨识（在线，恒速轻载段效果最好）：
 *          在 stm32g4xx_it.c 的 TIM2_IRQHandler 速度环里加一行：
 *              MotorID_FluxUpdateFromFoc(&mid);
 *          恒速运行 2~5 s 后读取：
 *              float flux_pm = MotorID_FluxGetResult(&mid);
 *
 *   4) 一键写入 NLFO：
 *          MotorID_ApplyToNlfo(&mid, &nlfo);
 *
 * @attention  阻感辨识期间请勿按下对齐按钮或发送遥控指令（会重启电流环中断）。
 *             磁链辨识需要 Rs/Ls 已已知（先做阻感辨识或 MotorID_SetParams）。
 ******************************************************************************
 */

#ifndef _MOTOR_IDENTIFY_H
#define _MOTOR_IDENTIFY_H

#include <stdint.h>

#include "nlfo.h"

#ifdef __cplusplus
extern "C"
{
#endif

/* ------------------------------ 状态码 ------------------------------ */
typedef enum
{
    MOTORID_OK = 0,              /* 成功 */
    MOTORID_ERR_BUSY,            /* 上一次辨识未结束（未调用 End/IdentifyAll） */
    MOTORID_ERR_ADC_TIMEOUT,     /* ADC 无触发：检查 TIM1 是否运行、系统是否上电 */
    MOTORID_ERR_OVERCURRENT,     /* 电流超过 i_limit，已停止输出 */
    MOTORID_ERR_CURRENT_TOO_SMALL, /* 电流过小（ADC 分辨率不足），已尝试自动增大电压 */
    MOTORID_ERR_SATURATED,       /* 方波电流饱和：增大 f_induct 或减小 v_induct */
    MOTORID_ERR_FREQ_TOO_HIGH,   /* 注入频率过高：半周期采样点不足 4 个 */
    MOTORID_ERR_NO_DATA,         /* 有效数据点不足 */
    MOTORID_ERR_RS_UNKNOWN,      /* 电感辨识需要已知 Rs（先做电阻辨识） */
} MotorID_Status_t;

/* ------------------------------ 句柄 ------------------------------ */
typedef struct
{
    /* ===== 辨识结果（只读，成功后自动更新） ===== */
    float Rs;        /* 相电阻 [Ω] */
    float Ls;        /* 相电感 [H] */
    float flux_pm;   /* 永磁磁链 ψf [Wb]（LSQ 结果，负值说明速度符号约定相反，取绝对值用） */
    float v_offset;  /* 电阻辨识拟合出的电压截距 [V]（死区/器件压降，诊断用） */

    /* ===== 电阻辨识配置 ===== */
    float v_dc_start;      /* 起始注入电压 [V]，默认 1.0（内部自适应缩放） */
    float settle_first_ms; /* 第一级注入等待 [ms]，默认 400（等转子对齐） */
    float settle_ms;       /* 后续每级等待 [ms]，默认 100 */
    uint32_t avg_samples;  /* 每点电流平均样本数，默认 2000 */
    float i_limit;         /* 安全电流限幅 [A]，默认 2.0 */

    /* ===== 电感辨识配置 ===== */
    float v_induct;        /* 方波注入电压幅值 [V]，默认 3.0 */
    float f_induct;        /* 方波注入频率 [Hz]，默认 1000 */
    uint32_t cycles;       /* 有效测量周期数，默认 100 */
    uint32_t settle_cycles;/* 测量前稳定周期数，默认 100 */

    /* ===== 磁链辨识配置（在线） ===== */
    float omega_min;       /* 参与辨识的最小电角速度 [rad/s]，默认 100 */
    float v_q_max;         /* |v_q| 超过该值视为电流环饱和，跳过样本 [V]，默认 15 */

    /* ===== 采样配置 ===== */
    uint32_t adc_off_a;    /* A 相 ADC 零点（传 foc.adc.i_a_offset） */
    uint32_t adc_off_b;    /* B 相 ADC 零点（foc.adc.i_b_offset） */
    uint32_t adc_off_c;    /* C 相 ADC 零点（foc.adc.i_c_offset） */
    float adc_gain;        /* 电流系数 [A/LSB]，默认 ADC_Coefficient */
    float v_bus;           /* 母线电压 [V]，默认 V_BUS */

    /* ===== 磁链 LSQ 累加器（内部，勿手动修改） ===== */
    float sum_wy;
    float sum_w2;
    uint32_t flux_samples;
    float i_q_prev;
    uint32_t ds_cnt;
    uint32_t ds_ratio;
    float ds_Ts;

    /* ===== 运行状态（内部） ===== */
    uint32_t it_was_enabled; /* Begin 前保存的 JEOS 中断使能状态 */
    uint32_t busy;           /* 阻感辨识进行中标志 */
    float fs;                /* Begin 时实测的采样频率 [Hz] */
} MotorID_Handle_t;

/* ------------------------------ 初始化 / 配置 ------------------------------ */

/** @brief 初始化辨识器：载入默认参数（电压/频率/限幅），清零全部结果与状态。 */
void MotorID_Init(MotorID_Handle_t *h);

/**
 * @brief  配置电流采样：ADC 零点、电流系数与母线电压。
 *         零点直接复用 foc 已校准值（见文件头示例）。
 */
void MotorID_ConfigAdc(MotorID_Handle_t *h, uint32_t off_a, uint32_t off_b, uint32_t off_c,
                       float adc_gain, float v_bus);

/** @brief 手动设置电机参数（跳过阻感辨识，直接做磁链辨识时用）。 */
void MotorID_SetParams(MotorID_Handle_t *h, float Rs, float Ls);

/* ------------------------------ 阻感辨识（阻塞式） ------------------------------ */

/**
 * @brief  开始辨识：保存并关闭电流环中断（JEOS IT），确保 TIM1/ADC 触发就绪，
 *         实测采样频率。之后可调用 IdentifyResistance / IdentifyInductance。
 * @return 状态码
 */
MotorID_Status_t MotorID_Begin(MotorID_Handle_t *h);

/**
 * @brief  辨识相电阻 Rs（多电平直流注入 + 最小二乘，自动剔除死区偏移）。
 *         必须已调用 MotorID_Begin，且电机静止（转子自由或锁轴均可）。
 * @return 状态码；成功后结果在 h->Rs（h->v_offset 为电压截距）
 */
MotorID_Status_t MotorID_IdentifyResistance(MotorID_Handle_t *h);

/**
 * @brief  辨识相电感 Ls（方波注入 + 峰峰值反解）。需要已知 Rs。
 *         必须已调用 MotorID_Begin，且电机静止。
 * @return 状态码；成功后结果在 h->Ls
 */
MotorID_Status_t MotorID_IdentifyInductance(MotorID_Handle_t *h);

/**
 * @brief  结束辨识：PWM 归零，恢复电流环中断（若之前是开启的）。
 */
void MotorID_End(MotorID_Handle_t *h);

/**
 * @brief  一键完成 Rs + Ls 辨识（Begin → Rs → Ls → End，中途出错也会安全收尾）。
 * @return 最后一步的状态码；成功后 mid.Rs / mid.Ls 即结果
 */
MotorID_Status_t MotorID_IdentifyAll(MotorID_Handle_t *h);

/* ------------------------------ 磁链辨识（在线，非阻塞） ------------------------------ */

/** @brief 清零磁链 LSQ 累加器（重新开始辨识前调用）。 */
void MotorID_FluxReset(MotorID_Handle_t *h);

/**
 * @brief  磁链在线辨识更新（每个速度环周期调用一次，内部自动降采样到 ~1kHz）。
 *         依据 dq 电压方程 v_q = Rs·i_q + Ls·di_q/dt + ωe·ψf 做最小二乘。
 * @param  h        辨识器句柄（Rs/Ls 必须已知）
 * @param  v_q      q 轴电压 [V]（foc.target_park_v.q）
 * @param  i_q      q 轴电流 [A]（foc.real_park_i.q）
 * @param  omega_el 电角速度 [rad/s]
 * @param  Ts       本次调用周期 [s]
 */
void MotorID_FluxUpdate(MotorID_Handle_t *h, float v_q, float i_q, float omega_el, float Ts);

/**
 * @brief  便捷封装：直接从 foc 全局变量取数（v_q/i_q/电角速度），
 *         放到 TIM2_IRQHandler 速度环里一行即可：
 *             MotorID_FluxUpdateFromFoc(&mid);
 *         （foc.real_speed 按机械转速 rev/s 换算电角速度）
 */
void MotorID_FluxUpdateFromFoc(MotorID_Handle_t *h);

/**
 * @brief  读取磁链辨识结果 ψf [Wb]。
 * @return 样本不足时返回 0；否则返回 LSQ 估计值并写入 h->flux_pm
 */
float MotorID_FluxGetResult(MotorID_Handle_t *h);

/* ------------------------------ 与 NLFO 联动 ------------------------------ */

/** @brief 把辨识结果（Rs/Ls/ψf 幅值）写入 NLFO 观测器。 */
void MotorID_ApplyToNlfo(const MotorID_Handle_t *h, NLFO_Handle_t *nlfo);

#ifdef __cplusplus
}
#endif

#endif /* _MOTOR_IDENTIFY_H */
