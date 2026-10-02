#include "nonlinear_flux_observer.h"
#include "main.h"
#include <string.h>
#include "motor_control.h"

/**
 * @brief 映射角度到 [0, 2π) 范围
 */
static float NFO_Wrap0To2Pi(float x)
{
    while (x >= NFO_TWO_PI)
    {
        x -= NFO_TWO_PI;
    }

    while (x < 0.0f)
    {
        x += NFO_TWO_PI;
    }

    return x;
}
/**
 * @brief 映射角度到 [-π, π) 范围
 */
static float NFO_WrapMinusPiToPi(float x)
{
    while (x >= NFO_PI)
    {
        x -= NFO_TWO_PI;
    }

    while (x < -NFO_PI)
    {
        x += NFO_TWO_PI;
    }

    return x;
}
/**
 * @brief 限幅
 */
static float NFO_ClampFloat(float x, float min_val, float max_val)
{
    if (x > max_val)
    {
        return max_val;
    }

    if (x < min_val)
    {
        return min_val;
    }

    return x;
}
static float NFO_AbsFloat(float x)
{
    if (x >= 0.0f)
    {
        return x;
    }
    else
    {
        return -x;
    }
}
/**
 * @brief 角度转换为 Q32 格式
 */
static uint32_t NFO_RadToQ32(float theta)
{
    theta = NFO_Wrap0To2Pi(theta);

    return (uint32_t)(theta * NFO_Q32_PER_RAD);
}
/**
 * @brief Q32 格式转换为弧度
 */
static float NFO_Q32ToRad(uint32_t theta_acc)
{
    return ((float)theta_acc) / NFO_Q32_PER_RAD;
}
/*
 * 假设输入已经接近单位圆：
 * 1 / sqrt(norm2) 的一阶牛顿近似：
 * scale ≈ 1.5 - 0.5 * norm2
 */
static void NFO_NormalizeUnitVector(float *x, float *y)
{
    float xx = *x;
    float yy = *y;

    float norm2 = xx * xx + yy * yy;
    float scale = 1.5f - 0.5f * norm2;

    *x = xx * scale;
    *y = yy * scale;
}

void NFO_Init(NFO_Handle_t *obs, float Rs, float Ls, float psi_f, float Ts, float k_nl)
{
    if (obs == 0)
    {
        return;
    }

    memset(obs, 0, sizeof(NFO_Handle_t));

    obs->Rs = Rs;
    obs->Ls = Ls;
    obs->psi_f = psi_f;
    obs->Ts = Ts;
    obs->k_nl = k_nl;

    obs->psi_f2 = psi_f * psi_f;

    if (psi_f > 0.0f)
    {
        obs->inv_psi_f = 1.0f / psi_f;
    }
    else
    {
        obs->inv_psi_f = 0.0f;
    }

    obs->theta_offset = 0.0f;
    obs->theta_offset_acc = 0u;

    obs->speed_lpf_alpha = 0.05f;

    obs->lambda_alpha = psi_f;
    obs->lambda_beta = 0.0f;
    obs->emf_alpha_prev = 0.0f;
    obs->emf_beta_prev = 0.0f;
    obs->emf_prev_valid = 0u;

    obs->emf_mag2 = 0.0f;
    obs->corr_mag2 = 0.0f;
    obs->corr_emf_ratio = 0.0f;

    obs->pm_alpha = psi_f;
    obs->pm_beta = 0.0f;
    obs->pm2 = psi_f * psi_f;
    obs->pm_abs = psi_f;

    obs->theta_acc = 0u;
    obs->theta_e = 0.0f;
    obs->theta_e_raw = 0.0f;
    obs->theta_prev = 0.0f;

    obs->omega_e = 0.0f;
    obs->omega_e_raw = 0.0f;

    obs->err_flux = 0.0f;

    /*
     * PLL 默认参数
     */
    obs->pll_kp = 1500.0f;
    obs->pll_ki = 80000.0f;
    obs->pll_integrator = 0.0f;
    obs->pll_phase_err = 0.0f;
    obs->pll_phase_err_limit = 0.8f;
    obs->pll_omega_limit = 30000.0f;
    obs->pll_phase_sign = 1.0f;

    obs->cos_e = 1.0f;
    obs->sin_e = 0.0f;

    obs->initialized = 1u;
}

void NFO_SetParam(NFO_Handle_t *obs, float Rs, float Ls, float psi_f, float Ts, float k_nl)
{
    if (obs == 0)
    {
        return;
    }

    obs->Rs = Rs;
    obs->Ls = Ls;
    obs->psi_f = psi_f;
    obs->Ts = Ts;
    obs->k_nl = k_nl;

    obs->psi_f2 = psi_f * psi_f;

    if (psi_f > 0.0f)
    {
        obs->inv_psi_f = 1.0f / psi_f;
    }
    else
    {
        obs->inv_psi_f = 0.0f;
    }
}
void NFO_SetThetaOffset(NFO_Handle_t *obs, float theta_offset)
{
    if (obs == 0)
    {
        return;
    }

    obs->theta_offset = NFO_Wrap0To2Pi(theta_offset);
    obs->theta_offset_acc = NFO_RadToQ32(obs->theta_offset);
}
void NFO_SetSpeedFilter(NFO_Handle_t *obs, float alpha)
{
    if (obs == 0)
    {
        return;
    }

    obs->speed_lpf_alpha = NFO_ClampFloat(alpha, 0.0f, 1.0f);
}
void NFO_SetPLL(NFO_Handle_t *obs, float kp, float ki)
{
    if (obs == 0)
    {
        return;
    }

    obs->pll_kp = kp;
    obs->pll_ki = ki;
}
void NFO_SetPLLOmegaLimit(NFO_Handle_t *obs, float omega_limit)
{
    if (obs == 0)
    {
        return;
    }

    if (omega_limit > 0.0f)
    {
        obs->pll_omega_limit = omega_limit;
    }
}
void NFO_SetPLLPhaseSign(NFO_Handle_t *obs, float sign)
{
    if (obs == 0)
    {
        return;
    }

    if (sign >= 0.0f)
    {
        obs->pll_phase_sign = 1.0f;
    }
    else
    {
        obs->pll_phase_sign = -1.0f;
    }
}

float NFO_GetPLLPhaseErr(const NFO_Handle_t *obs)
{
    if (obs == 0)
    {
        return 0.0f;
    }

    return obs->pll_phase_err;
}

void NFO_ResetAngle(NFO_Handle_t *obs, float theta_e, float i_alpha, float i_beta)
{
    if (obs == 0)
    {
        return;
    }

    theta_e = NFO_Wrap0To2Pi(theta_e);

    obs->theta_acc = NFO_RadToQ32(theta_e);

    /*
     * 用 Q32 CORDIC 直接算初始 sin/cos
     */
    cordic_angle angle = cor_q32(obs->theta_acc);

    obs->sin_e = angle.sin_angle;
    obs->cos_e = angle.cos_angle;

    obs->theta_e_raw = theta_e;
    obs->theta_e = NFO_Q32ToRad(obs->theta_acc + obs->theta_offset_acc);
    obs->theta_prev = theta_e;

    /*
     * lambda = Ls*i + psi_f*[cos, sin]
     */
    obs->pm_alpha = obs->psi_f * obs->cos_e;
    obs->pm_beta = obs->psi_f * obs->sin_e;

    obs->lambda_alpha = obs->Ls * i_alpha + obs->pm_alpha;
    obs->lambda_beta = obs->Ls * i_beta + obs->pm_beta;
    /*
     * 重新定位后清除梯形积分历史，
     * 防止带入重新定位前的导数。
     */
    obs->emf_alpha_prev = 0.0f;
    obs->emf_beta_prev = 0.0f;
    obs->emf_prev_valid = 0u;

    obs->emf_mag2 = 0.0f;
    obs->corr_mag2 = 0.0f;
    obs->corr_emf_ratio = 0.0f;

    obs->pm2 = obs->pm_alpha * obs->pm_alpha + obs->pm_beta * obs->pm_beta;
    obs->pm_abs = obs->psi_f;

    obs->err_flux = obs->psi_f2 - obs->pm2;

    obs->pll_integrator = 0.0f;
    obs->pll_phase_err = 0.0f;

    obs->omega_e = 0.0f;
    obs->omega_e_raw = 0.0f;

    obs->initialized = 1u;
}

float NFO_Update(NFO_Handle_t *obs, float v_alpha, float v_beta, float i_alpha, float i_beta)
{
    if (obs == 0)
    {
        return 0.0f;
    }

    if (obs->initialized == 0u)
    {
        NFO_ResetAngle(obs, 0.0f, i_alpha, i_beta);
    }

    /*
     * 1. 当前永磁体磁链估计：
     * pm = lambda - Ls*i
     */
    float pm_alpha = obs->lambda_alpha - obs->Ls * i_alpha;
    float pm_beta = obs->lambda_beta - obs->Ls * i_beta;

    float pm2 = pm_alpha * pm_alpha + pm_beta * pm_beta;

    /*
     * 2. 非线性磁链幅值约束：
     * err_flux = psi_f^2 - |pm|^2
     */
    float err_flux = obs->psi_f2 - pm2;

    /*
     * 2. 非线性磁链幅值约束
     *
     * corr 是非线性校正矢量：
     * corr_alpha、corr_beta 分别是 alpha、beta 分量。
     */
    float corr_alpha =
        obs->k_nl * err_flux * pm_alpha;

    float corr_beta =
        obs->k_nl * err_flux * pm_beta;

    /*
     * 3. 电压模型矢量
     *
     * emf_alpha = v_alpha - Rs * i_alpha
     * emf_beta  = v_beta  - Rs * i_beta
     */
    float emf_alpha =
        v_alpha - obs->Rs * i_alpha;

    float emf_beta =
        v_beta - obs->Rs * i_beta;

    /*
     * 调试量：两个 alpha-beta 矢量的模长平方。
     *
     * emf_mag2  = emf_alpha^2  + emf_beta^2
     * corr_mag2 = corr_alpha^2 + corr_beta^2
     */
    obs->emf_mag2 =
        emf_alpha * emf_alpha +
        emf_beta * emf_beta;

    obs->corr_mag2 =
        corr_alpha * corr_alpha +
        corr_beta * corr_beta;

    /*
     * 非线性校正量相对电压模型量的平方比。
     *
     * 由于这里使用的是模长平方，所以：
     *
     * corr_emf_ratio = 0.01
     * 表示校正矢量模长约为电压模型矢量的 10%；
     *
     * corr_emf_ratio = 1
     * 表示两者模长大致相等。
     */
    obs->corr_emf_ratio =
        obs->corr_mag2 /
        (obs->emf_mag2 + 1.0e-12f);

    /*
     * Reset 后第一拍没有上一拍 emf。
     * 将上一拍设置为当前拍，使第一拍等效为：
     *
     * lambda += Ts * emf
     *
     * 而不是只积分半拍。
     */
    /*
     * 当前时序下使用单拍积分：
     *
     * emf[k] = v_applied[k-1] - Rs*i[k]
     */
    float dlambda_alpha =
        emf_alpha + corr_alpha;

    float dlambda_beta =
        emf_beta + corr_beta;

    obs->lambda_alpha +=
        obs->Ts * dlambda_alpha;

    obs->lambda_beta +=
        obs->Ts * dlambda_beta;

    /*
     * 4. 更新后的永磁体磁链
     */
    pm_alpha = obs->lambda_alpha - obs->Ls * i_alpha;
    pm_beta = obs->lambda_beta - obs->Ls * i_beta;

    pm2 = pm_alpha * pm_alpha + pm_beta * pm_beta;

    obs->pm_alpha = pm_alpha;
    obs->pm_beta = pm_beta;
    obs->pm2 = pm2;

    obs->err_flux = obs->psi_f2 - pm2;

    /*
     * 不用 sqrt，只给一个近似调试值。
     * 真正建议看 obs->pm2 和 obs->err_flux。
     */
    if (obs->psi_f > 0.0f)
    {
        obs->pm_abs = obs->psi_f - obs->err_flux * 0.5f * obs->inv_psi_f;
    }
    else
    {
        obs->pm_abs = 0.0f;
    }

    /*
     * 5. 使用上一拍的速度，把PLL角度推进到当前采样时刻。
     *
     * 此时 omega_e_raw 仍然是上一拍PLL计算出的速度。
     * 新的 omega_raw 会在后面算出，留给下一次调用使用。
     */
    float dtheta = obs->omega_e_raw * obs->Ts;

    dtheta = NFO_ClampFloat(dtheta, -3.0f, 3.0f);

    int32_t inc_q32 =
        (int32_t)(dtheta * NFO_Q32_PER_RAD);

    obs->theta_acc += (uint32_t)inc_q32;

    /*
     * 当前采样时刻的PLL角度正余弦。
     */
    cordic_angle angle_now =
        cor_q32(obs->theta_acc);

    obs->sin_e = angle_now.sin_angle;
    obs->cos_e = angle_now.cos_angle;

    /*
     * 6. PLL相位误差
     *
     * phase_err = sin(theta_pm - theta_pll)
     */
    float cross =
        obs->cos_e * pm_beta -
        obs->sin_e * pm_alpha;

    float phase_err =
        cross * obs->inv_psi_f;

    phase_err *= obs->pll_phase_sign;

    phase_err =
        NFO_ClampFloat(
            phase_err,
            -obs->pll_phase_err_limit,
            obs->pll_phase_err_limit);

    obs->pll_phase_err = phase_err;

    /*
     * 7. PLL PI
     *
     * 本拍算出的omega_raw，不再立即用于推进本拍角度，
     * 而是保存下来，下一次调用时再推进角度。
     */
    obs->pll_integrator +=
        obs->pll_ki *
        phase_err *
        obs->Ts;

    obs->pll_integrator =
        NFO_ClampFloat(
            obs->pll_integrator,
            -obs->pll_omega_limit,
            obs->pll_omega_limit);

    float omega_raw =
        obs->pll_kp * phase_err +
        obs->pll_integrator;

    omega_raw =
        NFO_ClampFloat(
            omega_raw,
            -obs->pll_omega_limit,
            obs->pll_omega_limit);

    obs->omega_e_raw = omega_raw;

    /*
     * 速度低通。
     */
    obs->omega_e +=
        obs->speed_lpf_alpha *
        (obs->omega_e_raw - obs->omega_e);

    /*
     * 8. 输出当前采样时刻的PLL角度。
     *
     * 这里不再执行：
     * theta_acc += omega_raw * Ts
     *
     * 也不再进行第二次CORDIC计算。
     */
    obs->theta_e_raw =
        NFO_Q32ToRad(obs->theta_acc);

    obs->theta_e =
        NFO_Q32ToRad(
            obs->theta_acc +
            obs->theta_offset_acc);

    return obs->theta_e;
}
float NFO_UpdateFromDuty(NFO_Handle_t *obs, float duty_a, float duty_b, float duty_c, float vbus, float i_alpha, float i_beta)
{
    /*
     * 三相 duty 转 alpha-beta 电压。
     *
     * duty_a/b/c 是三相桥臂对负母线的占空比。
     * 先去掉共模，再做 Clarke。
     *
     * v_alpha = Vbus * (2*da - db - dc) / 3
     * v_beta  = Vbus * (db - dc) / sqrt(3)
     */
    float v_alpha = vbus * (2.0f * duty_a - duty_b - duty_c) * 0.333333333333f;
    float v_beta = vbus * (duty_b - duty_c) * NFO_INV_SQRT3;

    return NFO_Update(obs, v_alpha, v_beta, i_alpha, i_beta);
}

float NFO_GetTheta(const NFO_Handle_t *obs)
{
    if (obs == 0)
    {
        return 0.0f;
    }

    return obs->theta_e;
}

float NFO_GetOmega(const NFO_Handle_t *obs)
{
    if (obs == 0)
    {
        return 0.0f;
    }

    return obs->omega_e;
}
uint16_t NFO_GetThetaU16(const NFO_Handle_t *obs)
{
    if (obs == 0)
    {
        return 0u;
    }

    return (uint16_t)((obs->theta_acc + obs->theta_offset_acc) >> 16);
}

uint16_t NFO_GetTheta14bit(const NFO_Handle_t *obs)
{
    if (obs == 0)
    {
        return 0u;
    }

    return (uint16_t)(((obs->theta_acc + obs->theta_offset_acc) >> 18) & 0x3FFFu);
}
