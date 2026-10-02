// clang-format off
#ifndef NONLINEAR_FLUX_OBSERVER_H
#define NONLINEAR_FLUX_OBSERVER_H

#include <stdint.h>

#ifdef __cplusplus
extern "C" {
#endif

typedef struct
{
    //电机参数
    float Rs;
    float Ls;
    float psi_f;
    float Ts;
    float k_nl;

    float theta_offset;

    //磁链积分状态
    float lambda_alpha;
    float lambda_beta;
    float emf_alpha_prev;
    float emf_beta_prev;
    uint8_t emf_prev_valid;
    /* 调试量 */
    float emf_mag2;
    float corr_mag2;
    float corr_emf_ratio;

    //永磁体磁链估计：pm = lambda - Ls * i
    float pm_alpha;
    float pm_beta;

    /*
     * 注意：
     * 这里不再使用 sqrt，所以 pm_abs 不是严格的 sqrt(pm2)。
     * 它只是一个近似调试值。
     * 真正建议观察 pm2。
     */
    float pm_abs;
    float pm2;

    /*
     * 输出角度和速度。
     *
     * theta_e_raw：PLL 原始角度，rad
     * theta_e    ：加 theta_offset 后的角度，rad
     */
    float theta_e;
    float theta_e_raw;
    float theta_prev;

    float omega_e;
    float omega_e_raw;

    float speed_lpf_alpha;

    //磁链幅值误差：err_flux = psi_f^2 - pm2
    float err_flux;

    //新增：避免重复计算
    float psi_f2;
    float inv_psi_f;

    //新增：PLL 状态
    float pll_kp;
    float pll_ki;
    float pll_integrator;
    float pll_phase_err;
    float pll_phase_err_limit;
    float pll_omega_limit;

    //PLL 内部角度单位向量。这个向量由小角度旋转递推，不调用 sin/cos。

    float cos_e;
    float sin_e;

    //角度累加器：0x00000000 ~ 0xFFFFFFFF 对应 0 ~ 2pi
    uint32_t theta_acc;
    uint32_t theta_offset_acc;

    //如果角度方向反了，把这个设成 -1。
    float pll_phase_sign;

    uint8_t initialized;

} NFO_Handle_t;


void NFO_Init(NFO_Handle_t *obs,float Rs,float Ls,float psi_f,float Ts,float k_nl);

void NFO_SetParam(NFO_Handle_t *obs,float Rs,float Ls,float psi_f,float Ts,float k_nl);

void NFO_SetPLL(NFO_Handle_t *obs, float kp, float ki);
void NFO_SetThetaOffset(NFO_Handle_t *obs, float theta_offset);

void NFO_SetSpeedFilter(NFO_Handle_t *obs, float alpha);
void NFO_SetPLLOmegaLimit(NFO_Handle_t *obs, float omega_limit);
/*
 * 用已知电角度重置观测器。
 * 启动阶段建议在预定位/开环角度已知时调用一次。
 */
void NFO_ResetAngle(NFO_Handle_t *obs,float theta_e,float i_alpha,float i_beta);

/*
 * 直接使用 alpha-beta 电压更新。
 * v_alpha / v_beta 必须是“实际作用到电机上的电压”，最好是限幅后的电压。
 */
float NFO_Update(NFO_Handle_t *obs,float v_alpha, float v_beta, float i_alpha, float i_beta);

/*
 * 如果你手里只有三相 PWM 占空比，可以用这个。
 * duty_a / duty_b / duty_c 范围 0.0f ~ 1.0f。
 * vbus 是母线电压。
 */
float NFO_UpdateFromDuty(NFO_Handle_t *obs,float duty_a,float duty_b,float duty_c,float vbus,float i_alpha,float i_beta);

float NFO_GetTheta(const NFO_Handle_t *obs);
float NFO_GetOmega(const NFO_Handle_t *obs);

/*
 * 转成 0~65535 的 uint16 电角度。
 * 如果你现在编码器角度内部用 16bit，这个比较方便。
 */
uint16_t NFO_GetThetaU16(const NFO_Handle_t *obs);

/*
 * 转成 0~16383 的 14bit 电角度。
 * 适合对齐你那个磁编码器 0~16383 的习惯。
 */
uint16_t NFO_GetTheta14bit(const NFO_Handle_t *obs);


extern NFO_Handle_t nfo;

#define NFO_PI                  3.14159265358979323846f
#define NFO_TWO_PI              6.28318530717958647692f
#define NFO_HALF_PI             1.57079632679489661923f
#define NFO_INV_SQRT3           0.57735026918962576451f
#define NFO_Q32_PER_RAD         683565275.5764316f
#define NFO_DEFAULT_PLL_KP      1500.0f
#define NFO_DEFAULT_PLL_KI      80000.0f
#define NFO_DEFAULT_OMEGA_LIMIT 30000.0f


#ifdef __cplusplus
}
#endif

#endif