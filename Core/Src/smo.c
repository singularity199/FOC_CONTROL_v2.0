#include "smo.h"

#include <math.h>
#include <stddef.h>

#define SMO_PI 3.14159265358979323846f
#define SMO_TWO_PI 6.28318530717958647692f

#define SMO_MIN_LS 1.0e-8f
#define SMO_MIN_BOUNDARY 1.0e-6f
#define SMO_MIN_FREQUENCY 0.01f

static float SMO_Clamp(
    float value,
    float minimum,
    float maximum)
{
    if (value > maximum)
    {
        return maximum;
    }

    if (value < minimum)
    {
        return minimum;
    }

    return value;
}

static float SMO_WrapPi(float angle)
{
    while (angle > SMO_PI)
    {
        angle -= SMO_TWO_PI;
    }

    while (angle < -SMO_PI)
    {
        angle += SMO_TWO_PI;
    }

    return angle;
}

static float SMO_CalculateLPFCoefficient(
    float cutoff_hz,
    float ts_s)
{
    if ((cutoff_hz <= 0.0f) ||
        (ts_s <= 0.0f))
    {
        return 0.0f;
    }

    /*
     * First-order discrete low-pass filter:
     *
     * y[k] = y[k-1] + alpha * (x[k] - y[k-1])
     *
     * alpha = 1 - exp(-2*pi*fc*Ts)
     */
    return 1.0f -
           expf(
               -SMO_TWO_PI *
               cutoff_hz *
               ts_s);
}

void SMO_Init(
    SMO_t *smo,
    SMO_Config_t *config)
{
    if ((smo == NULL) ||
        (config == NULL))
    {
        return;
    }

    smo->config = *config;

    if (smo->config.ls_h < SMO_MIN_LS)
    {
        smo->config.ls_h = SMO_MIN_LS;
    }

    if (smo->config.boundary_a < SMO_MIN_BOUNDARY)
    {
        smo->config.boundary_a =
            SMO_MIN_BOUNDARY;
    }

    if (smo->config.emf_lpf_hz <
        SMO_MIN_FREQUENCY)
    {
        smo->config.emf_lpf_hz =
            SMO_MIN_FREQUENCY;
    }

    if (smo->config.speed_lpf_hz <
        SMO_MIN_FREQUENCY)
    {
        smo->config.speed_lpf_hz =
            SMO_MIN_FREQUENCY;
    }

    if (smo->config.direction >= 0)
    {
        smo->config.direction = 1;
    }
    else
    {
        smo->config.direction = -1;
    }

    smo->emf_lpf_coefficient =
        SMO_CalculateLPFCoefficient(
            smo->config.emf_lpf_hz,
            smo->config.ts_s);

    smo->speed_lpf_coefficient =
        SMO_CalculateLPFCoefficient(
            smo->config.speed_lpf_hz,
            smo->config.ts_s);

    smo->emf_lpf_omega =
        SMO_TWO_PI *
        smo->config.emf_lpf_hz;

    SMO_Reset(
        smo,
        0.0f,
        0.0f,
        0.0f);
}

void SMO_Reset(
    SMO_t *smo,
    float i_alpha,
    float i_beta,
    float initial_theta_e)
{
    if (smo == NULL)
    {
        return;
    }

    smo->i_hat_alpha = i_alpha;
    smo->i_hat_beta = i_beta;

    smo->i_error_alpha = 0.0f;
    smo->i_error_beta = 0.0f;

    smo->z_alpha = 0.0f;
    smo->z_beta = 0.0f;

    smo->emf_alpha = 0.0f;
    smo->emf_beta = 0.0f;
    smo->emf_magnitude = 0.0f;

    smo->theta_raw =
        SMO_WrapPi(initial_theta_e);

    smo->theta_compensated =
        SMO_WrapPi(initial_theta_e);

    smo->omega_e = 0.0f;
    smo->omega_e_instant = 0.0f;
    smo->phase_compensation = 0.0f;

    smo->last_theta_raw =
        SMO_WrapPi(initial_theta_e);

    smo->angle_valid = 0U;
}

void SMO_Update(
    SMO_t *smo,
    float v_alpha,
    float v_beta,
    float i_alpha,
    float i_beta)
{
    float saturation_alpha;
    float saturation_beta;

    float di_hat_alpha;
    float di_hat_beta;

    float theta_emf;
    float delta_theta;
    float omega_limit;

    if (smo == NULL)
    {
        return;
    }

    if ((smo->config.ts_s <= 0.0f) ||
        (smo->config.ls_h < SMO_MIN_LS))
    {
        return;
    }

    /*
     * Current estimation error:
     *
     * error = estimated current - measured current
     */
    smo->i_error_alpha =
        smo->i_hat_alpha -
        i_alpha;

    smo->i_error_beta =
        smo->i_hat_beta -
        i_beta;

    /*
     * Saturation boundary layer.
     *
     * sat(x):
     *
     *  x > 1  ->  1
     *  x < -1 -> -1
     *  else   ->  x
     */
    saturation_alpha =
        SMO_Clamp(
            smo->i_error_alpha /
                smo->config.boundary_a,
            -1.0f,
            1.0f);

    saturation_beta =
        SMO_Clamp(
            smo->i_error_beta /
                smo->config.boundary_a,
            -1.0f,
            1.0f);

    /*
     * Sliding-mode switching output.
     *
     * Unit: V.
     */
    smo->z_alpha =
        smo->config.k_slide_v *
        saturation_alpha;

    smo->z_beta =
        smo->config.k_slide_v *
        saturation_beta;

    /*
     * PMSM current observer:
     *
     * d(i_hat)/dt =
     * (v - Rs*i_hat - z) / Ls
     */
    di_hat_alpha =
        (v_alpha -
         smo->config.rs_ohm *
             smo->i_hat_alpha -
         smo->z_alpha) /
        smo->config.ls_h;

    di_hat_beta =
        (v_beta -
         smo->config.rs_ohm *
             smo->i_hat_beta -
         smo->z_beta) /
        smo->config.ls_h;

    /*
     * Forward-Euler discretization.
     */
    smo->i_hat_alpha +=
        smo->config.ts_s *
        di_hat_alpha;

    smo->i_hat_beta +=
        smo->config.ts_s *
        di_hat_beta;

    /*
     * On the sliding surface:
     *
     * z_alpha ~= e_alpha
     * z_beta  ~= e_beta
     *
     * Filter z to remove switching noise.
     */
    smo->emf_alpha +=
        smo->emf_lpf_coefficient *
        (smo->z_alpha -
         smo->emf_alpha);

    smo->emf_beta +=
        smo->emf_lpf_coefficient *
        (smo->z_beta -
         smo->emf_beta);

    smo->emf_magnitude =
        sqrtf(
            smo->emf_alpha *
                smo->emf_alpha +
            smo->emf_beta *
                smo->emf_beta);

    /*
     * Under the coordinate convention used here:
     *
     * e_alpha =
     * -omega_e * psi_f * sin(theta_e)
     *
     * e_beta =
     * omega_e * psi_f * cos(theta_e)
     *
     * Therefore:
     *
     * theta_e =
     * atan2(-e_alpha, e_beta)
     */
    theta_emf =
        atan2f(
            -smo->emf_alpha,
            smo->emf_beta);

    /*
     * When rotating in the negative direction,
     * the back-EMF vector reverses by pi.
     */
    if (smo->config.direction < 0)
    {
        theta_emf -= SMO_PI;
    }

    theta_emf +=
        smo->config.theta_offset_rad;

    smo->theta_raw =
        SMO_WrapPi(theta_emf);

    /*
     * At zero or very low speed, back-EMF
     * is too small to provide a reliable angle.
     */
    if (smo->emf_magnitude <
        smo->config.emf_valid_v)
    {
        smo->angle_valid = 0U;
        smo->omega_e_instant = 0.0f;

        return;
    }

    /*
     * First valid angle sample.
     *
     * Do not calculate speed from an invalid
     * previous angle.
     */
    if (smo->angle_valid == 0U)
    {
        smo->last_theta_raw =
            smo->theta_raw;

        smo->theta_compensated =
            smo->theta_raw;

        smo->omega_e = 0.0f;
        smo->omega_e_instant = 0.0f;
        smo->phase_compensation = 0.0f;

        smo->angle_valid = 1U;

        return;
    }

    delta_theta =
        SMO_WrapPi(
            smo->theta_raw -
            smo->last_theta_raw);

    smo->omega_e_instant =
        delta_theta /
        smo->config.ts_s;

    /*
     * Limit the maximum angle change to
     * approximately 0.9*pi rad per sample.
     *
     * This prevents a single noisy atan2 result
     * from producing an extreme speed spike.
     */
    omega_limit =
        0.9f *
        SMO_PI /
        smo->config.ts_s;

    smo->omega_e_instant =
        SMO_Clamp(
            smo->omega_e_instant,
            -omega_limit,
            omega_limit);

    /*
     * Electrical-speed low-pass filter.
     */
    smo->omega_e +=
        smo->speed_lpf_coefficient *
        (smo->omega_e_instant -
         smo->omega_e);

    /*
     * The first-order back-EMF LPF introduces:
     *
     * phase_lag =
     * atan(omega_e / omega_cutoff)
     *
     * Add the estimated phase lag back
     * to the raw angle.
     */
    smo->phase_compensation =
        atanf(
            smo->omega_e /
            smo->emf_lpf_omega);

    smo->theta_compensated =
        SMO_WrapPi(
            smo->theta_raw +
            smo->phase_compensation);

    smo->last_theta_raw =
        smo->theta_raw;
}

void SMO_SetMotorParameters(
    SMO_t *smo,
    float rs_ohm,
    float ls_h)
{
    if (smo == NULL)
    {
        return;
    }

    smo->config.rs_ohm =
        rs_ohm;

    if (ls_h < SMO_MIN_LS)
    {
        ls_h = SMO_MIN_LS;
    }

    smo->config.ls_h =
        ls_h;
}

void SMO_SetSlidingGain(
    SMO_t *smo,
    float k_slide_v)
{
    if (smo == NULL)
    {
        return;
    }

    if (k_slide_v < 0.0f)
    {
        k_slide_v =
            -k_slide_v;
    }

    smo->config.k_slide_v =
        k_slide_v;
}

void SMO_SetDirection(
    SMO_t *smo,
    int8_t direction)
{
    if (smo == NULL)
    {
        return;
    }

    if (direction >= 0)
    {
        smo->config.direction = 1;
    }
    else
    {
        smo->config.direction = -1;
    }
}
