#ifndef __SMO_H
#define __SMO_H

// clang-format off
#include <stdint.h>

#ifdef __cplusplus
extern "C" {
#endif

typedef struct
{
    /*
     * Motor phase resistance.
     * Unit: ohm.
     */
    float rs_ohm;

    /*
     * Motor phase inductance.
     * Unit: H.
     */
    float ls_h;

    /*
     * SMO execution period.
     * Unit: s.
     *
     * This should use your FOC_CONTROL_Ts.
     */
    float ts_s;

    /*
     * Sliding-mode gain.
     * Unit: V.
     */
    float k_slide_v;

    /*
     * Saturation boundary layer.
     * Unit: A.
     */
    float boundary_a;

    /*
     * Back-EMF LPF cutoff frequency.
     * Unit: Hz.
     */
    float emf_lpf_hz;

    /*
     * Electrical speed LPF cutoff frequency.
     * Unit: Hz.
     */
    float speed_lpf_hz;

    /*
     * Minimum back-EMF magnitude required
     * before the angle is considered valid.
     * Unit: V.
     */
    float emf_valid_v;

    /*
     * Fixed electrical-angle correction.
     * Unit: rad.
     */
    float theta_offset_rad;

    /*
     * Expected rotation direction.
     *
     * +1: positive direction
     * -1: negative direction
     */
    int8_t direction;

} SMO_Config_t;

typedef struct
{
    SMO_Config_t config;

    /*
     * Estimated alpha-beta currents.
     */
    float i_hat_alpha;
    float i_hat_beta;

    /*
     * Current estimation errors:
     *
     * i_error = i_hat - i_measured
     */
    float i_error_alpha;
    float i_error_beta;

    /*
     * Sliding-mode switching outputs.
     */
    float z_alpha;
    float z_beta;

    /*
     * Filtered back-EMF estimates.
     */
    float emf_alpha;
    float emf_beta;
    float emf_magnitude;

    /*
     * Electrical angles.
     *
     * theta_raw:
     * angle directly calculated from filtered back-EMF.
     *
     * theta_compensated:
     * theta_raw plus LPF phase compensation.
     */
    float theta_raw;
    float theta_compensated;

    /*
     * Estimated electrical angular speed.
     * Unit: rad/s.
     */
    float omega_e;

    /*
     * Unfiltered electrical angular speed.
     * Unit: rad/s.
     */
    float omega_e_instant;

    /*
     * Estimated LPF phase lag.
     * Unit: rad.
     */
    float phase_compensation;

    float last_theta_raw;

    /*
     * Internally calculated LPF coefficients.
     */
    float emf_lpf_coefficient;
    float speed_lpf_coefficient;
    float emf_lpf_omega;

    /*
     * 0: back-EMF is too small, angle invalid
     * 1: angle available
     */
    uint8_t angle_valid;

} SMO_t;

/*
 * Initialize SMO parameters and clear all states.
 */
void SMO_Init(    SMO_t *smo,    SMO_Config_t *config);

/*
 * Reset internal observer states.
 *
 * i_alpha, i_beta:
 * current measured alpha-beta current.
 *
 * initial_theta_e:
 * initial electrical angle in radians.
 */
void SMO_Reset(
    SMO_t *smo,
    float i_alpha,
    float i_beta,
    float initial_theta_e);

/*
 * Execute one SMO iteration.
 *
 * v_alpha, v_beta:
 * applied alpha-beta stator voltage, unit V.
 *
 * i_alpha, i_beta:
 * measured alpha-beta stator current, unit A.
 */
void SMO_Update(
    SMO_t *smo,
    float v_alpha,
    float v_beta,
    float i_alpha,
    float i_beta);

/*
 * Change motor parameters during operation.
 */
void SMO_SetMotorParameters(
    SMO_t *smo,
    float rs_ohm,
    float ls_h);

/*
 * Change sliding-mode gain during operation.
 */
void SMO_SetSlidingGain(
    SMO_t *smo,
    float k_slide_v);

/*
 * Set expected rotation direction.
 */
void SMO_SetDirection(
    SMO_t *smo,
    int8_t direction);

#ifdef __cplusplus
}
#endif

#endif