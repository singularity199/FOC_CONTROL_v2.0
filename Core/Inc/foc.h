#ifndef FOC_H
#define FOC_H
#include "main.h"
#include "pid.h"
typedef struct
{
    float i_a;
    float i_b;
    float i_c;
    uint32_t i_a_offset;
    uint32_t i_b_offset;
    uint32_t i_c_offset;
} adc_curr;

// alpha beta
typedef struct
{
    float alpha;
    float beta;
} clark_frame;

// d q
typedef struct
{
    float d;
    float q;
} park_frame;

typedef struct
{
    float sin_angle;
    float cos_angle;
} cordic_angle;

typedef struct
{
    float T4;
    float T6;
} T4T6_timeTypeDef;

typedef struct
{
    uint32_t tcm1;
    uint32_t tcm2;
    uint32_t tcm3;
} change_tickTypeDef;

typedef struct
{
    uint32_t ta;
    uint32_t tb;
    uint32_t tc;
} tabc_timeTypeDef;

typedef struct
{
    uint8_t sector_N;
    T4T6_timeTypeDef T4T6;
    tabc_timeTypeDef tabc_time;
    change_tickTypeDef change_tick;
} svpwm_status;

typedef struct
{
    uint16_t angle_machanine;
    float angle_el;
    uint32_t angle_error;
    uint32_t new_tick;
    uint32_t last_tick;
    float target_speed;
    float real_speed;
    adc_curr adc;
    clark_frame real_clark_i;
    park_frame real_park_i;
    park_frame target_park_i;
    clark_frame target_clark_v;
    park_frame target_park_v;
    PI_Controller pid_d;
    PI_Controller pid_q;
    PI_Controller pid_speed;
    svpwm_status svpwm;
} foc_state;

cordic_angle cor(float angle);
cordic_angle cor_q32(uint32_t theta_q32);
park_frame park_trans(float alpha, float beta, float angle_el);
T4T6_timeTypeDef T4T6_time(float v_alpha, float v_beta);
void svpwm_modulation(float v_alpha, float v_beta);
void foc_current_loop_control(float i_d, float i_q);

#endif // FOC_H