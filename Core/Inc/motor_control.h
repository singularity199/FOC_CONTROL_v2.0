#ifndef _MOTOR_CONTROL_H
#define _MOTOR_CONTROL_H

#include "main.h"
#include "pid.h"

typedef struct
{
    float target_velocity;  // 目标速度 (rad/s, 电角速度)
    float current_velocity; // 当前速度 (rad/s)
    float acceleration;     // 加速度 (rad/s^2)
    float virtual_angle;    // 虚拟电角度 (rad)
    float dt;               // 循环周期 (s)，例如 1/10000.0f (10kHz)
} OpenLoopPlanner;

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

extern foc_state foc;
extern OpenLoopPlanner planner;
void set_high_phase(uint8_t phase);
void set_pwm_phase(uint8_t phase, uint16_t throttle);
void set_low_phase(uint8_t phase);
void motor_stop_all(void);
void disable_phase_output(uint8_t phase);
void motor_step(uint8_t step);
cordic_angle cor(float angle);
float get_angle_el(void);
T4T6_timeTypeDef T4T6_time(float v_alpha, float v_beta);
void foc_current_loop_control(float i_d, float i_q);
void motor_init(void);
void foc_velocityloop_init(void);

#endif //
