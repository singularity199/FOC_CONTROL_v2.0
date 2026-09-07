#ifndef _MOTOR_CONTROL_H
#define _MOTOR_CONTROL_H

#include "main.h"
#include "pid.h"
#include "spi.h"
#include "usart.h"
#include "oled.h"
#include "nonlinear_flux_observer.h"
#include "foc.h"
#include "Sguan_NLFO.h"
#include "smo.h"
typedef struct
{
    float target_velocity;  // 目标速度 (rad/s, 电角速度)
    float current_velocity; // 当前速度 (rad/s)
    float acceleration;     // 加速度 (rad/s^2)
    float virtual_angle;    // 虚拟电角度 (rad)
    float dt;               // 循环周期 (s)，例如 1/10000.0f (10kHz)
} OpenLoopPlanner;

extern foc_state foc;
extern SMO_t smo;
extern NFO_Handle_t nfo;
extern NLFO_STRUCT nlfo;
extern OpenLoopPlanner planner;
void set_high_phase(uint8_t phase);
void set_pwm_phase(uint8_t phase, uint16_t throttle);
void set_low_phase(uint8_t phase);
void motor_stop_all(void);
void disable_phase_output(uint8_t phase);
void motor_step(uint8_t step);

float get_angle_el(void);

void motor_smo_init(void);
void motor_init(void);
void foc_velocityloop_init(void);

#endif //
