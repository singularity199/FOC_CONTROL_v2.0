// clang-format off
#ifndef S_CURVE_H
#define S_CURVE_H

#include <stdint.h>

#ifdef __cplusplus
extern "C" {
#endif

#ifndef SCURVE_LUT_SIZE
#define SCURVE_LUT_SIZE 257
#endif

typedef struct
{
    float start_speed;       // 本次轨迹起点速度
    float target_speed;      // 本次轨迹目标速度
    float speed_ref;         // 当前输出速度参考值

    float elapsed_time;      // 已运行时间
    float total_time;        // 本次轨迹总时间
    float control_period;    // 速度环周期

    float accel_max;         // 最大加速度，单位 rpm/s 或 rad/s^2
    uint8_t shape_k;           // S 曲线形状参数，建议 2.0 ~ 5.0

    float command_deadband;  // 目标速度变化死区
    uint8_t running;         // 轨迹是否正在运行
} SCurveTrajectory_t;

extern SCurveTrajectory_t speed_curve;

void SCurve_BuildLUT(uint8_t shape_k);

void SCurve_Init(SCurveTrajectory_t *traj,
                 float initial_speed,
                 float control_period,
                 float accel_max,
                 uint8_t shape_k);

void SCurve_SetTarget(SCurveTrajectory_t *traj,
                      float target_speed);

float SCurve_Update(SCurveTrajectory_t *traj);

void SCurve_SetParam(SCurveTrajectory_t *traj,
                     float accel_max,
                     uint8_t shape_k);

uint8_t SCurve_IsRunning(const SCurveTrajectory_t *traj);

float SCurve_GetSpeedRef(const SCurveTrajectory_t *traj);




#ifdef __cplusplus
}
#endif

#endif