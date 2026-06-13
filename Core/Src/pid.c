#include "pid.h"
#include "main.h"
#include "motor_control.h"

void PI_Init(PI_Controller *pi, float Kp, float Ki, float limit_out, float limit_int)
{
    pi->Kp = Kp;
    pi->Ki = Ki;
    pi->error_sum = 0.0f;
    pi->limit_out = limit_out;
    pi->limit_int = limit_int;
}

float PI_Calc(PI_Controller *pi, float target, float measure)
{
    // float error = target - measure;
    pi->error = target - measure;

    // 1. 积分累加
    pi->error_sum += pi->Ki * pi->error;

    // 2. 积分抗饱和限幅 (核心保护逻辑)
    if (pi->error_sum > pi->limit_int)
        pi->error_sum = pi->limit_int;
    else if (pi->error_sum < -pi->limit_int)
        pi->error_sum = -pi->limit_int;

    // 3. 计算输出
    pi->output = (pi->Kp * pi->error) + pi->error_sum;

    // 4. 总输出限幅
    if (pi->output > pi->limit_out)
        pi->output = pi->limit_out;
    else if (pi->output < -pi->limit_out)
        pi->output = -pi->limit_out;

    return pi->output;
}
