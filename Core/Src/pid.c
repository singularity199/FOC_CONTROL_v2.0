#include "pid.h"
#include "main.h"
#include "motor_control.h"
void pid_current_init(void)
{
    // Q轴 PI初始化 (控制转矩)
    foc.pid_q.Kp = 0.5f; // 需要根据电机电感电阻实际整定
    foc.pid_q.Ki = 0.01f;
    foc.pid_q.error_sum = 0.0f;
    foc.pid_q.limit_out = 12.0f; // 假设母线电压12V，最大输出不能超过母线电压
    foc.pid_q.limit_int = 10.0f; // 积分限幅稍微比输出限幅小一点或相等

    // D轴 PI初始化 (控制励磁，通常目标为0)
    foc.pid_d.Kp = 0.5f;
    foc.pid_d.Ki = 0.01f;
    foc.pid_d.error_sum = 0.0f;
    foc.pid_d.limit_out = 12.0f;
    foc.pid_d.limit_int = 10.0f;
}

float PI_Calc(PI_Controller *pi, float target, float measure)
{
    float error = target - measure;

    // 1. 积分累加
    pi->error_sum += error;

    // 2. 积分抗饱和限幅 (核心保护逻辑)
    if (pi->error_sum > pi->limit_int)
        pi->error_sum = pi->limit_int;
    else if (pi->error_sum < -pi->limit_int)
        pi->error_sum = -pi->limit_int;

    // 3. 计算输出
    float output = (pi->Kp * error) + (pi->Ki * pi->error_sum);

    // 4. 总输出限幅
    if (output > pi->limit_out)
        output = pi->limit_out;
    else if (output < -pi->limit_out)
        output = -pi->limit_out;

    return output;
}
