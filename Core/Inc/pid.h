#ifndef _PID_H
#define _PID_H

typedef struct
{
    float Kp; // 比例系数
    float Ki; // 积分系数
    float error;
    float output;
    float error_sum; // 误差积分累计
    float limit_out; // 输出限幅（最大电压）
    float limit_int; // 积分限幅（防止风速效应/爆冲）
} PI_Controller;

void PI_Init(PI_Controller *pi, float Kp, float Ki, float limit_out, float limit_int);
float PI_Calc(PI_Controller *pi, float target, float measure);

#endif // !_PID_H
