#include "s_curve.h"
#include "main.h"

static float s_curve_lut[SCURVE_LUT_SIZE];
static uint8_t s_curve_current_k = 0;
static uint8_t s_curve_lut_ready = 0;
SCurveTrajectory_t speed_curve;

/**
 * @brief 计算无符号整数的m次方
 * @param base 基础值
 * @param exponent 指数
 * @return float 结果
 */
static float SCurve_PowUInt(float base, uint8_t exponent)
{
    float result = 1.0f;

    while (exponent > 0)
    {
        result *= base;
        exponent--;
    }

    return result;
}

/**
 * @brief 计算浮点数的绝对值
 * @param value 输入值
 * @return float 绝对值
 */
static float SCurve_AbsFloat(float value)
{
    if (value >= 0.0f)
    {
        return value;
    }
    else
    {
        return -value;
    }
}

/**
 * @brief 限制浮点数在指定范围内
 * @param value 输入值
 * @param minimum 最小值
 * @param maximum 最大值
 * @return float 限制后的值
 */
static float SCurve_ClampFloat(float value, float minimum, float maximum)
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
/**
 * @brief
 *
 * @param shape_k
 */
void SCurve_BuildLUT(uint8_t shape_k)
{
    if (shape_k < 2)
    {
        shape_k = 2;
    }
    else if (shape_k > 6)
    {
        shape_k = 6;
    }

    s_curve_current_k = shape_k;

    for (uint16_t i = 0; i < SCURVE_LUT_SIZE; i++)
    {
        float u = (float)i / (float)(SCURVE_LUT_SIZE - 1);

        if (i == 0)
        {
            s_curve_lut[i] = 0.0f;
        }
        else if (i == SCURVE_LUT_SIZE - 1)
        {
            s_curve_lut[i] = 1.0f;
        }
        else
        {
            /*
             * 参数化 S 曲线：
             *
             * S(u) = u^k / (u^k + (1-u)^k)
             *
             * k 越大，两端越平缓，中间越陡。
             */
            float left = SCurve_PowUInt(u, shape_k);
            float right = SCurve_PowUInt(1.0f - u, shape_k);

            s_curve_lut[i] = left / (left + right);
        }
    }

    s_curve_lut_ready = 1;
}

static float SCurve_Lookup(float u)
{
    u = SCurve_ClampFloat(u, 0.0f, 1.0f);

    float position = u * (float)(SCURVE_LUT_SIZE - 1);
    uint16_t index = (uint16_t)position;
    float fraction = position - (float)index;

    if (index >= SCURVE_LUT_SIZE - 1)
    {
        return s_curve_lut[SCURVE_LUT_SIZE - 1];
    }

    float y0 = s_curve_lut[index];
    float y1 = s_curve_lut[index + 1];

    return y0 + fraction * (y1 - y0);
}

void SCurve_Init(SCurveTrajectory_t *traj, float initial_speed, float control_period, float accel_max, uint8_t shape_k)
{
    if (traj == 0)
    {
        return;
    }

    if (control_period <= 0.0f)
    {
        control_period = 0.001f;
    }

    if (accel_max <= 0.0f)
    {
        accel_max = 1000.0f;
    }

    if (shape_k < 2)
    {
        shape_k = 2;
    }
    else if (shape_k > 6)
    {
        shape_k = 6;
    }

    traj->start_speed = initial_speed;
    traj->target_speed = initial_speed;
    traj->speed_ref = initial_speed;

    traj->elapsed_time = 0.0f;
    traj->total_time = 0.0f;
    traj->control_period = control_period;

    traj->accel_max = accel_max;
    traj->shape_k = shape_k;

    traj->command_deadband = 1.0f;
    traj->running = 0;

    SCurve_BuildLUT(shape_k);
}

void SCurve_SetTarget(SCurveTrajectory_t *traj, float target_speed)
{
    if (traj == 0)
    {
        return;
    }

    /*
     * 目标变化太小，不重新规划。
     * 防止旋钮/通信噪声反复清零 elapsed_time。
     */
    if (SCurve_AbsFloat(target_speed - traj->target_speed) <= traj->command_deadband)
    {
        return;
    }

    /*
     * 新轨迹从当前规划速度开始，而不是从实际反馈速度开始。
     * 这样 speed_ref 不会发生阶跃。
     */
    traj->start_speed = traj->speed_ref;
    traj->target_speed = target_speed;
    traj->elapsed_time = 0.0f;

    float delta_speed = traj->target_speed - traj->start_speed;

    if (SCurve_AbsFloat(delta_speed) <= traj->command_deadband)
    {
        traj->speed_ref = traj->target_speed;
        traj->running = 0;
        traj->total_time = 0.0f;
        return;
    }

    /*
     * 对于曲线：
     *
     * S_k(u) = u^k / (u^k + (1-u)^k)
     *
     * 在 u = 0.5 附近最大斜率约为 k。
     *
     * speed_ref = start + delta * S(u)
     *
     * 所以最大加速度近似：
     *
     * a_max = |delta_speed| * k / total_time
     *
     * 反推：
     *
     * total_time = |delta_speed| * k / accel_max
     */
    traj->total_time =
        SCurve_AbsFloat(delta_speed) *
        traj->shape_k /
        traj->accel_max;

    /*
     * 轨迹时间至少一个速度环周期。
     */
    if (traj->total_time < traj->control_period)
    {
        traj->total_time = traj->control_period;
    }

    traj->running = 1;
}

float SCurve_Update(SCurveTrajectory_t *traj)
{
    if (traj == 0)
    {
        return 0.0f;
    }

    if (!s_curve_lut_ready)
    {
        SCurve_BuildLUT(traj->shape_k);
    }

    if (!traj->running)
    {
        return traj->speed_ref;
    }

    traj->elapsed_time += traj->control_period;

    float u = traj->elapsed_time / traj->total_time;

    if (u >= 1.0f)
    {
        traj->speed_ref = traj->target_speed;
        traj->elapsed_time = traj->total_time;
        traj->running = 0;

        return traj->speed_ref;
    }

    float curve_value = SCurve_Lookup(u);

    traj->speed_ref =
        traj->start_speed +
        (traj->target_speed - traj->start_speed) *
            curve_value;

    return traj->speed_ref;
}

void SCurve_SetParam(SCurveTrajectory_t *traj, float accel_max, uint8_t shape_k)
{
    if (traj == 0)
    {
        return;
    }
    if (traj->running)
    {
        return;
    }

    if (accel_max > 0.0f)
    {
        traj->accel_max = accel_max;
    }

    if (shape_k < 2)
    {
        shape_k = 2;
    }
    else if (shape_k > 6)
    {
        shape_k = 6;
    }

    traj->shape_k = shape_k;

    /*
     * 只有曲线参数变化时才重建 LUT。
     */
    if (shape_k != s_curve_current_k)
    {
        SCurve_BuildLUT(shape_k);
    }
}

uint8_t SCurve_IsRunning(const SCurveTrajectory_t *traj)
{
    if (traj == 0)
    {
        return 0;
    }

    return traj->running;
}

float SCurve_GetSpeedRef(const SCurveTrajectory_t *traj)
{
    if (traj == 0)
    {
        return 0.0f;
    }

    return traj->speed_ref;
}