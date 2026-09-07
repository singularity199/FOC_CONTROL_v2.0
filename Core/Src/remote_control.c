#include "remote_control.h"
#include "motor_control.h"
#include <string.h>

#define REMOTE_LINE_SIZE 64

typedef enum
{
    REMOTE_KNOB_BIND_SPEED = 0,
    REMOTE_KNOB_BIND_IQ
} RemoteKnobBind;

static char rx_line[REMOTE_LINE_SIZE];
static volatile uint8_t rx_line_ready;

static char line_build[REMOTE_LINE_SIZE];
static char line_copy[REMOTE_LINE_SIZE];
static uint16_t line_index;
static uint8_t line_overflow;

static RemoteKnobBind knob_bind;
uint8_t speed_control_flag = 0;
uint8_t iq_control_flag = 0;

/**
 * @brief 将命令中的数字字符串转换为浮点数。
 * @param text 指向数字字符串起始位置的指针。
 * @return 解析得到的浮点数。
 *
 * 这里不用 strtof，是为了避免把较重的标准库浮点解析函数拉进固件。
 * 支持格式示例：10、-10、0.2、-0.2。
 */
static float remote_parse_float(const char *text)
{
    float value = 0.0f;
    float decimal = 0.1f;
    float sign = 1.0f;
    uint8_t after_dot = 0;

    while (*text == ' ')
    {
        text++;
    }

    if (*text == '-')
    {
        sign = -1.0f;
        text++;
    }
    else if (*text == '+')
    {
        text++;
    }

    while (*text != '\0')
    {
        if (*text == '.')
        {
            after_dot = 1;
            text++;
            continue;
        }

        if (*text < '0' || *text > '9')
        {
            break;
        }

        if (after_dot)
        {
            value += (float)(*text - '0') * decimal;
            decimal *= 0.1f;
        }
        else
        {
            value = value * 10.0f + (float)(*text - '0');
        }

        text++;
    }

    return sign * value;
}

/**
 * @brief 根据 rps 输入设置速度目标。
 * @param rps 目标机械转速，单位为转每秒。
 */
static void remote_set_speed_rps(float rps)
{
    foc.target_speed = rps;
    speed_control_flag = 1;
}

/**
 * @brief 设置 q 轴电流目标。
 * @param iq 目标 q 轴电流，单位为安培。
 */
static void remote_set_iq_amp(float iq)
{
    foc.target_park_i.q = iq;
    iq_control_flag = 1;
}

/**
 * @brief 初始化远程命令解析器的缓存和默认状态。
 */
void remote_control_init(void)
{
    memset(rx_line, 0, sizeof(rx_line));
    memset(line_build, 0, sizeof(line_build));
    memset(line_copy, 0, sizeof(line_copy));
    rx_line_ready = 0;
    line_index = 0;
    line_overflow = 0;
    knob_bind = REMOTE_KNOB_BIND_SPEED;
}

/**
 * @brief 将 USB CDC 收到的原始字节喂给命令行拼接器。
 * @param data 指向 CDC 接收回调传入的数据缓存。
 * @param len 收到的数据字节数。
 *
 * 这个函数设计为在 USB 接收回调中调用，只负责拼接以换行结尾的命令行，
 * 不在回调里解析命令，避免 USB 回调执行过重。
 */
void remote_control_rx_feed(uint8_t *data, uint32_t len)
{
    for (uint16_t i = 0; i < len; i++)
    {
        char c = (char)data[i];

        if (c == '\r')
        {
            continue;
        }

        if (c == '\n')
        {
            if (line_overflow == 0)
            {
                line_build[line_index] = '\0';
                strncpy(rx_line, line_build, REMOTE_LINE_SIZE - 1);
                rx_line[REMOTE_LINE_SIZE - 1] = '\0';
                rx_line_ready = 1;
            }

            line_index = 0;
            line_overflow = 0;
            memset(line_build, 0, sizeof(line_build));
            continue;
        }

        if (line_overflow)
        {
            continue;
        }

        if (line_index < (REMOTE_LINE_SIZE - 1))
        {
            line_build[line_index++] = c;
        }
        else
        {
            line_index = 0;
            line_overflow = 1;
            memset(line_build, 0, sizeof(line_build));
        }
    }
}

/**
 * @brief 解析一整行命令，并把命令作用到电机控制状态上。
 * @param line 不带换行符、以 '\0' 结尾的命令字符串。
 */
static void remote_parse_line(char *line)
{
    float value;

    /*
     * 速度命令短写：SPD 10
     * SPD 是 speed 的缩写，后面的参数表示目标机械转速，单位为 rps。
     * 例如 SPD 10 表示目标速度为每秒 10 转。
     */
    if (strncmp(line, "SPD ", 4) == 0)
    {
        value = remote_parse_float(&line[4]);
        remote_set_speed_rps(value);
        return;
    }

    /*
     * 速度命令完整写法：SPEED 10
     * SPEED 和 SPD 功能完全相同，只是名字更直观。
     * 保留两种写法是为了让上位机命令既可以简短，也可以更易读。
     */
    if (strncmp(line, "SPEED ", 6) == 0)
    {
        value = remote_parse_float(&line[6]);
        remote_set_speed_rps(value);
        return;
    }

    /*
     * q 轴电流命令：IQ 0.2
     * IQ 表示 FOC 中的 q 轴电流目标，后面的参数单位为 A。
     * 例如 IQ 0.2 表示目标 q 轴电流为 0.2 A。
     */
    if (strncmp(line, "IQ ", 3) == 0)
    {
        value = remote_parse_float(&line[3]);
        remote_set_iq_amp(value);
        return;
    }

    /*
     * 旋钮命令：K 10
     * K 表示 knob，也就是上位机旋钮、滑条或实时控件发送过来的数值。
     * K 后面的参数会根据当前绑定关系解释：
     * 默认或执行 BIND SPD 后，K 10 等价于 SPD 10；
     * 执行 BIND IQ 后，K 0.2 等价于 IQ 0.2。
     */
    if (strncmp(line, "K ", 2) == 0)
    {
        value = remote_parse_float(&line[2]);

        if (knob_bind == REMOTE_KNOB_BIND_IQ)
        {
            remote_set_iq_amp(value);
        }
        else
        {
            remote_set_speed_rps(value);
        }
        return;
    }

    /*
     * 停止命令：STOP
     * 清零速度目标和 q 轴电流目标，然后关闭三相输出。
     */
    if (strcmp(line, "STOP") == 0)
    {
        foc.target_speed = 0.0f;
        foc.target_park_i.q = 0.0f;
        motor_stop_all();
        return;
    }

    /*
     * 旋钮绑定到速度：BIND SPD 或 BIND SPEED
     * 绑定后，后续 K xxx 会把 xxx 当作速度目标，单位为 rps。
     */
    if (strcmp(line, "BIND SPD") == 0 || strcmp(line, "BIND SPEED") == 0)
    {
        knob_bind = REMOTE_KNOB_BIND_SPEED;
        return;
    }

    /*
     * 旋钮绑定到 q 轴电流：BIND IQ
     * 绑定后，后续 K xxx 会把 xxx 当作 q 轴电流目标，单位为 A。
     */
    if (strcmp(line, "BIND IQ") == 0)
    {
        knob_bind = REMOTE_KNOB_BIND_IQ;
        return;
    }
}

/**
 * @brief 处理最新收到的一整行命令。
 *
 * 建议在主循环或低优先级任务中调用。这里不关闭全局中断，避免影响 USB、
 * ADC、TIM 等中断响应。USB 回调只负责更新 rx_line 和 rx_line_ready，
 * 这里发现 rx_line_ready 置位后，把 rx_line 整块复制到 line_copy，
 * 再在回调之外解析 line_copy。line_copy 使用静态缓存，方便仿真时观察。
 */
void remote_control_task(void)
{
    if (rx_line_ready == 0)
    {
        return;
    }

    memcpy(line_copy, rx_line, REMOTE_LINE_SIZE);
    line_copy[REMOTE_LINE_SIZE - 1] = '\0';
    rx_line_ready = 0;

    remote_parse_line(line_copy);
}
