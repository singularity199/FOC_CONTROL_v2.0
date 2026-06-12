#include "oled.h"
#include "oledfont.h"
#include <stdio.h>
#include <stdarg.h>
#include <string.h>
static uint8_t OLED_GRAM[8][128];
static uint8_t OLED_Dirty[8];
/**
 * @brief  OLED 屏幕初始化 (必须在主循环前调用)
 * @note   初始化完成后，屏幕将显示空白
 * @param  无
 * @return 无
 */
void OLED_Init(void)
{
    // 注意：建议在上电后延时 100ms 左右再调用此函数，等待屏幕硬件复位稳定

    OLED_WriteCmd(0xAE); // 关闭显示 (Display OFF)

    OLED_WriteCmd(0xD5); // 设置显示时钟分频比/振荡器频率
    OLED_WriteCmd(0x80); // 默认值

    OLED_WriteCmd(0xA8); // 设置多路复用率 (Multiplex Ratio)
    OLED_WriteCmd(0x3F); // 128x64 屏幕为 0x3F (即 64-1)

    OLED_WriteCmd(0xD3); // 设置显示偏移 (Display Offset)
    OLED_WriteCmd(0x00); // 默认值为 0

    OLED_WriteCmd(0x40); // 设置显示开始行 (Start Line) 0x40~0x7F

    OLED_WriteCmd(0x8D); // 设置电荷泵 (Charge Pump)
    OLED_WriteCmd(0x14); // 开启电荷泵 (0x10 为关闭，0x14 为开启，必须开启才能亮)

    OLED_WriteCmd(0x20); // 设置内存寻址模式 (Memory Addressing Mode)
    OLED_WriteCmd(0x02); // 0x02: 页寻址模式 (Page Addressing Mode)

    OLED_WriteCmd(0xA1); // 设置段重映射 (Segment Remap) -> 左右反转：0xA0正常，0xA1反转
    OLED_WriteCmd(0xC8); // 设置 COM 输出扫描方向 -> 上下反转：0xC0正常，0xC8反转

    OLED_WriteCmd(0xDA); // 设置 COM 引脚硬件配置
    OLED_WriteCmd(0x12);

    OLED_WriteCmd(0x81); // 设置对比度 (Contrast Control)
    OLED_WriteCmd(0xCF); // 范围 0x00~0xFF，越大越亮

    OLED_WriteCmd(0xD9); // 设置预充电周期 (Pre-charge Period)
    OLED_WriteCmd(0xF1);

    OLED_WriteCmd(0xDB); // 设置 VCOMH 取消选择级别
    OLED_WriteCmd(0x40);

    OLED_WriteCmd(0xA4); // 全局显示开启 (0xA4: 恢复 RAM 内容显示，0xA5: 忽略 RAM 强制全亮)
    OLED_WriteCmd(0xA6); // 设置正常/反色显示 (0xA6: 正常，0xA7: 反色)

    OLED_Clear(); // 初始化完成后，先清空显存中的随机乱码

    OLED_WriteCmd(0xAF); // 最后，开启显示 (Display ON)
}
void OLED_WriteCmd(uint8_t cmd)
{
    /* 1. 告诉硬件：设备地址，7位寻址，共发2个字节，发完自动STOP，开始发送(START) */
    LL_I2C_HandleTransfer(I2C1, OLED_I2C_ADDR, LL_I2C_ADDRSLAVE_7BIT, 2, LL_I2C_MODE_AUTOEND, LL_I2C_GENERATE_START_WRITE);

    /* 2. 等待发送中断状态标志位 (TXIS) 置位，表示移位寄存器空了，可以塞数据 */
    while (!LL_I2C_IsActiveFlag_TXIS(I2C1))
        ;
    LL_I2C_TransmitData8(I2C1, 0x00); // 发送命令控制字 0x00

    /* 3. 等待再次可以塞数据 */
    while (!LL_I2C_IsActiveFlag_TXIS(I2C1))
        ;
    LL_I2C_TransmitData8(I2C1, cmd); // 发送实际命令

    /* 4. 等待硬件自动产生的停止信号 (STOPF) */
    while (!LL_I2C_IsActiveFlag_STOP(I2C1))
        ;
    LL_I2C_ClearFlag_STOP(I2C1); // 必须手动清除 STOP 标志位
}
void OLED_WriteData(uint8_t data)
{
    /* 逻辑与发命令完全一致，只需将控制字改为 0x40 */
    LL_I2C_HandleTransfer(I2C1, OLED_I2C_ADDR, LL_I2C_ADDRSLAVE_7BIT, 2, LL_I2C_MODE_AUTOEND, LL_I2C_GENERATE_START_WRITE);

    while (!LL_I2C_IsActiveFlag_TXIS(I2C1))
        ;
    LL_I2C_TransmitData8(I2C1, 0x40);

    while (!LL_I2C_IsActiveFlag_TXIS(I2C1))
        ;
    LL_I2C_TransmitData8(I2C1, data);

    while (!LL_I2C_IsActiveFlag_STOP(I2C1))
        ;
    LL_I2C_ClearFlag_STOP(I2C1);
}
void OLED_SetCursor(uint8_t x, uint8_t y)
{
    x += 2;
    // 1. 设置页地址 (Page Address)：0xB0 ~ 0xB7
    OLED_WriteCmd(0xB0 | y);

    // 2. 设置列地址的低 4 位 (Lower Column Address)：0x00 ~ 0x0F
    OLED_WriteCmd(0x00 | (x & 0x0F));

    // 3. 设置列地址的高 4 位 (Higher Column Address)：0x10 ~ 0x1F
    OLED_WriteCmd(0x10 | ((x & 0xF0) >> 4));
}

/**
 * @brief 清除oled缓存
 *
 */
void OLED_Clear(void)
{
    memset(OLED_GRAM, 0, sizeof(OLED_GRAM));
    for (int i = 0; i < 8; i++)
    {
        OLED_Dirty[i] = 1;
    }
}

/**
 * @brief  设置 OLED 光标位置 (坐标定位)
 * @param  x: X 轴坐标，范围 0 ~ 127
 * @param  y: Y 轴坐标 (页)，范围 0 ~ 7 (对于 128x64 屏幕)
 */

void OLED_DrawChar(uint8_t x, uint8_t y, char chr)
{
    uint8_t c = chr - ' ';
    for (uint8_t i = 0; i < 8; i++)
    {
        OLED_GRAM[y][x + i] = OLED_F8x16[c][i];
        OLED_GRAM[y + 1][x + i] = OLED_F8x16[c][i + 8];
    }
    OLED_Dirty[y] = 1;
    OLED_Dirty[y + 1] = 1;
}
void OLED_ShowChar(uint8_t x, uint8_t y, char chr)
{
    uint8_t i;
    uint8_t c = chr - ' '; // 计算该字符在字库数组中的偏移量 (空格的 ASCII 码是 32)

    OLED_SetCursor(x, y); // 调用坐标定位函数，设置上半部分的绘制位置
    for (i = 0; i < 8; i++)
    {
        OLED_WriteData(OLED_F8x16[c][i]); // 发送上半部分的 8 列像素数据
    }

    OLED_SetCursor(x, y + 1); // 坐标下移一页，设置下半部分的绘制位置
    for (i = 0; i < 8; i++)
    {
        OLED_WriteData(OLED_F8x16[c][i + 8]); // 发送下半部分的 8 列像素数据
    }
}

/**
 * @brief  在 OLED 上显示一段 ASCII 字符串
 * @param  x: 起始 X 坐标
 * @param  y: 起始 Y 坐标 (页)
 * @param  str: 要显示的字符串指针
 */
void OLED_ShowStr(uint8_t x, uint8_t y, char *str)
{
    uint8_t top_buf[128]; // 用于暂存上半行的数据
    uint8_t bot_buf[128]; // 用于暂存下半行的数据
    uint8_t len = 0;

    // 1. 将字符串的像素点阵预先拼装到缓冲区中 (最多处理 16 个字符以防越界)
    while (*str != '\0' && len < 16 && (x + len * 8) <= 128)
    {
        uint8_t c = *str - ' ';
        for (uint8_t i = 0; i < 8; i++)
        {
            top_buf[len * 8 + i] = OLED_F8x16[c][i];     // 拼装上半部分
            bot_buf[len * 8 + i] = OLED_F8x16[c][i + 8]; // 拼装下半部分
        }
        len++;
        str++;
    }

    if (len == 0)
        return;

    // 2. 爆发写入上半部分 (只需 1 次 I2C 通讯)
    OLED_SetCursor(x, y);
    OLED_WriteDataBlock(top_buf, len * 8);

    // 3. 爆发写入下半部分 (只需 1 次 I2C 通讯)
    OLED_SetCursor(x, y + 1);
    OLED_WriteDataBlock(bot_buf, len * 8);
}

/**
 * @brief  [新增] 高性能：连续爆发发送数据块 (极大降低 CPU 占用)
 * @param  data: 数据数组指针
 * @param  size: 数据长度 (最多 254)
 */
void OLED_WriteDataBlock(uint8_t *data, uint8_t size)
{
    // size 个数据 + 1 个控制字节(0x40)，总共发 size+1 个字节
    LL_I2C_HandleTransfer(I2C1, OLED_I2C_ADDR, LL_I2C_ADDRSLAVE_7BIT, size + 1, LL_I2C_MODE_AUTOEND, LL_I2C_GENERATE_START_WRITE);

    while (!LL_I2C_IsActiveFlag_TXIS(I2C1))
        ;
    LL_I2C_TransmitData8(I2C1, 0x40); // 发送“写入数据”的控制字

    // 连续将数据塞入移位寄存器
    for (uint8_t i = 0; i < size; i++)
    {
        while (!LL_I2C_IsActiveFlag_TXIS(I2C1))
            ;
        LL_I2C_TransmitData8(I2C1, data[i]);
    }

    // 等待全部发完并产生 STOP
    while (!LL_I2C_IsActiveFlag_STOP(I2C1))
        ;
    LL_I2C_ClearFlag_STOP(I2C1);
}

/**
 * @brief  [重构] 高性能字符串显示 (将原本的碎片化发送整合为按块发送)
 */
void OLED_ShowString(uint8_t x, uint8_t y, char *str)
{
    while (*str)
    {
        OLED_DrawChar(x, y, *str);
        x += 8;
        str++;
    }
}

/**
 * @brief  [新增] 终极武器：直接打印变量 (用法和 printf 一模一样)
 * @param  x: 起始横坐标
 * @param  y: 起始页坐标
 * @param  format: 格式化字符串，例如 "Spd: %d"
 */
void OLED_Printf(uint8_t x, uint8_t y, const char *format, ...)
{
    char buf[32]; // 字符缓冲区
    va_list args; // 变参列表

    va_start(args, format);
    // 将变量格式化并填入 buf 中
    vsnprintf(buf, sizeof(buf), format, args);
    va_end(args);

    // 调用高性能字符串显示函数
    OLED_ShowString(x, y, buf);
}

void OLED_Update(void)
{
    for (uint8_t page = 0; page < 8; page++)
    {
        if (!OLED_Dirty[page])
            continue;

        OLED_Dirty[page] = 0;

        OLED_SetCursor(0, page);

        OLED_WriteDataBlock(OLED_GRAM[page], 128);
    }
}
