#ifndef __CAN_H__
#define __CAN_H__
#include "main.h"
#include "fdcan.h"
// clang-format off

typedef struct
{
    volatile uint32_t rx_count;      // 成功接收计数
    volatile uint32_t rx_error;      // 读取失败计数
    volatile uint32_t last_dlc;      // 最近一帧 DLC
    volatile uint16_t last_id;       // 最近一帧 ID
    volatile uint8_t  last_data[8];  // 最近一帧数据
    volatile uint8_t  new_flag;      // 收到新数据标志
} CAN_RxDebugTypeDef;

extern CAN_RxDebugTypeDef can_rx_dbg;

#define CAN_FUNC_CURRENT    0x100      //0x100 + id ：电流 / 力矩模式
#define CAN_FUNC_VELOCITY   0x200      //0x200 + id ：速度模式
#define CAN_FUNC_POSITION   0x300      //0x300 + id ：位置模式
#define CAN_FUNC_FEEDBACK   0x400      //0x400 + id ：状态反馈
#define CAN_FUNC_PARAM      0x500      //0x500 + id ：参数设置
#define CAN_FUNC_ERROR      0x600      //0x600 + id ：错误码
#define CAN_FUNC_HEARTBEAT  0x700      //0x700 + id ：心跳

#define CAN_FUNC_MASK       0x7F0
#define CAN_ID_MASK         0x00F

void CAN_UserInit(void);
uint8_t CAN_SendStdData(uint16_t id, uint8_t data[8]);
void CAN_ParseFrame(uint16_t id, uint8_t data[8], uint32_t dlc);

void MotorCAN_ParseCurrentCmd(uint8_t motor_id, uint8_t data[8]);

#endif /* __CAN_H__ */
