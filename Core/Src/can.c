#include "can.h"
#include "fdcan.h"

static CanFrame rx_queue[CAN_RX_QUEUE_SIZE];
static volatile uint8_t rx_queue_head = 0;
static volatile uint8_t rx_queue_tail = 0;

static volatile uint32_t rx_count = 0;
static volatile uint32_t rx_drop_count = 0;
CAN_RxDebugTypeDef can_rx_dbg = {0};

void CAN_UserInit(void)
{
    FDCAN_FilterTypeDef sFilterConfig;

    /*
     * 接收标准 ID 0x000 ~ 0x7FF 的所有帧
     * 第一版调试时先全收，后面再改成只收 0x100~0x7FF
     */
    sFilterConfig.IdType = FDCAN_STANDARD_ID;
    sFilterConfig.FilterIndex = 0;
    sFilterConfig.FilterType = FDCAN_FILTER_MASK;
    sFilterConfig.FilterConfig = FDCAN_FILTER_TO_RXFIFO0;
    sFilterConfig.FilterID1 = 0x000;
    sFilterConfig.FilterID2 = 0x000;

    if (HAL_FDCAN_ConfigFilter(&hfdcan1, &sFilterConfig) != HAL_OK)
    {
        Error_Handler();
    }
    if (HAL_FDCAN_ConfigGlobalFilter(&hfdcan1,
                                     FDCAN_ACCEPT_IN_RX_FIFO0,
                                     FDCAN_REJECT,
                                     FDCAN_REJECT_REMOTE,
                                     FDCAN_REJECT_REMOTE) != HAL_OK)
    {
        Error_Handler();
    }

    if (HAL_FDCAN_Start(&hfdcan1) != HAL_OK)
    {
        Error_Handler();
    }
    // 开启 FIFO0 新消息中断
    if (HAL_FDCAN_ActivateNotification(&hfdcan1, FDCAN_IT_RX_FIFO0_NEW_MESSAGE, 0) != HAL_OK)
    {
        Error_Handler();
    }
}

uint8_t CAN_SendStdData(uint16_t id, uint8_t data[8])
{
    FDCAN_TxHeaderTypeDef txHeader;

    txHeader.Identifier = id;
    txHeader.IdType = FDCAN_STANDARD_ID;
    txHeader.TxFrameType = FDCAN_DATA_FRAME;
    txHeader.DataLength = FDCAN_DLC_BYTES_8;
    txHeader.ErrorStateIndicator = FDCAN_ESI_ACTIVE;
    txHeader.BitRateSwitch = FDCAN_BRS_OFF;
    txHeader.FDFormat = FDCAN_CLASSIC_CAN;
    txHeader.TxEventFifoControl = FDCAN_NO_TX_EVENTS;
    txHeader.MessageMarker = 0;

    if (HAL_FDCAN_GetTxFifoFreeLevel(&hfdcan1) == 0)
    {
        return 1;
    }

    if (HAL_FDCAN_AddMessageToTxFifoQ(&hfdcan1, &txHeader, data) != HAL_OK)
    {
        return 2;
    }

    return 0;
}

void HAL_FDCAN_RxFifo0Callback(FDCAN_HandleTypeDef *hfdcan, uint32_t RxFifo0ITs)
{
    if (hfdcan->Instance != FDCAN1)
    {
        return;
    }

    if (RxFifo0ITs & FDCAN_IT_RX_FIFO0_NEW_MESSAGE == 0)
    {
        return;
    }

    while(HAL_FDCAN_GetRxFifoFillLevel(hfdcan, FDCAN_RX_FIFO0) > 0)
    {
        fdcan

        for (uint8_t i = 0; i < 8; i++)
        {
            can_rx_dbg.last_data[i] = rxData[i];
        }

        can_rx_dbg.new_flag = 1;

        // CAN_ParseFrame(rxHeader.Identifier, rxData, rxHeader.DataLength);
    }
}

static uint8_t Can_RxQueuePushFromISR(const CanFrame *frame)
{
    uint8_t next_head = (rx_queue_head + 1) % CAN_RX_QUEUE_SIZE;

    if (next_head == rx_queue_tail)
    {
        return 0; // 队列已满
    }

    rx_queue[rx_queue_head] = *frame;

    __DMB();
    rx_queue_head = next_head;
    return 1; // 成功入队
}
static uint8_t Can_DlcToLength(uint32_t dlc)
{
    switch (dlc)
    {
    case FDCAN_DLC_BYTES_0:
        return 0;
    case FDCAN_DLC_BYTES_1:
        return 1;
    case FDCAN_DLC_BYTES_2:
        return 2;
    case FDCAN_DLC_BYTES_3:
        return 3;
    case FDCAN_DLC_BYTES_4:
        return 4;
    case FDCAN_DLC_BYTES_5:
        return 5;
    case FDCAN_DLC_BYTES_6:
        return 6;
    case FDCAN_DLC_BYTES_7:
        return 7;
    case FDCAN_DLC_BYTES_8:
        return 8;
    default:
        return 0; // Invalid DLC
    }
}
uint8_t Can_TryRead(CanFrame *frame)
{
    if (rx_queue_head == rx_queue_tail)
    {
        return 0; // 队列为空
    }

    *frame = rx_queue[rx_queue_tail];

    rx_queue_tail = (rx_queue_tail + 1) % CAN_RX_QUEUE_SIZE;
    return 1; // 成功出队
}
void CAN_ParseFrame(uint16_t id, uint8_t data[8], uint32_t dlc)
{
    if (dlc != FDCAN_DLC_BYTES_8)
    {
        return;
    }

    uint16_t function = id & CAN_FUNC_MASK;
    uint8_t motor_id = id & CAN_ID_MASK;

    if (motor_id == 0)
    {
        // 0 号先保留为广播，第一版可以不处理
        return;
    }

    switch (function)
    {
    case CAN_FUNC_CURRENT:
        // MotorCAN_ParseCurrentCmd(motor_id, data);
        break;

    case CAN_FUNC_VELOCITY:
        // MotorCAN_ParseVelocityCmd(motor_id, data);
        break;

    case CAN_FUNC_POSITION:
        // MotorCAN_ParsePositionCmd(motor_id, data);
        break;

    case CAN_FUNC_HEARTBEAT:
        // MotorCAN_ParseHeartbeat(motor_id, data);
        break;

    default:
        break;
    }
}

static int16_t get_i16_le(uint8_t *buf)
{
    return (int16_t)((uint16_t)buf[0] | ((uint16_t)buf[1] << 8));
}
static void put_i16_le(uint8_t *buf, int16_t value)
{
    buf[0] = value & 0xFF;
    buf[1] = (value >> 8) & 0xFF;
}
void MotorCAN_ParseCurrentCmd(uint8_t motor_id, uint8_t data[8])
{
    int16_t iq_mA = get_i16_le(&data[0]);
    int16_t id_mA = get_i16_le(&data[2]);
    int16_t limit_mA = get_i16_le(&data[4]);

    uint8_t mode = data[6];
    uint8_t enable = data[7];

    float iq_ref_A = iq_mA / 1000.0f;
    float id_ref_A = id_mA / 1000.0f;
    float limit_A = limit_mA / 1000.0f;

    if (enable && mode == 1)
    {
        // Motor_SetCurrentMode(motor_id, iq_ref_A, id_ref_A, limit_A);
    }
    else
    {
        // Motor_Disable(motor_id);
    }
}

void MotorCAN_SendFeedback(uint8_t motor_id, float angle_rad, float speed_rad_s, float iq_A, uint8_t temperature, uint8_t error_flags)
{
    uint8_t data[8];

    int16_t angle_i16 = (int16_t)(angle_rad * 1000.0f);
    int16_t speed_i16 = (int16_t)(speed_rad_s * 100.0f);
    int16_t iq_i16 = (int16_t)(iq_A * 1000.0f);

    put_i16_le(&data[0], angle_i16);
    put_i16_le(&data[2], speed_i16);
    put_i16_le(&data[4], iq_i16);

    data[6] = temperature;
    data[7] = error_flags;

    CAN_SendStdData(0x400 + motor_id, data);
}
