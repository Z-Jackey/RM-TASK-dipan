/**
  ******************************************************************************
  * @file    motor.c
  * @brief   DJI M3508 + C620 驱动
  *
  *          C620 电调工作在电流（转矩）模式：
  *            主机每 1ms 发一次 0x200，一帧打包 1~4 号电机
  *            每个 int16 范围 -16384 ~ +16384，对应 -20A ~ +20A
  *            高字节在前（大端）
  *
  *          反馈帧 0x201~0x204：
  *            data[0..1] 机械角度 0~8191
  *            data[2..3] 转子转速 rpm
  *            data[4..5] 实际转矩电流
  *            data[6]    温度
  *            data[7]    保留
  ******************************************************************************
  */

#include "motor.h"
#include "fdcan.h"

Motor_t motor[MOTOR_COUNT];

/* 单点限幅 ----------------------------------------------------------------*/
static int16_t Motor_ClampCurrent(int32_t value)
{
    if (value >  MAX_CURRENT_RAW)
    {
        return (int16_t) MAX_CURRENT_RAW;
    }
    if (value < -MAX_CURRENT_RAW)
    {
        return (int16_t)(-MAX_CURRENT_RAW);
    }
    return (int16_t) value;
}

/* 导出函数 ----------------------------------------------------------------*/

void Motor_Init(void)
{
    for (uint8_t i = 0; i < MOTOR_COUNT; i++)
    {
        motor[i].angle        = 0U;
        motor[i].rpm          = 0;
        motor[i].current      = 0;
        motor[i].temp         = 0U;
        motor[i].online       = 0U;
        motor[i].last_rx_tick = 0U;
    }
}

/**
  * @brief  给 1~4 号电机发一帧电流给定
  * @param  current 数组下标与轮子下标一致，单位是 C620 的原始值
  */
void Motor_SendCurrent(const int16_t current[MOTOR_COUNT])
{
    FDCAN_TxHeaderTypeDef tx_header;
    uint8_t tx_data[8];
    int16_t cmd;

    for (uint8_t i = 0; i < MOTOR_COUNT; i++)
    {
        cmd = Motor_ClampCurrent(current[i]);

        /* 大端：高字节在前 */
        tx_data[2U * i]      = (uint8_t)((uint16_t)cmd >> 8);
        tx_data[2U * i + 1U] = (uint8_t)((uint16_t)cmd & 0x00FFU);
    }

    tx_header.Identifier          = MOTOR_CAN_CMD_ID;
    tx_header.IdType              = FDCAN_STANDARD_ID;
    tx_header.TxFrameType         = FDCAN_DATA_FRAME;
    tx_header.DataLength          = FDCAN_DLC_BYTES_8;
    tx_header.ErrorStateIndicator = FDCAN_ESI_ACTIVE;
    tx_header.BitRateSwitch       = FDCAN_BRS_OFF;
    tx_header.FDFormat            = FDCAN_CLASSIC_CAN;      /* 3508 只认经典 CAN */
    tx_header.TxEventFifoControl  = FDCAN_NO_TX_EVENTS;
    tx_header.MessageMarker       = 0U;

    /* 发送 FIFO 满的时候会返回 HAL_ERROR，丢掉这一帧即可（1ms 后还有新的一帧） */
    (void)HAL_FDCAN_AddMessageToTxFifoQ(&hfdcan1, &tx_header, tx_data);
}

/**
  * @brief  超时检测：在底盘任务里每周期调用一次
  */
void Motor_UpdateOnline(void)
{
    uint32_t now = HAL_GetTick();

    for (uint8_t i = 0; i < MOTOR_COUNT; i++)
    {
        if (motor[i].online != 0U)
        {
            if ((now - motor[i].last_rx_tick) > MOTOR_OFFLINE_TIMEOUT_MS)
            {
                motor[i].online = 0U;
            }
        }
    }
}

uint8_t Motor_IsOnline(uint8_t idx)
{
    if (idx >= MOTOR_COUNT)
    {
        return 0U;
    }
    return motor[idx].online;
}

/* CAN 接收回调：FIFO0 收到新消息时由 HAL 调用 ------------------------------*/
void HAL_FDCAN_RxFifo0Callback(FDCAN_HandleTypeDef *hfdcan, uint32_t RxFifo0ITs)
{
    FDCAN_RxHeaderTypeDef rx_header;
    uint8_t rx_data[8];

    if (hfdcan->Instance != FDCAN1)
    {
        return;
    }
    if ((RxFifo0ITs & FDCAN_IT_RX_FIFO0_NEW_MESSAGE) == 0U)
    {
        return;
    }

    /* 一次中断可能攒了好几帧，全部取出来 */
    while (HAL_FDCAN_GetRxMessage(hfdcan, FDCAN_RX_FIFO0, &rx_header, rx_data) == HAL_OK)
    {
        if ((rx_header.IdType != FDCAN_STANDARD_ID) ||
            (rx_header.DataLength != FDCAN_DLC_BYTES_8))
        {
            continue;
        }

        if ((rx_header.Identifier < MOTOR_CAN_FB_ID_BASE) ||
            (rx_header.Identifier >= (MOTOR_CAN_FB_ID_BASE + MOTOR_COUNT)))
        {
            continue;
        }

        uint8_t idx = (uint8_t)(rx_header.Identifier - MOTOR_CAN_FB_ID_BASE);

        motor[idx].angle        = (uint16_t)(((uint16_t)rx_data[0] << 8) | rx_data[1]);
        motor[idx].rpm          = (int16_t)(((uint16_t)rx_data[2] << 8) | rx_data[3]);
        motor[idx].current      = (int16_t)(((uint16_t)rx_data[4] << 8) | rx_data[5]);
        motor[idx].temp         = rx_data[6];
        motor[idx].online       = 1U;
        motor[idx].last_rx_tick = HAL_GetTick();
    }
}
