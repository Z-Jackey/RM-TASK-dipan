#include "DJI_Motor.h"

// 大疆 M3508 电流发送函数
void DJI_Motor_SendCurrent(FDCAN_HandleTypeDef *hfdcan, int16_t i1, int16_t i2, int16_t i3, int16_t i4)
{
    FDCAN_TxHeaderTypeDef txHeader;
    uint8_t txData[8];

    // 大疆标准：高字节在前
    txData[0] = (i1 >> 8) & 0xFF; txData[1] = i1 & 0xFF;
    txData[2] = (i2 >> 8) & 0xFF; txData[3] = i2 & 0xFF;
    txData[4] = (i3 >> 8) & 0xFF; txData[5] = i3 & 0xFF;
    txData[6] = (i4 >> 8) & 0xFF; txData[7] = i4 & 0xFF;

    txHeader.Identifier = 0x200; // 1~4号电机
    txHeader.IdType = FDCAN_STANDARD_ID;
    txHeader.TxFrameType = FDCAN_DATA_FRAME;
    txHeader.DataLength = FDCAN_DLC_BYTES_8;
    txHeader.FDFormat = FDCAN_CLASSIC_CAN;
    txHeader.BitRateSwitch = FDCAN_BRS_OFF;
    txHeader.TxEventFifoControl = FDCAN_NO_TX_EVENTS;
    txHeader.MessageMarker = 0;

    HAL_FDCAN_AddMessageToTxFifoQ(hfdcan, &txHeader, txData);
}