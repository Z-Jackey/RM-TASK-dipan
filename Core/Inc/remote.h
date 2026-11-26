/**
  ******************************************************************************
  * @file    remote.h
  * @brief   DJI DR16 遥控器 DBUS 接收与解析
  *
  *          DBUS：100000bps，8 数据位 + 偶校验 + 2 停止位，每帧 18 字节
  *                数据位在 STM32 上要配成 9bit（第 9 位是校验位）
  *          帧格式（无帧头！）：
  *            byte 0~5  : 4 个摇杆通道，每通道 11bit，范围 364~1684，中值 1024
  *            byte 5    : bit5:4 = s1，bit7:6 = s2
  *            byte 6~11 : 鼠标 x / y / z
  *            byte 12~13: 鼠标左右键
  *            byte 14~15: 键盘
  ******************************************************************************
  */

#ifndef __REMOTE_H
#define __REMOTE_H

#ifdef __cplusplus
extern "C" {
#endif

#include "main.h"
#include "robot_config.h"

#define DBUS_FRAME_LEN      18U

typedef struct
{
    int16_t  ch[4];         /* 摇杆原始值，已减中值：-660 ~ +660           */
    float    nch[4];        /* 归一化：-1.0 ~ +1.0（不含死区）             */
    uint8_t  s1;            /* 左拨杆 1=上 3=中 2=下                       */
    uint8_t  s2;            /* 右拨杆 1=上 3=中 2=下                       */
    int16_t  mouse_x;
    int16_t  mouse_y;
    int16_t  mouse_z;
    uint8_t  mouse_l;
    uint8_t  mouse_r;
    uint16_t key;
    uint8_t  online;        /* 1 = 遥控在线                                */
    uint32_t last_rx_tick;  /* 最后一次收到有效帧的时间                     */
    uint32_t frame_cnt;     /* 收到的有效帧数                              */
    uint32_t err_cnt;       /* 校验失败 / 错位丢弃的帧数                    */
} Remote_t;

extern Remote_t remote;

#if DIAG_PIN_SCAN
/* 引脚活动统计：上电时把 GPIOA/B/D 每个引脚的电平翻转次数数一遍，
   信号线所在引脚的数字会明显大于 0（DBUS 约几千次，CAN 约几百次）。 */
typedef struct
{
    uint32_t a[16];
    uint32_t b[16];
    uint32_t c[16];
    uint32_t d[16];
    uint32_t e[16];
    uint32_t b12_high;       /* 扫描时 PB12 是高电平的次数（判断线是否被拉低/悬空） */
    uint32_t b12_low;
    uint32_t scan_pass;      /* 已经累计扫描了多少轮 */
} PinDiag_t;

extern PinDiag_t pin_diag;

void Remote_DiagScanPins(void);
#endif

void    Remote_Init(void);
uint8_t Remote_IsOnline(void);

#ifdef __cplusplus
}
#endif

#endif /* __REMOTE_H */
