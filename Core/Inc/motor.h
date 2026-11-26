/**
  ******************************************************************************
  * @file    motor.h
  * @brief   DJI M3508 + C620 电调驱动（输出电流模式）
  *
  *          发送：0x200 一帧 8 字节，把 4 个电机的电流给定打包发出去
  *          接收：0x201 ~ 0x204 每帧 8 字节，角度 / 转速 / 电流 / 温度
  ******************************************************************************
  */

#ifndef __MOTOR_H
#define __MOTOR_H

#ifdef __cplusplus
extern "C" {
#endif

#include "main.h"
#include "robot_config.h"

typedef struct
{
    uint16_t angle;         /* 机械角度 0 ~ 8191            */
    int16_t  rpm;           /* 转子转速 rpm（未除减速比）    */
    int16_t  current;       /* 实际转矩电流值                */
    uint8_t  temp;          /* 温度 ℃                       */
    uint8_t  online;        /* 1 = 在线                     */
    uint32_t last_rx_tick;  /* 最后一次收到反馈的时间         */
} Motor_t;

extern Motor_t motor[MOTOR_COUNT];

void    Motor_Init(void);
void    Motor_SendCurrent(const int16_t current[MOTOR_COUNT]);
void    Motor_UpdateOnline(void);
uint8_t Motor_IsOnline(uint8_t idx);

#ifdef __cplusplus
}
#endif

#endif /* __MOTOR_H */
