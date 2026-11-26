/**
  ******************************************************************************
  * @file    chassis.h
  * @brief   麦克纳姆轮底盘（4 × M3508）
  ******************************************************************************
  */

#ifndef __CHASSIS_H
#define __CHASSIS_H

#ifdef __cplusplus
extern "C" {
#endif

#include "main.h"
#include "robot_config.h"

/* 底盘工作模式 --------------------------------------------------------------*/
typedef enum
{
    CHASSIS_MODE_STOP = 0,      /* 停机：电流直接给 0                        */
    CHASSIS_MODE_REMOTE,        /* 遥控行驶：s1 中档                         */
    CHASSIS_MODE_SPIN_TEST      /* 自转测试：s1 下档                         */
} ChassisMode_t;

/* 底盘对象，调试时在 watch 窗口里看这个就够了 ------------------------------*/
typedef struct
{
    ChassisMode_t mode;                         /* 当前模式                    */
    float         vx;                           /* 期望 vx  m/s                */
    float         vy;                           /* 期望 vy  m/s（左为正）      */
    float         wz;                           /* 期望 wz  rad/s（逆时针为正）*/
    float         target_rpm[MOTOR_COUNT];      /* 运动学解算出来的轮速        */
    float         ref_rpm[MOTOR_COUNT];         /* 斜坡处理后的目标轮速        */
    float         fdb_rpm[MOTOR_COUNT];         /* 反馈轮速                    */
    int16_t       current_cmd[MOTOR_COUNT];     /* 发给电调的电流              */
    uint8_t       rc_online;                    /* 遥控是否在线                */
    uint8_t       motor_all_online;             /* 四个电机是否都在线          */
} Chassis_t;

extern Chassis_t chassis;

void Chassis_Init(void);
void Chassis_Task(float dt);

#ifdef __cplusplus
}
#endif

#endif /* __CHASSIS_H */
