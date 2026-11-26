/**
  ******************************************************************************
  * @file    pid.h
  * @brief   通用位置式 PID（带 dt、积分限幅、条件积分抗饱和、微分先行）
  ******************************************************************************
  */

#ifndef __PID_H
#define __PID_H

#ifdef __cplusplus
extern "C" {
#endif

#include <stdint.h>

typedef struct
{
    float kp;
    float ki;
    float kd;

    float out_limit;    /* 输出限幅（对称）        */
    float i_limit;      /* 积分项输出限幅（对称）  */

    float i_term;       /* 已经乘过 ki 的积分项    */
    float last_fdb;     /* 上一次反馈，用于微分先行 */
    float out;          /* 上一次输出              */
    uint8_t inited;     /* 微分项是否已初始化      */
} PID_t;

void  PID_Init(PID_t *pid, float kp, float ki, float kd, float out_limit, float i_limit);
void  PID_Reset(PID_t *pid);
float PID_Calc(PID_t *pid, float ref, float fdb, float dt);

#ifdef __cplusplus
}
#endif

#endif /* __PID_H */
