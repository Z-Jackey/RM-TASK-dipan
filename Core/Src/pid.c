/**
  ******************************************************************************
  * @file    pid.c
  * @brief   位置式 PID
  *
  *          和原来那份 PID 的区别：
  *            1. 积分、微分都带 dt，控制周期变了不用重新调参
  *            2. 积分项直接按"输出量"限幅，换 kp/ki 不会出现积分爆炸
  *            3. 输出饱和时不再继续积分（条件积分抗饱和）
  *            4. 微分作用在反馈量上（derivative on measurement），给定突变不会打冲击
  ******************************************************************************
  */

#include "pid.h"

static float pid_clamp(float v, float limit)
{
    if (v > limit)
    {
        return limit;
    }
    if (v < -limit)
    {
        return -limit;
    }
    return v;
}

void PID_Init(PID_t *pid, float kp, float ki, float kd, float out_limit, float i_limit)
{
    if (pid == 0)
    {
        return;
    }

    pid->kp        = kp;
    pid->ki        = ki;
    pid->kd        = kd;
    pid->out_limit = out_limit;
    pid->i_limit   = i_limit;

    PID_Reset(pid);
}

void PID_Reset(PID_t *pid)
{
    if (pid == 0)
    {
        return;
    }

    pid->i_term   = 0.0f;
    pid->last_fdb = 0.0f;
    pid->out      = 0.0f;
    pid->inited   = 0U;
}

float PID_Calc(PID_t *pid, float ref, float fdb, float dt)
{
    float err;
    float p_term;
    float d_term;
    float out;

    if (pid == 0)
    {
        return 0.0f;
    }

    if (dt <= 0.0001f)
    {
        dt = 0.0001f;
    }
    else if (dt > 0.02f)
    {
        dt = 0.02f;
    }

    err = ref - fdb;

    p_term = pid->kp * err;

    /* 微分先行：对反馈量求导，避免给定阶跃时输出尖峰 */
    if (pid->inited)
    {
        d_term = -pid->kd * (fdb - pid->last_fdb) / dt;
    }
    else
    {
        d_term = 0.0f;
        pid->inited = 1U;
    }
    pid->last_fdb = fdb;

    /* 条件积分：已经顶到限幅且误差还在往同一个方向推，就不再积分 */
    out = p_term + pid->i_term + d_term;
    if (!(((out >= pid->out_limit) && (err > 0.0f)) ||
          ((out <= -pid->out_limit) && (err < 0.0f))))
    {
        pid->i_term += pid->ki * err * dt;
        pid->i_term  = pid_clamp(pid->i_term, pid->i_limit);
    }

    out = p_term + pid->i_term + d_term;
    out = pid_clamp(out, pid->out_limit);

    pid->out = out;
    return out;
}
