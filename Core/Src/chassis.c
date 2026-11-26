/**
  ******************************************************************************
  * @file    chassis.c
  * @brief   麦克纳姆轮底盘控制（4 × M3508 / C620）
  *
  *          数据流（1kHz）：
  *            遥控 DBUS ─> 摇杆归一化 ─> 期望速度 (vx, vy, wz)
  *                                    │
  *                          麦轮逆运动学（4 个轮子的线速度）
  *                                    │
  *                          轮速 rpm（乘减速比、限幅、斜坡）
  *                                    │
  *                          每个轮子一个速度环 PID ─> 电流给定 ─> CAN 0x200
  *
  *          坐标系约定：x 向前，y 向左，z 向上，wz 逆时针为正。
  *
  *          麦轮逆运动学（0=左前 1=右前 2=左后 3=右后）：
  *            v_FL = vx - vy - k·wz
  *            v_FR = vx + vy + k·wz
  *            v_RL = vx + vy - k·wz
  *            v_RR = vx - vy + k·wz        k = 半轴距 + 半轮距
  ******************************************************************************
  */

#include "chassis.h"
#include "motor.h"
#include "remote.h"
#include "pid.h"

#define TWO_PI_F            6.28318530718f
/* 1 m/s 的车速对应多少轮子 rpm（轮子 rpm = v / R * 60 / 2π） */
#define WHEEL_RPM_PER_MPS   (60.0f / (TWO_PI_F * WHEEL_RADIUS_M))

Chassis_t chassis;

/* 每个轮子的安装方向系数（用指定初始化器，这样改了 WHEEL_xx 也不会错位） */
static const float s_dir_sign[MOTOR_COUNT] =
{
    [WHEEL_FL] = MOTOR_DIR_FL,
    [WHEEL_FR] = MOTOR_DIR_FR,
    [WHEEL_RL] = MOTOR_DIR_RL,
    [WHEEL_RR] = MOTOR_DIR_RR,
};

/* 每个轮子一个速度环 */
static PID_t s_speed_pid[MOTOR_COUNT];

/* ------------------------------------------------------------------------- */
static float Chassis_Clamp(float v, float limit)
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

/**
  * @brief  摇杆原始值 -> -1.0 ~ +1.0（带死区，死区外重新映射，避免跳变）
  */
static float Chassis_StickNorm(int16_t raw)
{
    const float full = 660.0f;
    const float dz   = (float)RC_DEADZONE / full;
    float       v    = (float)raw / full;
    float       mag;

    if (v > 1.0f)
    {
        v = 1.0f;
    }
    else if (v < -1.0f)
    {
        v = -1.0f;
    }

    mag = (v >= 0.0f) ? v : -v;
    if (mag <= dz)
    {
        return 0.0f;
    }

    mag = (mag - dz) / (1.0f - dz);
    return (v >= 0.0f) ? mag : -mag;
}

/**
  * @brief  目标值斜坡，限制加速度
  */
static float Chassis_Slew(float now, float target, float step)
{
    if (target > (now + step))
    {
        return now + step;
    }
    if (target < (now - step))
    {
        return now - step;
    }
    return target;
}

/**
  * @brief  过温降额系数 1.0 -> 0.0
  */
static float Chassis_TempDerate(uint8_t temp)
{
    if (temp >= MOTOR_TEMP_STOP_C)
    {
        return 0.0f;
    }
    if (temp <= MOTOR_TEMP_DERATE_C)
    {
        return 1.0f;
    }
    return (float)(MOTOR_TEMP_STOP_C - temp) /
           (float)(MOTOR_TEMP_STOP_C - MOTOR_TEMP_DERATE_C);
}

/**
  * @brief  根据遥控拨杆决定工作模式
  */
static ChassisMode_t Chassis_SelectMode(void)
{
    /* 遥控掉线 -> 一律停机，这是最后一道保险 */
    if (Remote_IsOnline() == 0U)
    {
        return CHASSIS_MODE_STOP;
    }

    switch (remote.s1)
    {
        case RC_SW_MID:
            return CHASSIS_MODE_REMOTE;

        case RC_SW_DOWN:
            return CHASSIS_MODE_SPIN_TEST;

        case RC_SW_UP:
        default:
            return CHASSIS_MODE_STOP;
    }
}

/* 导出函数 ----------------------------------------------------------------*/

void Chassis_Init(void)
{
    Motor_Init();
    Remote_Init();

    for (uint8_t i = 0; i < MOTOR_COUNT; i++)
    {
        PID_Init(&s_speed_pid[i], SPD_PID_KP, SPD_PID_KI, SPD_PID_KD,
                 SPD_PID_OUT_LIMIT, SPD_PID_I_LIMIT);

        chassis.target_rpm[i]  = 0.0f;
        chassis.ref_rpm[i]     = 0.0f;
        chassis.fdb_rpm[i]     = 0.0f;
        chassis.current_cmd[i] = 0;
    }

    chassis.mode             = CHASSIS_MODE_STOP;
    chassis.vx               = 0.0f;
    chassis.vy               = 0.0f;
    chassis.wz               = 0.0f;
    chassis.rc_online        = 0U;
    chassis.motor_all_online = 0U;
}

/**
  * @brief  底盘控制任务，由 main 以 1kHz 调用
  * @param  dt 距离上一次调用的时间，单位秒
  */
void Chassis_Task(float dt)
{
    float vx;
    float vy;
    float wz;

    if (dt <= 0.0f)
    {
        dt = (float)CHASSIS_TASK_PERIOD_MS * 0.001f;
    }
    else if (dt > CHASSIS_DT_MAX)
    {
        dt = CHASSIS_DT_MAX;
    }

    /* ---------- 1. 在线检测 ---------- */
    Motor_UpdateOnline();

    chassis.rc_online        = Remote_IsOnline();
    chassis.motor_all_online = 1U;
    for (uint8_t i = 0; i < MOTOR_COUNT; i++)
    {
        if (motor[i].online == 0U)
        {
            chassis.motor_all_online = 0U;
        }
    }

    /* ---------- 2. 模式 ---------- */
    chassis.mode = Chassis_SelectMode();

    /* 只要有一个电机掉线就整车不动，避免三个轮子推着车乱跑 */
    if (chassis.motor_all_online == 0U)
    {
        chassis.mode = CHASSIS_MODE_STOP;
    }

    /* ---------- 3. 期望速度 ---------- */
    vx = 0.0f;
    vy = 0.0f;
    wz = 0.0f;

    if (chassis.mode == CHASSIS_MODE_REMOTE)
    {
        float gear;

        switch (remote.s2)
        {
            case RC_SW_UP:
                gear = RC_GEAR_LOW;
                break;

            case RC_SW_MID:
                gear = RC_GEAR_MID;
                break;

            case RC_SW_DOWN:
                gear = RC_GEAR_HIGH;
                break;

            default:
                gear = RC_GEAR_LOW;
                break;
        }

        vx = Chassis_StickNorm(remote.ch[RC_CH_VX]) * MAX_LINEAR_SPEED_MPS   * gear * RC_VX_SIGN;
        vy = Chassis_StickNorm(remote.ch[RC_CH_VY]) * MAX_LINEAR_SPEED_MPS   * gear * RC_VY_SIGN;
        wz = Chassis_StickNorm(remote.ch[RC_CH_WZ]) * MAX_ANGULAR_SPEED_RADPS * gear * RC_WZ_SIGN;
    }
    else if (chassis.mode == CHASSIS_MODE_SPIN_TEST)
    {
        /* 自转测试：慢速原地转，用来检查四个轮子转向和安装方向 */
        wz = SPIN_TEST_WZ;
    }
    else
    {
        /* 停机，保持 0 */
    }

    chassis.vx = vx;
    chassis.vy = vy;
    chassis.wz = wz;

    /* ---------- 4. 麦轮逆运动学 -> 目标轮速 ---------- */
    if (chassis.mode != CHASSIS_MODE_STOP)
    {
        float wheel_mps[MOTOR_COUNT];

        wheel_mps[WHEEL_FL] = vx - vy - (KINEMATIC_K * wz);
        wheel_mps[WHEEL_FR] = vx + vy + (KINEMATIC_K * wz);
        wheel_mps[WHEEL_RL] = vx + vy - (KINEMATIC_K * wz);
        wheel_mps[WHEEL_RR] = vx - vy + (KINEMATIC_K * wz);

        for (uint8_t i = 0; i < MOTOR_COUNT; i++)
        {
            float rpm = wheel_mps[i] * WHEEL_RPM_PER_MPS * REDUCTION_RATIO;
            chassis.target_rpm[i] = Chassis_Clamp(rpm, MAX_WHEEL_RPM);
        }
    }
    else
    {
        for (uint8_t i = 0; i < MOTOR_COUNT; i++)
        {
            chassis.target_rpm[i] = 0.0f;
        }
    }

    /* ---------- 5. 斜坡 + 速度环 ---------- */
    for (uint8_t i = 0; i < MOTOR_COUNT; i++)
    {
        float step = WHEEL_ACCEL_RPM_PER_S * dt;
        float ref_motor;
        float fdb_motor;
        float out;
        float derate;

        chassis.ref_rpm[i] = Chassis_Slew(chassis.ref_rpm[i], chassis.target_rpm[i], step);
        chassis.fdb_rpm[i] = (float)motor[i].rpm;

        if ((chassis.mode == CHASSIS_MODE_STOP) || (motor[i].online == 0U))
        {
            PID_Reset(&s_speed_pid[i]);
            chassis.current_cmd[i] = 0;
            continue;
        }

        /* 换算到电机自身的转向：
           ★ 方向系数只能乘在【给定】上，反馈必须保持电机原始符号。
             如果连反馈一起乘 -1，速度环会变成正反馈 → 满速狂转。
             这样处理之后：反装电机的 PID 依然稳定，运动学也正确。 */
        ref_motor = s_dir_sign[i] * chassis.ref_rpm[i];
        fdb_motor = (float)motor[i].rpm;

        out = PID_Calc(&s_speed_pid[i], ref_motor, fdb_motor, dt);

        derate = Chassis_TempDerate(motor[i].temp);
        out *= derate;

        chassis.current_cmd[i] = (int16_t)Chassis_Clamp(out, (float)MAX_CURRENT_RAW);
    }

    /* ---------- 6. 发出电流给定（1kHz 一帧，4 个电机打包） ---------- */
    Motor_SendCurrent(chassis.current_cmd);
}
