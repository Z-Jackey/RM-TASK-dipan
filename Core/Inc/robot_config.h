/**
  ******************************************************************************
  * @file    robot_config.h
  * @brief   底盘全部可调参数集中在这里（机械尺寸 / 限幅 / PID / 遥控映射 / 电机方向）
  *
  *          调车时 90% 的时间只会改这个文件，不需要动 .c 文件。
  ******************************************************************************
  */

#ifndef __ROBOT_CONFIG_H
#define __ROBOT_CONFIG_H

/* ======================= 1. 控制周期 ======================= */
#define CHASSIS_TASK_PERIOD_MS      1U          /* 底盘任务周期 1ms，即 1kHz     */
#define CHASSIS_DT_MAX              0.02f       /* 单次 dt 上限，防止卡顿后积分爆掉 */

/* ======================= 2. 机械参数 ======================= */
#define WHEEL_RADIUS_M              0.076f      /* 麦轮半径 m（直径 152mm）       */
#define REDUCTION_RATIO             19.2f       /* M3508 减速比 = 3591 / 187      */
#define HALF_WHEELBASE_M            0.25f       /* 前后轮中心距 / 2               */
#define HALF_TRACK_M                0.25f       /* 左右轮中心距 / 2（量出来再改） */

/* 麦轮运动学的旋转半径系数：k = 半轴距 + 半轮距 */
#define KINEMATIC_K                 (HALF_WHEELBASE_M + HALF_TRACK_M)

/* ======================= 3. 速度与电流限幅 ======================= */
#define MAX_WHEEL_RPM               8000.0f     /* 转子最大转速（官方约 9300，留余量） */
#define MAX_LINEAR_SPEED_MPS        2.0f        /* 遥控最大平移速度 m/s           */
#define MAX_ANGULAR_SPEED_RADPS     3.0f        /* 遥控最大旋转角速度 rad/s       */
#define WHEEL_ACCEL_RPM_PER_S       10000.0f    /* 目标转速斜坡，防止电流冲击      */
#define MAX_CURRENT_RAW             16000       /* 电流给定限幅（16384 = 20A）    */

/* ======================= 4. 电机 / CAN ======================= */
#define MOTOR_COUNT                 4
#define MOTOR_CAN_CMD_ID            0x200       /* 1~4 号电机电流控制帧           */
#define MOTOR_CAN_FB_ID_BASE        0x201       /* 反馈帧 0x201 ~ 0x204           */
#define MOTOR_OFFLINE_TIMEOUT_MS    100U        /* 超过 100ms 没反馈就认为掉线     */
#define MOTOR_TEMP_DERATE_C         80U         /* 超过 80℃ 开始降额              */
#define MOTOR_TEMP_STOP_C           95U         /* 95℃ 直接停转                   */

/* ======================= 5. 轮子顺序 / 安装方向 ======================= */
/* 这四个宏既是"轮子位置"，也是"CAN 反馈帧下标"：
     下标 i  <->  CAN 反馈帧 0x201 + i  <->  C620 拨码 ID = i + 1

   ★ 本车实测的电调 ID 分布（俯视，照片上方为车头）：
        左前 = ID1    右前 = ID4
        左后 = ID2    右后 = ID3
     所以下标对应关系是：ID1 -> 0   ID2 -> 1   ID3 -> 2   ID4 -> 3

   如果实际车头方向相反（你站的那侧才是车头），把下面四个数字换成这么一组：
        WHEEL_FL 1   WHEEL_FR 2   WHEEL_RL 0   WHEEL_RR 3
   chassis.c 里用指定初始化器自动跟着换，不用改别的地方。               */
#define WHEEL_FL                    0           /* 左前 = ID1 */
#define WHEEL_FR                    3           /* 右前 = ID4 */
#define WHEEL_RL                    1           /* 左后 = ID2 */
#define WHEEL_RR                    2           /* 右后 = ID3 */

/* 方向系数：
     +1 = 该电机给正电流时，轮子朝"车头方向"转
     -1 = 电机反装（给正电流轮子反而往后转）
   本车实测：四个电调极性一致，全部取 +1（按镜像给 -1 会导致速度环正反馈）。
   ★ 第一次上电请把车架空，推杆向前，四个轮子都应该朝前转；
     哪个轮子反了就把对应的符号改一下。                                  */
#define MOTOR_DIR_FL                (+1.0f)
#define MOTOR_DIR_FR                (-1.0f)
#define MOTOR_DIR_RL                (+1.0f)
#define MOTOR_DIR_RR                (-1.0f)

/* ======================= 6. 轮速环 PID ======================= */
/* 输入：误差 (rpm)   输出：电流给定 (-16384 ~ +16384)
   调参顺序：先把 ki 置 0，加大 kp 到轮子跟得上又不啸叫，再加 ki 消除静差。 */
#define SPD_PID_KP                  6.0f
#define SPD_PID_KI                  10.0f
#define SPD_PID_KD                  0.0f
#define SPD_PID_OUT_LIMIT           16000.0f
#define SPD_PID_I_LIMIT             8000.0f

/* ======================= 7. DBUS 遥控映射 ======================= */
#define RC_OFFLINE_TIMEOUT_MS       100U        /* 100ms 收不到帧就判掉线        */
#define RC_DEADZONE                 20          /* 摇杆死区（原始值，约 3%）      */

/* 通道号（DT7/DR16 手册 & 官方协议：A-E-T-R 顺序）
      ch0 = 右摇杆左右
      ch1 = 右摇杆上下
      ch2 = 左摇杆上下
      ch3 = 左摇杆左右
   默认：左摇杆开车（上下=前后，左右=横移），右摇杆左右=转向。

   ★ 如果实车发现"推左摇杆上下，车却横着走"，说明你的遥控器通道顺序
     和这里不一样，把 RC_CH_VX 和 RC_CH_VY 的值对调即可。           */
/* 通道号（本遥控器实测，DT7 改过设置后的顺序）：
      ch[0] = 右摇杆左右（右为正）
      ch[1] = 右摇杆上下（未使用）
      ch[2] = 左摇杆左右（右为正）
      ch[3] = 左摇杆上下（上为正）
   布局：左摇杆管方向（前后+横移），右摇杆左右管自转。               */
#define RC_CH_VX                    3           /* 左摇杆上下 -> 前进方向        */
#define RC_CH_VY                    2           /* 左摇杆左右 -> 横移方向        */
#define RC_CH_WZ                    0           /* 右摇杆左右 -> 自转方向        */

/* 通道方向：+1 保持，-1 取反
   车体坐标系：x 向前，y 向左，绕 z 逆时针为正。
   摇杆往右推时 ch[3]/ch[0] 为正，对应"向右走 / 顺时针转"，所以取 -1。 */
#define RC_VX_SIGN                  (+1.0f)
#define RC_VY_SIGN                  (-1.0f)
#define RC_WZ_SIGN                  (-1.0f)

/* DR16 拨杆档位约定：1=上  3=中  2=下 */
#define RC_SW_UP                    1
#define RC_SW_MID                   3
#define RC_SW_DOWN                  2

/* s2 速度档位倍率（上=慢 中=中 下=快） */
#define RC_GEAR_LOW                 0.3f
#define RC_GEAR_MID                 0.6f
#define RC_GEAR_HIGH                1.0f

/* s1 = 下 的时候进入自转测试，角速度 rad/s */
#define SPIN_TEST_WZ                1.0f

/* ======================= 8. 调试诊断开关 ======================= */
/* 1 = 上电时把所有 GPIO（A~E）临时切成数字输入+下拉，扫描 100ms 内的电平翻转，
 *     结果在全局变量 pin_diag 里，可以查出"信号到底接在哪个引脚"。
 *     STM32H7 复位后 GPIO 默认是模拟模式，不切开根本读不到外部电平，
 *     所以扫描会临时改 MODER/PUPDR（SWD 口和 CAN 口保留，扫描完自动恢复）。
 *     调车/正式跑之前务必改回 0。 */
#define DIAG_PIN_SCAN               0
#define DIAG_SCAN_ITER              100000U     /* 每次扫描的采样次数（约 100ms） */

#endif /* __ROBOT_CONFIG_H */
