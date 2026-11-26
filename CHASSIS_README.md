# 麦轮底盘代码说明（STM32H723VG + 4×M3508/C620 + DR16 DBUS）

## 1. 文件结构

| 文件 | 作用 |
| --- | --- |
| `Core/Inc/robot_config.h` | **所有可调参数**：机械尺寸、限幅、PID、遥控映射、电机方向 |
| `Core/Inc/chassis.h` / `Core/Src/chassis.c` | 麦轮逆运动学 + 4 路速度环 + 模式/安全逻辑 |
| `Core/Inc/motor.h` / `Core/Src/motor.c` | M3508/C620 驱动：发 0x200、收 0x201~0x204 |
| `Core/Inc/remote.h` / `Core/Src/remote.c` | DR16 DBUS 接收解析（DMA + 空闲中断） |
| `Core/Inc/pid.h` / `Core/Src/pid.c` | 带 dt 的位置式 PID |
| `Core/Src/main.c` | 初始化 + 1kHz 任务调度 |

`Core/Src/DJI_motor.c`、`Core/Inc/DJI_motor.h` 是原来留下的重复实现（没有加进
CMakeLists，不参与编译），可以直接删掉。

## 2. 外设与引脚（沿用 CubeMX 里的配置）

| 功能 | 外设 | 引脚 | 参数 |
| --- | --- | --- | --- |
| 遥控 DBUS | UART5 | **PD2=RX**（本板实测，非默认 PB12） PB13=TX | 100000bps，9bit（8 数据+偶校验），**1 停止位** |
| 电机 CAN | FDCAN1 | PD0=RX PD1=TX | 经典 CAN，1Mbps，时钟源 HSE(24MHz) |
| 时钟 | HSE | PH0/PH1 | 24MHz，SYSCLK=120MHz |

### ★ 本车实测配置（调通时确定的，不要随便改）

| 项目 | 值 |
| --- | --- |
| DBUS 接收脚 | **PD2**（UART5_RX / AF8）—— 不是 CubeMX 默认的 PB12 |
| 遥控通道 | `ch[0]`=右摇杆左右，`ch[2]`=左摇杆左右，`ch[3]`=左摇杆上下 |
| 布局 | 左摇杆=方向（前后+横移），右摇杆左右=自转 |
| 电调 ID | 左前=1 右前=4 左后=2 右后=3（`WHEEL_* = 0/3/1/2`） |
| 电机方向 | `MOTOR_DIR = +1 / -1 / +1 / -1`（左正右反，镜像安装） |

> ★ **方向系数只乘在"给定"上，绝不能乘在反馈上** —— 反馈乘了会把速度环变成正反馈，
> 表现就是"零输入时某个轮子满速狂转"。详见 `chassis.c` 里 `Chassis_Task()` 的注释。

## 3. 数据流（1kHz）

```
DBUS 18 字节 ──> remote.ch[0..3] ──> 左摇杆 (vx, vy) + 右摇杆 (wz)
                                          │
                                 麦轮逆运动学
                                          │
                              4 个轮子目标转速（转子 rpm）
                                          │
                           斜坡限幅 ──> 4 × 速度环 PID
                                          │
                              CAN 0x200 电流给定 ──> C620
                    CAN 0x201~0x204 反馈 ──> motor[0..3].rpm 回到 PID
```

坐标系：**x 向前，y 向左，z 向上，wz 逆时针为正**。
麦轮逆运动学（0=左前 1=右前 2=左后 3=右后）：
```
v_FL = vx - vy - k·wz
v_FR = vx + vy + k·wz
v_RL = vx + vy - k·wz
v_RR = vx - vy + k·wz        k = 半轴距 + 半轮距
```

## 4. 遥控器拨杆含义

| 开关 | 位置 | 含义 |
| --- | --- | --- |
| 左拨杆 S1 | 上 (1) | **停机**（上电默认，最安全） |
| | 中 (3) | 正常遥控行驶 |
| | 下 (2) | 自转测试（慢速原地逆时针转） |
| 右拨杆 S2 | 上 (1) | 低速 30% |
| | 中 (3) | 中速 60% |
| | 下 (2) | 全速 100% |

摇杆：左摇杆上下=前后，左摇杆左右=横移，右摇杆左右=转向。

## 5. 上车前必做的检查（★ 先架空，别直接落地）

1. **架空四个轮子**，S1 打上档，上电。
2. 调试器里看 `remote.frame_cnt` 是否在涨、`remote.online` 是否为 1。
   - 不涨 → DBUS 没收到，先查接线 / 波特率 / 停止位。
3. 看 `remote.ch[0..3]` 随摇杆变化，`remote.s1 / remote.s2` 随开关变化（1/3/2）。
4. 看 `motor[0..3].online` 是否都为 1，`temp` 是否正常，`rpm` 静止时≈0。
5. S1 打中档，轻推左摇杆向前，**四个轮子都应该朝"前进"方向转**。
6. S1 打下档自转，俯视应逆时针转：左前/左后向后，右前/右后向前。
7. 全部正确后再落地，先用 S2 低速档试。

## 6. 方向/通道不对时改哪里（只改 `robot_config.h`）

| 现象 | 改哪个 |
| --- | --- |
| 某个轮子转向反了 | `MOTOR_DIR_FL/FR/RL/RR` 对应那个改成 `-1` |
| 推杆向前车却后退 | `RC_VX_SIGN` 改 `-1` |
| 推杆向右车却向左平移 | `RC_VY_SIGN` 改 `+1` |
| 推杆向右车却逆时针转 | `RC_WZ_SIGN` 改 `+1` |
| 推左摇杆上下却是横移 | `RC_CH_VX` 和 `RC_CH_VY` 的值对调（2 和 3） |
| 转向用的是左摇杆 | 把 `RC_CH_WZ` 改成 2 或 3 |
| C620 的 ID 顺序不是 FL/FR/RL/RR | 改 `WHEEL_FL/FR/RL/RR` 四个下标 |
| 车实际轴距轮距和我填的不一样 | `HALF_WHEELBASE_M` / `HALF_TRACK_M` 按实测改 |
| 太快/太慢 | `MAX_LINEAR_SPEED_MPS` / `MAX_ANGULAR_SPEED_RADPS` |

## 7. 速度环调参（`robot_config.h` 第 6 节）

输入是 rpm 误差，输出是 C620 电流原始值（16384 ≈ 20A）。默认 `kp=6, ki=10, kd=0`。

1. 先把 `SPD_PID_KI` 设为 0，从小往大加 `SPD_PID_KP`，加到轮子响应快、
   不啸叫、不抖为止（一般 4~15）。
2. 再加 `SPD_PID_KI` 消除稳态误差（一般 5~30）。加太大车会"点头"或低频摆动。
3. `SPD_PID_KD` 一般保持 0；轮速环噪声大，加了反而抖。
4. 温度超过 80℃ 会自动降额，95℃ 直接停转；想改看 `MOTOR_TEMP_*`。

## 8. 内置安全逻辑

- 遥控 100ms 收不到帧 → 立即停机（`RC_OFFLINE_TIMEOUT_MS`）。
- 任意一个电机 100ms 没反馈 → 整车停机（`MOTOR_OFFLINE_TIMEOUT_MS`）。
- S1 上档 / 上电默认 → 停机，电流给 0。
- 目标轮速有斜坡限幅（`WHEEL_ACCEL_RPM_PER_S`），避免电流冲击。
- 电流硬限幅 `MAX_CURRENT_RAW`。
- DBUS 串口错误（校验/噪声/溢出）后会自动重启接收，不会永久掉线。

## 9. ⚠ CubeMX 重新生成代码前必读

下面这些改动在 **USER CODE 区之外**，重新生成会被覆盖：

1. **`Core/Src/fdcan.c`** 里的
   `StdFiltersNbr=1`、`RxFifo0ElmtsNbr=8`、`TxFifoQueueElmtsNbr=8`、
   `FDCAN1_ConfigBitRate()` 调用，以及 MspInit 里的 `RCC_FDCANCLKSOURCE_HSE`。
   → 请在 CubeMX 里改成同样的值：FDCAN1 → Std Filters Nbr=1，
   Rx FIFO0 = 8，Tx FIFO Queue = 8；时钟源 HSE；
   位时序 Prescaler=1 / TimeSeg1=19 / TimeSeg2=4 / SJW=4（24MHz→1Mbps）。
   否则重新生成后 CAN 直接不通（ConfigFilter 失败会进 Error_Handler）。
2. **`STM32H723xG_flash.ld`** 里的 `.dma_buffer` 段（放在 RAM_D1）。
   STM32H7 的 DTCM 不能被 DMA 访问，串口 DMA 缓冲必须放在 RAM_D1。
   → 重新生成后检查这个段还在不在，不在的话 DBUS 会收不到数据。
3. `Core/Src/usart.c` 里把 UART5 停止位强制成 1 个（`USER CODE` 区，安全）。
   建议同时在 CubeMX 里把 UART5 的 Stop Bits 改成 1。

其他新加的代码都在 `USER CODE` 区或独立文件里，重新生成不会丢
（`Core/Src/main.c` 的初始化、`chassis.c` / `motor.c` / `remote.c` / `pid.c`）。
