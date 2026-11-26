/**
  ******************************************************************************
  * @file    remote.c
  * @brief   DJI DR16 遥控器 DBUS 接收（串口 DMA + 空闲中断）
  *
  *          ★ 为什么要用"空闲中断 + 取最后 18 字节"这种写法：
  *            1. DBUS 帧没有帧头，不能靠找帧头来对齐；
  *            2. 上电时 DMA 可能正好从一帧的中间开始收，缓冲区会永久错位；
  *            3. 串口空闲（IDLE）中断一定发生在两帧之间的空隙里，
  *               所以"IDLE 时刻 DMA 写指针往回数 18 字节"就一定是完整的一帧，
  *               每一帧都重新对齐，丢字节也能自己恢复。
  *
  *          ★ DMA 缓冲区必须放在 DMA 能访问的 RAM 里！
  *            STM32H7 的 DTCM(0x20000000) 只有 CPU 能访问，DMA1/DMA2 访问不到，
  *            所以这里用 section(".dma_buffer")，由链接脚本放到 RAM_D1(0x24000000)。
  ******************************************************************************
  */

#include "remote.h"
#include "usart.h"

#define DBUS_DMA_BUF_LEN    (DBUS_FRAME_LEN * 4U)   /* 4 帧的环形缓冲 */

/* 通道合法范围：中值 1024 ± 660，再留一点余量 */
#define DBUS_CH_MIN         264
#define DBUS_CH_MAX         1784

Remote_t remote;

/* 放在 RAM_D1，DMA1 可以直接写 ---------------------------------------------*/
static uint8_t s_dma_buf[DBUS_DMA_BUF_LEN] __attribute__((section(".dma_buffer"), aligned(32)));

/* 0 = 还没标定帧边界（上电后的第一帧直接丢掉） */
static uint8_t s_frame_synced = 0U;

/* ------------------------------------------------------------------------- */
static uint8_t DBUS_ChannelValid(uint16_t ch)
{
    return ((ch >= DBUS_CH_MIN) && (ch <= DBUS_CH_MAX)) ? 1U : 0U;
}

/**
  * @brief  解析 18 字节 DBUS 帧
  */
static void DBUS_Decode(const uint8_t *d)
{
    uint16_t c0;
    uint16_t c1;
    uint16_t c2;
    uint16_t c3;
    uint8_t  s1;
    uint8_t  s2;

    /* 4 个摇杆，每个 11bit，高位在前、跨字节 */
    c0 = (uint16_t)(((uint16_t)d[0] | ((uint16_t)d[1] << 8)) & 0x07FFU);
    c1 = (uint16_t)(((uint16_t)(d[1] >> 3) | ((uint16_t)d[2] << 5)) & 0x07FFU);
    c2 = (uint16_t)(((uint16_t)(d[2] >> 6) | ((uint16_t)d[3] << 2) |
                     ((uint16_t)d[4] << 10)) & 0x07FFU);
    c3 = (uint16_t)(((uint16_t)(d[4] >> 1) | ((uint16_t)d[5] << 7)) & 0x07FFU);

    s1 = (uint8_t)((d[5] >> 4) & 0x03U);
    s2 = (uint8_t)(((d[5] >> 4) & 0x0CU) >> 2);

    /* 帧错位 / 丢字节时的保护：摇杆和拨杆都得在合法范围里 */
    if ((DBUS_ChannelValid(c0) == 0U) || (DBUS_ChannelValid(c1) == 0U) ||
        (DBUS_ChannelValid(c2) == 0U) || (DBUS_ChannelValid(c3) == 0U) ||
        (s1 == 0U) || (s2 == 0U))
    {
        remote.err_cnt++;
        return;
    }

    remote.ch[0] = (int16_t)c0 - 1024;
    remote.ch[1] = (int16_t)c1 - 1024;
    remote.ch[2] = (int16_t)c2 - 1024;
    remote.ch[3] = (int16_t)c3 - 1024;

    for (uint8_t i = 0; i < 4U; i++)
    {
        remote.nch[i] = (float)remote.ch[i] / 660.0f;
        if (remote.nch[i] > 1.0f)
        {
            remote.nch[i] = 1.0f;
        }
        if (remote.nch[i] < -1.0f)
        {
            remote.nch[i] = -1.0f;
        }
    }

    remote.s1 = s1;
    remote.s2 = s2;

    remote.mouse_x = (int16_t)(((uint16_t)d[7] << 8) | d[6]);
    remote.mouse_y = (int16_t)(((uint16_t)d[9] << 8) | d[8]);
    remote.mouse_z = (int16_t)(((uint16_t)d[11] << 8) | d[10]);
    remote.mouse_l = d[12];
    remote.mouse_r = d[13];
    remote.key     = (uint16_t)(((uint16_t)d[15] << 8) | d[14]);

    remote.online       = 1U;
    remote.last_rx_tick = HAL_GetTick();
    remote.frame_cnt++;
}

/* 导出函数 ----------------------------------------------------------------*/

#if DIAG_PIN_SCAN
PinDiag_t pin_diag;

static void count_edges(uint32_t diff, uint32_t *cnt)
{
    while (diff != 0U)
    {
        uint32_t bit = (uint32_t)__builtin_ctz(diff);
        cnt[bit]++;
        diff &= (diff - 1U);
    }
}

/**
  * @brief  引脚活动扫描（诊断用，上电执行一次）
  *
  *         ★ 关键：STM32H7 复位后所有 GPIO 默认是【模拟模式】，
  *           模拟模式下输入缓冲器关闭、IDR 读不到外部电平。
  *           所以扫描期间要把 A~E 全部临时切成"数字输入 + 下拉"，
  *           扫完再把原来的 MODER/PUPDR/AFR 全部恢复。
  *           SWD(PA13/PA14) 和 CAN(PD0/PD1) 保持原样，避免断调试/断总线。
  */
void Remote_DiagScanPins(void)
{
    GPIO_TypeDef *const ports[5] = {GPIOA, GPIOB, GPIOC, GPIOD, GPIOE};
    uint32_t save_moder[5];
    uint32_t save_pupdr[5];
    uint32_t save_afr0[5];
    uint32_t save_afr1[5];
    uint32_t prev[5];
    uint32_t cur;
    uint32_t keep;
    uint32_t p;
    uint32_t n;

    /* 串口可能落在任意端口上，先把时钟都打开 */
    __HAL_RCC_GPIOC_CLK_ENABLE();
    __HAL_RCC_GPIOE_CLK_ENABLE();

    for (p = 0U; p < 5U; p++)
    {
        save_moder[p] = ports[p]->MODER;
        save_pupdr[p] = ports[p]->PUPDR;
        save_afr0[p]  = ports[p]->AFR[0];
        save_afr1[p]  = ports[p]->AFR[1];

        keep = 0U;
        if (ports[p] == GPIOA)
        {
            keep = (3UL << 26) | (3UL << 28);       /* PA13 SWDIO, PA14 SWCLK */
        }
        if (ports[p] == GPIOD)
        {
            keep = (3UL << 0) | (3UL << 2);         /* PD0 FDCAN1_RX, PD1 FDCAN1_TX */
        }

        ports[p]->MODER = save_moder[p] & keep;                            /* 其余全切输入 */
        ports[p]->PUPDR = (save_pupdr[p] & keep) | (0xAAAAAAAAUL & ~keep); /* 其余全下拉   */
    }

    for (p = 0U; p < 5U; p++)
    {
        prev[p] = ports[p]->IDR;
    }

    for (n = 0U; n < DIAG_SCAN_ITER; n++)
    {
        cur = ports[0]->IDR;
        count_edges((cur ^ prev[0]) & 0xFFFFU, pin_diag.a);
        prev[0] = cur;

        cur = ports[1]->IDR;
        count_edges((cur ^ prev[1]) & 0xFFFFU, pin_diag.b);
        prev[1] = cur;

        cur = ports[2]->IDR;
        count_edges((cur ^ prev[2]) & 0xFFFFU, pin_diag.c);
        prev[2] = cur;

        cur = ports[3]->IDR;
        count_edges((cur ^ prev[3]) & 0xFFFFU, pin_diag.d);
        prev[3] = cur;

        cur = ports[4]->IDR;
        count_edges((cur ^ prev[4]) & 0xFFFFU, pin_diag.e);
        prev[4] = cur;
    }

    if ((GPIOB->IDR & (1UL << 12)) != 0U)
    {
        pin_diag.b12_high++;
    }
    else
    {
        pin_diag.b12_low++;
    }
    pin_diag.scan_pass++;

    /* 恢复原来的引脚配置 */
    for (p = 0U; p < 5U; p++)
    {
        ports[p]->MODER = save_moder[p];
        ports[p]->PUPDR = save_pupdr[p];
        ports[p]->AFR[0] = save_afr0[p];
        ports[p]->AFR[1] = save_afr1[p];
    }
}
#endif /* DIAG_PIN_SCAN */

void Remote_Init(void)
{
    remote.ch[0] = remote.ch[1] = remote.ch[2] = remote.ch[3] = 0;
    for (uint8_t i = 0; i < 4U; i++)
    {
        remote.nch[i] = 0.0f;
    }
    remote.s1 = 0U;
    remote.s2 = 0U;
    remote.online       = 0U;
    remote.last_rx_tick = 0U;
    remote.frame_cnt    = 0U;
    remote.err_cnt      = 0U;

    s_frame_synced = 0U;

#if DIAG_PIN_SCAN
    Remote_DiagScanPins();
#endif

    /* 循环 DMA，只启动一次，之后 HAL 自己会一直收 */
    if (HAL_UARTEx_ReceiveToIdle_DMA(&huart5, s_dma_buf, DBUS_DMA_BUF_LEN) != HAL_OK)
    {
        Error_Handler();
    }
}

uint8_t Remote_IsOnline(void)
{
    /* 监督：万一因为错误中断把 DMA 接收停掉了，这里自动重新启动。
       1kHz 调用，最多 1ms 就能恢复，车不会因为一次噪声永久失控。 */
    if (huart5.RxState != HAL_UART_STATE_BUSY_RX)
    {
        s_frame_synced = 0U;
        (void)HAL_UARTEx_ReceiveToIdle_DMA(&huart5, s_dma_buf, DBUS_DMA_BUF_LEN);
    }

    if (remote.online == 0U)
    {
        return 0U;
    }

    if ((HAL_GetTick() - remote.last_rx_tick) > RC_OFFLINE_TIMEOUT_MS)
    {
        remote.online = 0U;
    }

    return remote.online;
}

/* 串口空闲 / DMA 接收完成回调 ---------------------------------------------*/
void HAL_UARTEx_RxEventCallback(UART_HandleTypeDef *huart, uint16_t Size)
{
    uint16_t remaining;
    uint16_t pos;
    uint8_t  frame[DBUS_FRAME_LEN];

    (void)Size;

    /* 只关心 UART5 */
    if (huart->Instance != UART5)
    {
        return;
    }

    /* 只处理 IDLE 事件：DMA 半满 / 全满回调是环形的，不能当边界用 */
    if (huart->RxEventType != HAL_UART_RXEVENT_IDLE)
    {
        return;
    }

    if (huart->hdmarx == 0)
    {
        return;
    }

    /* DMA 还差多少字节写完整个缓冲区 -> 反推出当前写指针位置 */
    remaining = (uint16_t)__HAL_DMA_GET_COUNTER(huart->hdmarx);
    if (remaining > DBUS_DMA_BUF_LEN)
    {
        remaining = 0U;
    }
    pos = (uint16_t)(DBUS_DMA_BUF_LEN - remaining);

    /* 上电第一帧只用来标定边界，内容可能是不完整的，直接丢 */
    if (s_frame_synced == 0U)
    {
        s_frame_synced = 1U;
        return;
    }

    /* 取写指针前面 18 个字节，跨过缓冲区末尾时绕回去 */
    for (uint16_t i = 0; i < DBUS_FRAME_LEN; i++)
    {
        frame[i] = s_dma_buf[(uint16_t)((pos + DBUS_DMA_BUF_LEN - DBUS_FRAME_LEN + i)
                                        % DBUS_DMA_BUF_LEN)];
    }

    DBUS_Decode(frame);
}

/* 串口错误回调 -------------------------------------------------------------
 * 校验错 / 帧错 / 噪声 / 溢出时，HAL 会把 DMA 接收整个中止掉，
 * 之后再也收不到数据（遥控会一直显示掉线）。这里重新启动接收。
 */
void HAL_UART_ErrorCallback(UART_HandleTypeDef *huart)
{
    if (huart->Instance != UART5)
    {
        return;
    }

    /* 重新对齐：丢掉下一帧不完整的数据 */
    s_frame_synced = 0U;

    (void)HAL_UARTEx_ReceiveToIdle_DMA(&huart5, s_dma_buf, DBUS_DMA_BUF_LEN);
}
