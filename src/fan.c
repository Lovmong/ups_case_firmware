#include "fan.h"

/*---------------------------------------------------------------
    fan: (PWMA)
        FAN -> PWM3P_2 -> P2.4
        (注意不是默认的PWM引脚，需要使用 PWMA_PS 寄存器 选择引脚)
---------------------------------------------------------------*/
void fanInit()
{
    P2M0 |= 0x10;
    P2M1 &= ~0x10;

    // P2M0 |= 0x10;
    // P2M1 &= ~0x10; // P2.4 推挽输出
    // P2M0 &= ~0x10; P2M1 &= ~0x10;    // P2.4 准双向口
    FAN = 0;

#if FAN_USE_PWM
    // PWM3P P2.4
    P_SW2 |= 0x80; // 使能扩展寄存器(XFR)访问

    PWMA_PS |= 0x10; // 设置PWM3 输出脚为 P2.4, P2.5

    PWMA_CCER1 = 0x00; // 写CCMRx前必须先清零CCERx关闭通道
    PWMA_CCMR3 = 0x68; // 设置PWM3输出模式 1, 使能预装载功能
    PWMA_CCER2 = 0x01; // 使能PWM3P通道, 极性为 高电平有效

    PWMA_PSCRH = (u8)((_PWMA_PSCR - 1) >> 8); // 设置PWMA预分频
    PWMA_PSCRL = (u8)(_PWMA_PSCR - 1);
    PWMA_ARRH = (u8)((_PWMA_PERIOD - 1) >> 8); // 设置周期时间， PWMA_ARR自动重装值
    PWMA_ARRL = (u8)(_PWMA_PERIOD - 1);

    PWMA_CCR3H = (u8)(0 >> 8); // 初始化PWM3占空比时间
    PWMA_CCR3L = (u8)(0);

    PWMA_ENO |= 0x10; // 使能PWM3P输出

    // PWMA_IOAUX &= ~0x10;                        // PWM3P 的输出直接由 ENO3P 控制

    PWMA_BKR = 0x80; // 使能PWMA主输出
    PWMA_CR1 = 0x01; // 使能PWMA计数器，开始计时, (00 边沿对齐模式)

#endif
}

void fanSetSpeed(u8 speed)
{
#if FAN_USE_PWM
    u16 ccr = 0;
    ccr = ((u32)_PWMA_PERIOD * speed / 100);

    P_SW2 |= 0x80;               // 使能扩展寄存器(XFR)访问
    PWMA_CCR3H = (u8)(ccr >> 8); // 设置PWM3占空比
    PWMA_CCR3L = (u8)(ccr);
#else
    if (speed == 0)
        FAN = 0;
    else
        FAN = 1;
#endif
}
