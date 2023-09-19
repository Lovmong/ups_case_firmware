#include "btn.h"
#include "rgb.h"

// u8 timeCount = 0;
// u16 time0Count = 0;

void btnInit()
{
    P3M0 &= ~0x08;
    P3M1 |= 0x08; // 设置 P3.3 为高阻输入
    // P3M0 &= ~0x08; P3M1 &= ~0x08; // 设置 P3.3 为准双向口

    // -- 定时器0 (模式0， 16位自动重装载， 用做定时)
    //     TMOD = 0x00; // 模式0，用作定时器（对内部系统时钟进行计数）；16位自动重装载
    //     // TH0 = (u8)(TIME0_CR >> 8); // 配置重装载值，设置周期
    //     // TL0 = (u8)(TIME0_CR);
    //     TL0 = 0x66; // 65536-11.0592M/12/1000
    //     TH0 = 0xfc;
    //     TR0 = 1; // 启动定时器
    //     ET0 = 1; // 使能定时器中断
    //     EA = 1;
}

/** 定时器0 中断服务
 */
// void TM0_Isr() interrupt 1
// {
//     rgbWrite(0, 255, 0); // 工作指示
//     timeCount = 20;
//     // count++;
//     // if (count > 100)
//     // {
//     //     count = 0;
//     //     timeCount++;
//     // }
// }

bool btnIsPressed(void)
{
    if (BTN == 0)
    {
        delayMs(5); // 消抖
        if (BTN == 0)
            return 1;
        else
            return 0;
    }
    else
    {
        return 0;
    }
}

bool btnIsReleased(void)
{
    if (BTN == 1)
    {
        delayMs(5); // 消抖
        if (BTN == 1)
            return 1;
        else
            return 0;
    }
    else
    {
        return 0;
    }
}
