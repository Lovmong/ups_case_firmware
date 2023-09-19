#ifndef __BTN_H
#define __BTN_H

#include "stc8h.h"
#include "types.h"
#include "delay.h"

// https://blog.csdn.net/weixin_46251230/article/details/126659673

#define BTN P33

// // 定时器周期 = 12*（65535 - 重装载值）/ 系统工作频率
// // 16位定时器 65535，系统工作频率 12MHz, 默认12T模式（每12个时钟计数+1）
// // 周期 = 12*（6,5535 - 60,535）/12,000,000 = 0.005 s , 5ms
// //
// #define TIME0_CR 60535 // 重装载值

void btnInit();
bool btnIsPressed(void);
bool btnIsReleased(void);
#endif
