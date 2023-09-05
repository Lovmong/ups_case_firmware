#ifndef __FAN_H
#define __FAN_H

#include "stc8h.h"
#include "types.h"

#define FAN P10
#define USE_PWM 1

#define MAIN_Fosc 12000000L     //定义主时钟 12MHz

// pwm频率 = 系统时钟/（（预分频+1）/ （自动重装载值arr+1））= 12MHz / 12 / 20,000 = 50Hz
#define PWM_PSCR 12           // 预分频 12 
#define PWM_PERIOD 20000      // PWM周期 (PWMA_ARR自动重装值) 20,000

// pwm频率 = 系统时钟/（（预分频+1）/ （自动重装载值arr+1））= 12MHz / 2 / 240 = 25KHz
// #define PWM_PSCR 2           // 预分频 2 
// #define PWM_PERIOD 240      // PWM周期 (PWMA_ARR自动重装值) 240

// // pwm频率 = 系统时钟/（（预分频+1）/ （自动重装载值arr+1））= 12MHz / 2 / 600 = 10KHz
// #define PWM_PSCR 2          // 预分频 2 
// #define PWM_PERIOD 600      // PWM周期 (PWMA_ARR自动重装值) 600


// pwm频率 = 系统时钟/（（预分频+1）/ （自动重装载值arr+1））= 12MHz / 2 / 6000 = 1KHz
// #define PWM_PSCR 2          // 预分频 2 
// #define PWM_PERIOD 6000      // PWM周期 (PWMA_ARR自动重装值) 6000

// pwm频率 = 系统时钟/（（预分频+1）/ （自动重装载值arr+1））= 12MHz / 4 / 6000 = 500Hz
// #define PWM_PSCR 4          // 预分频 4
// #define PWM_PERIOD 6000      // PWM周期 (PWMA_ARR自动重装值) 6000

// pwm频率 = 系统时钟/（（预分频+1）/ （自动重装载值arr+1））= 12MHz / 24 / 25,000 = 20Hz
// #define PWM_PSCR 24           // 预分频 24 
// #define PWM_PERIOD 25000      // PWM周期 (PWMA_ARR自动重装值) 20,000

// pwm频率 = 系统时钟/（（预分频+1）/ （自动重装载值arr+1））= 12MHz / 16 / 10,000 = 75Hz
// #define PWM_PSCR 16           // 预分频 16 
// #define PWM_PERIOD 10000      // PWM周期 (PWMA_ARR自动重装值) 10,000

// pwm频率 = 系统时钟/（（预分频+1）/ （自动重装载值arr+1））= 12MHz / 24 / 2,500 = 200Hhz
// #define PWM_PSCR 24          // 预分频 24 
// #define PWM_PERIOD 2500      // PWM周期 (PWMA_ARR自动重装值) 2,500

// pwm频率 = 系统时钟/（（预分频+1）/ （自动重装载值arr+1））= 12MHz / 24 / 5,000 = 100Hhz
// #define PWM_PSCR 24          // 预分频 24 
// #define PWM_PERIOD 5000     // PWM周期 (PWMA_ARR自动重装值) 5,000

void fanInit();
void fanSetSpeed(u8 speed);

#endif
