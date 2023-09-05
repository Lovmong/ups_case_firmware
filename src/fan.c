#include "fan.h"


/* ----------------------------------------------------------------
PWMx_ENO : 输出使能寄存器
PWMA_ENO 
      7     6     5     4     3     2     1     0
    ENO4N ENO4P ENO3N ENO3P ENO2N ENO2P ENO1N ENO1P
PWMB_ENO 
      7     6     5     4     3     2     1     0
      -    ENO8P  -    ENO7P  -    ENO6P  -   ENO5P

输出附加使能寄存器（PWMx_IOAUX）

控制寄存器 1（PWMx_CR1）
PWMA_CR1
        7     6   5    4   3     2    1    0
     ARPEA CMSA[1:0] DIRA OPMA URSA UDISA CENA
PWMB_CR1
        7     6   5    4   3     2    1    0
     ARPEB CMSB[1:0] DIRB OPMB URSB UDISB CENB

控制寄存器 2（PWMx_CR2）
从模式控制寄存器(PWMx_SMCR)
外部触发寄存器(PWMx_ETR)
中断使能寄存器(PWMx_IER)
状态寄存器 1(PWMx_SR1)
状态寄存器 2(PWMx_SR2)
事件产生寄存器（PWMx_EGR）
捕获/比较模式寄存器 1（PWMx_CCMR1）
捕获/比较模式寄存器 2（PWMx_CCMR2）
捕获/比较模式寄存器 3（PWMx_CCMR3）
捕获/比较模式寄存器 4（PWMx_CCMR4）
捕获/比较使能寄存器 1（PWMx_CCER1）
捕获/比较使能寄存器 2（PWMx_CCER2）
计数器高 8 位（PWMx_CNTRH）
计数器低 8 位（PWMx_CNTRL）
预分频器高 8 位（PWMx_PSCRH）
预分频器低 8 位（PWMx_PSCRL）
自动重装载寄存器高 8 位（PWMx_ARRH）
自动重装载寄存器低 8 位（PWMx_ARRL）
重复计数器寄存器（PWMx_RCR）
捕获/比较寄存器 1/5 高 8 位（PWMx_CCR1H）
捕获/比较寄存器 1/5 低 8 位（PWMx_CCR1L）
捕获/比较寄存器 2/6 高 8 位（PWMx_CCR2H）
捕获/比较寄存器 2/6 低 8 位（PWMx_CCR2L）
捕获/比较寄存器 3/7 高 8 位（PWMx_CCR3H）
捕获/比较寄存器 3/7 低 8 位（PWMx_CCR3L）
捕获/比较寄存器 4/8 高 8 位（PWMx_CCR4H）
捕获/比较寄存器 4/8 低 8 位（PWMx_CCR4L）
刹车寄存器（PWMx_BKR）
死区寄存器（PWMx_DTR）
输出空闲状态寄存器（PWMx_OISR）
---------------------------------------------------------------- */
void fanInit() {
    // P1M0 |= 0x01; P1M1 &= ~0x01;  // P1.0 推挽输出
    P1M0 &= ~0x01; P1M1 &= ~0x01; // P1.0 准双向口
    P1M0 &= ~0x04; P1M1 &= ~0x04; 

    // P1.0 PWM1P
#if USE_PWM
    P_SW2 |= 0x80;  //扩展寄存器(XFR)访问使能

    PWMA_CCER1 = 0x00;                          //写CCMRx前必须先清零CCERx关闭通道
    PWMA_CCMR1 = 0x68;                          //设置CC1为PWMA输出模式 1, 使能预装载功能
    // PWMA_CCER1 = 0x01;                          //使能CC1通道
    // PWMA_CCER1 = 0x55;                          //使能CC1通道
    PWMA_CCMR2 = 0x68;
    PWMA_CCER1 = 0x31;                          //使能CC1, CC2通道

    // pwm频率 = 系统时钟/（（预分频+1）/ （自动重装载值arr+1））= 12MHz / 12 / 20,000 = 50Hz
    // pwm频率 = 系统时钟/（（预分频+1）/ （自动重装载值arr+1））= 12MHz / 2 / 240 = 25KHz
    PWMA_PSCRH = (u8)((PWM_PSCR-1) >> 8); // 设置预分频
    PWMA_PSCRL = (u8)(PWM_PSCR-1);
    PWMA_ARRH = (u8)((PWM_PERIOD-1) >> 8); //设置周期时间， PWMA_ARR自动重装值
    PWMA_ARRL = (u8)(PWM_PERIOD-1);

    PWMA_CCR1H = (u8)(0 >> 8);         //设置占空比时间
    PWMA_CCR1L = (u8)(0);

    PWMA_CCR2H = (u8)(0 >> 8);         //设置占空比时间
    PWMA_CCR2L = (u8)(0);

    PWMA_ENO |= 0x01;                              //使能PWM1P端口输出
    PWMA_ENO |= 0x04;                              //使能PWM2P端口输出

    // PWMA_IOAUX &= ~0x01;                           // PWM1P 的输出直接由 ENO1P 控制

    PWMA_BKR = 0x80;                            //使能主输出
    PWMA_CR1 = 0x01;                            //使能计数器，开始计时
#endif
} 


void fanSetSpeed(u8 speed) {
#if USE_PWM
    u16 ccr = 0;
    ccr = ((u32)PWM_PERIOD * speed/100);
    P_SW2 |= 0x80;
    // 更新占空比
    PWMA_CCR1H = (u8)(ccr >> 8);
    PWMA_CCR1L = (u8)(ccr);

    PWMA_CCR2H = (u8)(ccr >> 8);
    PWMA_CCR2L = (u8)(ccr);
#else
    if (speed == 0) FAN = 0;
    else FAN = 1;
#endif
}



