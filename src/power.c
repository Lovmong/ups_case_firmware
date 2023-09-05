#include "power.h"

/* ----- 电源管理相关io初始化 -----


*/
void PowerIoInit() {
    // output 推挽输出
    P3M0 |= 0x10; P3M1 &= ~0x10; // EN, P3.4 推挽输出
    P3M0 |= 0x40; P3M1 &= ~0x40; // PWR, P3.6 推挽输出
    // input 高阻输入
    P1M0 &= ~0x80; P1M1 |= 0x80; // ALWAYS_ON, P1.7 高阻输入
    P5M0 &= ~0x10; P5M1 |= 0x10; // RPI_STATE, P5.4 高阻输入 
    P3M0 &= ~0x08; P3M1 |= 0x08; // CHG, P3.3 高阻输入
    P3M0 &= ~0x20; P3M1 |= 0x20; // Power_Source, P3.5 高阻输入
    // ADC
    P3M0 &= ~0x01; P3M1 |= 0x01; // OUTPUT_CURRENT, P3.0 高阻输入
    P3M0 &= ~0x02; P3M1 |= 0x02; // BATTERY_CURRENT, P3.1 高阻输入

}

/* ---- 维持电池给单片机供电 ---- 
PWR  维持高电平

*/
void PowerInHold() {
    P3M0 |= 0x40; P3M1 &= ~0x40; // PWR, P3.6 推挽输出
    PWR = 1; // 维持高电平， 维持给单片机供电
}


/* ---- 输出5v ---

*/
void PowerOutOpen() {
    P3M0 |= 0x10; P3M1 &= ~0x10; // EN, P3.4 推挽输出
    EN = 1;
}


void PowerOutClose() {
    P3M0 |= 0x10; P3M1 &= ~0x10; // EN, P3.4 推挽输出
    EN = 0;
}

/* ----- 上电时电源输出 -----
当 ALWAYS_ON 为有效时（低电平）时，power 输出(),
否则：
    读取eeprom值
    如果值为 0x01，power不输出
    如果值为 0x02，power不输出
    如果值错误，默认power不输出，写入 0x01
*/
void PowerManagerAtStart() {
    char stat = 0;
    
    P1M0 |= 0x80; P1M1 &= ~0x80; // ALWAYS_ON, P1.7 高阻输入
    P3M0 |= 0x10; P3M1 &= ~0x10; // EN, P3.4 推挽输出 

    if (ALWAYS_ON == 0) {
        delayUs(500); // 消抖
        if (ALWAYS_ON == 0) {
            EN = 1; // power输出
            return;
        }
    }

    stat = IapRead(POWER_MEMORY_ADDR);
    // EEPROM 的写操作只能将字节中的 1 写为 0，当需要将字节中的 0 写为 1，则必须执行扇区 擦除操作。
    switch (stat) {
        case 0x01: // 关闭输出
            EN = 0;
            break;
        case 0x02: // 开启输出
            EN = 1;
            break;
        default: // 默认power不输出，写入 0x01
            EN = 0;
            IapErase(POWER_MEMORY_ADDR);
            IapProgram(POWER_MEMORY_ADDR, 0x01);
            break;
    }

}

/* ----- 低电量时 -----

*/
void PowerManagerInLowBattery() {

}

/* ----- 充电管理 ----- 

*/
void ChargeManager() {

}

/* ----- 电源数据读取 -----

*/
void PowerMonitor() {

}