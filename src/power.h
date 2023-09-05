#ifndef __POWER_H
#define __POWER_H

#include "stc8h.h"
#include "eeprom.h"
#include "delay.h"

#define POWER_MEMORY_ADDR 0x0400  // 掉电记忆eeprom地址

#define ALWAYS_ON P17  // Always_ON 引脚 P1.7
#define PWR P36 // PWR 引脚 P3.6, 开机时维持供电，保持高电平
#define RPI_STATE P54 // RPI_STATE 引脚 P5.4
#define CHG P33 // CHG
#define EN P34 // EN 树莓派供电开关
#define Power_Source P35 // Power Source h 电池
#define OUTPUT_CURRENT P30 // 电流, val /100 / 0.05
#define BATTERY_CURRENT P31 // 电压

void PowerIoInit();
void PowerInHold();
void PowerOutOpen();
void PowerOutClose();
void PowerManagerAtStart();
void PowerManagerInLowBattery();
void ChargeManager();
void PowerMonitor();

#endif

