#ifndef __POWER_H
#define __POWER_H

#include "stc8h.h"
#include "eeprom.h"
#include "delay.h"
#include "intrins.h"
/*---------------------------------------------------------------
    output:
        PWR -> P3.2
        EN  -> P0.0
    input:
        BTN -> P3.3
        CHG -> P0.1
        POWER_SOURCE -> P1.3
        RPI_STATE (pi gpio 6)-> P2.3
        ALWAYS_ON -> P2.5
    dac: （PWM1P）
        DAC -> P1.0
    adc:
        VBUS_3V3 -> ADC6 -> P1.6
        BT_LV_3V3 -> ADC7 -> P1.7
        BATTERY_CURRENT -> ADC10 -> P0.2
        OUTPUT_CURRENT -> ADC11 -> P0.3
---------------------------------------------------------------*/
#define POWER_MEMORY_ADDR 0x0400                    // 掉电记忆eeprom地址
#define MAIN_Fosc 12000000L     //定义主时钟 12MHz

// 注意 dac 和 fan 都使用 PWMA, 初始化时 预分频 和 自动重装载值要一致，或者仅初始化一次
#define _PWMA_PSCR 12           // 预分频 12 
#define _PWMA_PERIOD 20000      // PWM周期 (PWMA_ARR 自动重装载值) 20,000

#define PWR P32                                     // PWR 引脚 P3.2, 开机时维持供电，保持高电平
#define EN P00                                      // EN，P0.0 树莓派供电开关(5v输出开关)

#define CHG P01                                     // CHG, P0.1 是否在充电，高电平有效
#define Power_Source P13                            // Power Source, P1.3 是否使用电池供电，高电平有效
#define RPI_STATE P23                               // RPI_STATE, P2.3 树莓派开机状态，高电平为开机，低电平关机
#define ALWAYS_ON P25                               // Always_ON，P2.5 树莓派供电开关(5v输出开关)是否常开, 低电平有效

#define DAC P10                                     // DAC，P1.0, 电池充电电流控制，0~1.5v, 1.5v时充电电流最大

#define VBUS_3V3 P16                                // VBUS_3V3, p1.6 USB输入电压
#define BT_LV_3V3 P17                               // BT_LV_3V3, P1.7 电池输入电压 
#define BATTERY_CURRENT P02                         // BATTERY_CURRENT, P0.2 电池输出电流
#define OUTPUT_CURRENT P03                          // OUTPUT_CURRENT, P0.3 当前输出电流, val /100 / 0.05

#define kp 0.1                                      // 充电电流调节pid参数
#define ki 0.0
#define kd 0.0
#define targetVoltage 4900
#define maxValue 1500

#define BATTERY_VOLTAGE_CHANNEL 0x07
#define BATTERY_CURRENT_CHANNEL 0x06
#define USB_VOLTAGE_CHANNEL 0x10
#define OUTPUT_CURRENT_CHANNEL 0x11

#define BATTERY_VOLTAGE_GAIN ((float)(120+51)/51.0)
#define USB_VOLTAGE_GAIN ((float)(120+120)/120.0)



void PowerIoInit();
void PowerInHold();
void PowerOutOpen();
void PowerOutClose();
void PowerManagerAtStart();

void AdcInit();
u16 AdcRead(u8 channel);
u16 UsbVoltageRead();
u16 BatteryVoltageRead();
u16 BatteryCurrentRead();
u16 OutputCurrentRead();
void ChargeManager();
void dacInit();
void setDac(u16 voltage);
u16 pidCalculate(u16 current_vol);
u16 calculateBatteryIR();
u8 UpdateBatteryPercentage();

void PowerManagerInLowBattery();
void PowerMonitor();

#endif

