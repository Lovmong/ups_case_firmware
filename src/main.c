/*============================================================================

STC8H1K28:
    - 32个引脚

引脚连接:
    output:
        PWR -> P3.2
        EN  -> P0.0
    input:
        BTN -> P3.3
        CHG -> P0.1
        POWER_SOURCE -> P1.3
        RPI_STATE (pi gpio 6)-> P2.3
        ALWAYS_ON -> P2.5
    dac:
        DAC -> P1.0
    adc:
        VBUS_3V3 -> P1.6
        BT_LV_3V3 -> P1.7
        BATTERY_CURRENT -> P0.2
        OUTPUT_CURRENT -> P0.3
    i2c:
        SDA -> P1.4
        SCL -> P1.5
    rgb: (PWMB)
        LED_B -> PWM5 -> P2.0
        LED_G -> PWM6 -> P2.1
        LED_R -> PWM7 -> P2.2
    fan: (PWMA)
        FAN -> PWM3P_2 -> P2.4
    uart:
        TXD -> P3.1
        RXD -> P3.0

==============================================================================*/

#include "stc8h.h"
#include "intrins.h"

#include "types.h"
#include "delay.h"

#include "btn.h"
#include "rgb.h"
#include "fan.h"

#include "power.h"

#include "test.h"

/*---------------------------------------------------------------
    i2c: (默认i2c1)
        SDA -> P1.4
        SCL -> P1.5

    STC8H 系列 i2c 的中断号为 24
---------------------------------------------------------------*/
#define I2C_SLAVE_ADDRESS 0x5A
#define SDA P14
#define SCL P15

bit isda;       // 设备地址标志
bit isma;       // 存储地址标志
bit islock;     // 数据同步锁
u8 idata addr;  // i2c buffer address
u8 readOrWrite; //

u8 idata dataBuffer[] = {
    0, // 0  电池电压高位 mV
    1, // 1  电池电压低位 mV

    2, // 2  电池电流高位 mA
    3, // 3  电池电流低位 mA

    4, // 4  usb电压高位 mV
    5, // 5  usb电压低位 mV

    6, // 6  5V输出电流高位 mA
    7, // 7  5V输出电流低位 mA

    8, // 8  内部参考电压高位 mV
    9, // 9  内部参考电压地位 mV

    10, // 10  Power_Source, 0 usb 供电，1 电池供电

    11, // 11  is_usb_plugged_in , usb是否插入
    12, // 12  CHG, 0 电池未充电， 1 正在充电
    13, // 13  风扇速度 0 ~ 100
    14, // 14  batteryPercentage

    15, // 15  ShutdownRequest 关机请求 0，不请求； 1，低电量关机请求； 2，按键关机请求
    // 0, // 14  batteryCapacity 高位
    // 0, // 15  batteryCapacity 低位
    // 0, // 16  dac_vol_H
    // 0, // 17  dac_vol_L
    // 0, // 18  pidOutput_H
    // 0, // 19  pidOutput_L
};

u8 xdata controlBuffer[] = {
    0, // 风扇速度 0 ~ 100
};
u8 idata fanSpeed = 0;
u8 idata lastFanSpeed = 0;

// ------------ I2C_Init------------------------
void I2C_Init()
{
    P1M0 &= ~0x30;
    P1M1 &= ~0x30; // P1.4, P1.5 准双向口

    P_SW2 |= 0x80; // 使能扩展寄存器(XFR)访问
    // P_SW2 |= 0x10;                              //切换i2c引脚

    I2CCFG = 0x81;                              // 使能I2C从机模式
    I2CSLADR = (I2C_SLAVE_ADDRESS << 1) & 0xFE; // 设置从机设备地址, MA=0
    I2CSLST = 0x00;                             // 复位从机状态寄存器
    I2CSLCR = 0x78;                             // 使能从机模式中断
    EA = 1;                                     // 使能全局中断

    isda = 1; // 用户变量初始化
    isma = 1;
    islock = 0;
    addr = 0;
    I2CTXD = dataBuffer[addr];
}

#if 1 // ------------ I2C_Isr (中断号 24)------------------------
void I2C_Isr() interrupt 24
{
    islock = 1;
    _push_(P_SW2);
    P_SW2 |= 0x80;

    // --------- START事件 ---------
    if (I2CSLST & 0x40)
    {
        I2CSLST &= ~0x40;
    }
    // --------- RECV事件 ---------
    else if (I2CSLST & 0x20)
    {
        I2CSLST &= ~0x20;
        if (isda)
        {                                // 处理RECV事件（RECV DEVICE ADDR）
            readOrWrite = I2CRXD & 0x01; // 0为写，1为读
            isda = 0;
            // dataBuffer[15] = I2CRXD;
        }
        else if (isma)
        { // 处理RECV事件（RECV MEMORY ADDR）
            isma = 0;
            addr = I2CRXD;
            I2CTXD = dataBuffer[addr];
        }
        else
        { // 处理RECV事件（RECV DATA）
            if (addr == 0 && I2CRXD != 181)
            {
                controlBuffer[0] = I2CRXD;
                fanSpeed = (u8)controlBuffer[0];
                dataBuffer[13] = fanSpeed;
                addr++;
            }
            //
            // if (readOrWrite == 1)
            // {
            //     // dataBuffer[13] = 100;
            // }
            // else
            // {
            //     dataBuffer[13] = I2CRXD;
            // }
        }
    }
    // --------- SEND事件 ---------
    else if (I2CSLST & 0x10)
    {
        I2CSLST &= ~0x10;
        if (I2CSLST & 0x02)
        { // 接收到NAK则停止读取数据
            I2CTXD = 0xff;
        }
        else
        { // 接收到ACK则继续读取数据
            I2CTXD = dataBuffer[++addr];
        }
    }
    // ---------STOP事件 ---------
    else if (I2CSLST & 0x08)
    {
        I2CSLST &= ~0x08;
        isda = 1;
        isma = 1;
    }

    _pop_(P_SW2);

    islock = 0;
}
#endif

// 按键状态机  定时器0
// =================================================================
// 定时器周期 = 12*（65535 - 重装载值）/ 系统工作频率
// 16位定时器 65535，系统工作频率 12MHz, 默认12T模式（每12个时钟计数+1）
// 周期 = 12*（6,5535 - 60,535）/12,000,000 = 0.005 s , 5ms
//
#define Time0Preiod 5  // 定时器0 周期 5ms
#define TIME0_CR 60535 // 重装载值

#define KeyDetectInterval 10   // ms 按键检测间隔
#define LongPressTime2S 2000   // ms 按键长按3S时长
#define LongPressTime3S 3000   // ms 按键长按3S时长
#define LongPressTime5S 5000   // ms 按键长按3S时长
#define DoulePressInterval 200 // ms 按键双击间隔

#define RpiShutdownCurrent 50 // 树莓派关机电流

u16 xdata scanTimeCount = 0;
u16 xdata longPressCount = 0;
u16 xdata doulePressCount = 0;
u8 xdata hasPressed = 0;

u8 xdata SafeShutdownColor[] = {109, 0, 107}; // purple
u8 xdata ForceShutdownColor[] = {200, 0, 0};  // red
u8 xdata ShutdownRequest = 0;
u16 xdata rgbTimeCount = 0;

enum // 定义状体机使用的枚举类型
{
    BTN_UP = (u8)0,         // 按键松开
    BTN_DOWN_SHAKE = (u8)1, // 按键按下抖动
    BTN_DOWN = (u8)2,       // 按键按下
    BTN_UP_SHAKE = 3,       // 按键松开抖动
} KeyMachineState;

enum
{
    Released = 0,                 // 按键松开
    SingleClicked = 1,            // 单击, 按下松开100ms（双击间隔）后未按下
    DouleClicked = 2,             // 双击， 按下松开100ms（双击间隔）内按下再松开
    LongPressed2S = 3,            // 长按2s， 按下2s后未松开
    LongPressed2SAndReleased = 4, // 长按2s， 按下2s后松开
    // LongPressed3S = 3, // 长按3s， 按下3s后未松开
    LongPressed5S = 5, // 长按5s， 按下5s后未松开

} KeyState;

/**
 定时器0初始化 (模式0， 16位自动重装载， 用做定时)
 */
void TM0_INIT()
{
    TMOD &= 0xF0;              // 定时器0，模式0，用作定时器（对内部系统时钟进行计数）；16位自动重装载
    TH0 = (u8)(TIME0_CR >> 8); // 配置重装载值，设置周期
    TL0 = (u8)(TIME0_CR);
    TR0 = 1; // 启动定时器
    ET0 = 1; // 使能定时器中断
    EA = 1;
}

/** 定时器0 中断服务
 */
void TM0_Isr() interrupt 1
{
    // 每次 计数 加 5ms
    scanTimeCount += Time0Preiod;
    longPressCount += Time0Preiod;
    doulePressCount += Time0Preiod;
    rgbTimeCount += Time0Preiod;
}

void KeyHandler()
{
    if (scanTimeCount > KeyDetectInterval)
    {
        // key detect
        switch (KeyMachineState)
        {
        case BTN_UP:
            if (BTN == 0)
            {
                KeyMachineState = BTN_DOWN_SHAKE;
            }
            else
            {
                if (hasPressed)
                {
                    if (doulePressCount > DoulePressInterval) // 单击：按下松开后200ms
                    {
                        KeyState = SingleClicked;
                        hasPressed = 0;
                    }
                }
                else
                {
                    KeyState = Released; // 松开
                }
            }
            break;
        case BTN_DOWN_SHAKE:
            if (BTN == 0)
            {
                KeyMachineState = BTN_DOWN;
                longPressCount = 0; // 按下计时器复位
            }
            else
            {
                KeyMachineState = BTN_UP;
            }
            break;
        case BTN_DOWN:
            if (BTN == 1)
            {
                KeyMachineState = BTN_UP_SHAKE;
            }
            else
            {
                if (longPressCount > LongPressTime5S)
                {
                    KeyState = LongPressed5S;
                }
                else if (longPressCount > LongPressTime2S)
                {
                    KeyState = LongPressed2S;
                }
            }
            break;
        case BTN_UP_SHAKE:
            if (BTN == 0)
            { // 如果为按下，视为抖动，回到上一状态
                KeyMachineState = BTN_DOWN;
            }
            else
            {
                KeyMachineState = BTN_UP;

                // 按下到松开
                if (KeyState == LongPressed2S)
                {
                    KeyState = LongPressed2SAndReleased;
                    hasPressed = 0;
                }
                else if (KeyState == LongPressed5S)
                {
                    // KeyState = LongPressed5SAndReleased;
                    hasPressed = 0;
                }
                else
                {
                    if (hasPressed)
                    {
                        KeyState = DouleClicked;
                        hasPressed = 0;
                    }
                    else
                    {
                        hasPressed = 1;
                        doulePressCount = 0;
                    }
                }
                longPressCount = 0;
            }
            break;
        default:
            KeyMachineState = BTN_UP;
            break;
        }

        // --- 按键处理程序 ---
        switch (KeyState)
        {
        case SingleClicked: // 单击开启输出
            PowerOutOpen();
            rgbWrite(0, 255, 0);
            delayMs(200);
            break;
        case DouleClicked:
            rgbWrite(0, 0, 255);
            delayMs(200);
            break;
        case LongPressed2S: // 长按的rgb显示需要放在RgbHandler中，避免灯显示冲突
            // rgbWrite(255, 120, 0);
            // delayMs(200);
            break;
        case LongPressed2SAndReleased:
            rgbWrite(SafeShutdownColor[0], SafeShutdownColor[1], SafeShutdownColor[2]);
            ShutdownRequest = 2; // 2，按键关机请求
            dataBuffer[15] = ShutdownRequest;
            delayMs(200);
            // while (1)
            // {
            //     // 等待树莓派关机，输出电流为低水平
            //     if (OutputCurrentRead() < RpiShutdownCurrent)
            //     {
            //         break;
            //     }

            //     // 等待树莓派关机，RPI_STATE为低水平
            //     // if (RPI_STATE == 0)
            //     // {
            //     //     break;
            //     // }

            //     delayMs(5);
            // }
            // PowerOutClose(); // 断开5V输出
            // PowerInClose();  // 断开电池供电
            break;
        case LongPressed5S: // 长按5s, 强制关闭输出，同时断开PWR
            rgbWrite(ForceShutdownColor[0], ForceShutdownColor[1], ForceShutdownColor[2]);
            PowerOutClose();
            PowerInClose();
            delayMs(200);
            // while (1) // 阻塞以不再进行工作
            // {
            //     if (BTN == 1) // 等待松开按键, 不接入usb，会断电，接入usb会跳出进行运行
            //     {
            //         delayMs(5);
            //         if (BTN == 1)
            //         {
            //             // PowerInHold(); // 恢复电池供电
            //             break;
            //         }
            //     }
            //     delayMs(2);
            // }
            break;
        default:
            // rgbWrite(5, 5, 5);
            // delayMs(200);
            break;
        }

        // 检测间隔计数清零
        scanTimeCount = 0;
    }
}

// rgb 状态灯  与按键共用定时器0 来计数
// =================================================================
extern bool idata outputState;
extern u8 idata isUsbPowered;
extern u8 idata isBatteryPowered;
extern u8 idata isLowPower;
extern u8 idata isCharging;
/* 优先级：充电 > 低电量 > 电池供电 > usb供电
    充电：蓝色呼吸
    低电量：红色闪烁
    电池供电：黄色
    usb供电：绿色
*/
u8 xdata rgbR = 0;
u8 xdata rgbG = 0;
u8 xdata rgbB = 0;
float xdata rgbBrightness = 0; // 0~1
u8 xdata rgbBlinkFlag = 0;     // 0, 灭； 1，亮
/**  lastRgbState
 * 0, 空
 * 1 isCharging
 * 2 isLowPower
 * 3 isBatteryPowered
 * 4 isUsbPowered
 * 5 outputClose
 * 6 others
 */
u8 xdata lastRgbState = 0;

#define rgbBreathInterVal 50   // rgb 呼吸间隔
#define rgbBlinkInterval 500   // rgb 闪烁间隔 ms
#define rgbStaticInterval 3000 // rgb 静态更新间隔
u8 xdata chargingColor[] = {120, 10, 0};
u8 xdata lowPowerColor[] = {60, 0, 0};
u8 xdata batteryPoweredColor[] = {120, 100, 0};
u8 xdata usbPoweredColor[] = {0, 120, 0};

void RgbHandler()
{
    // isCharging = 0;
    // isLowPower = 0;
    // isBatteryPowered = 0;
    // isUsbPowered = 0;

    if ((KeyState == LongPressed2S) || (KeyState == LongPressed2SAndReleased))
    {
        rgbWrite(SafeShutdownColor[0], SafeShutdownColor[1], SafeShutdownColor[2]);
        return;
    }
    else if (KeyState == LongPressed5S)
    {
        rgbWrite(ForceShutdownColor[0], ForceShutdownColor[1], ForceShutdownColor[2]);
        return;
    }

    // ----------------------------------------------------------------
    if (isCharging)
    {
        if (rgbTimeCount > rgbBreathInterVal || lastRgbState != 1)
        {
            lastRgbState = 1;
            if (rgbBrightness == 1)
            {
                rgbBrightness = 0;
            }
            else
            {
                rgbBrightness += 0.05;
            }
            rgbR = (u8)(rgbBrightness * (float)chargingColor[0]);
            rgbG = (u8)(rgbBrightness * (float)chargingColor[1]);
            rgbB = (u8)(rgbBrightness * (float)chargingColor[2]);
            rgbWrite(rgbR, rgbG, rgbB);
            rgbTimeCount = 0;
        }
    }
    else if (isLowPower)
    {
        if (rgbTimeCount > rgbBlinkInterval || lastRgbState != 2)
        {
            lastRgbState = 2;
            rgbBlinkFlag = !rgbBlinkFlag;
            if (rgbBlinkFlag)
            {
                rgbR = lowPowerColor[0];
                rgbG = lowPowerColor[1];
                rgbB = lowPowerColor[2];
                rgbWrite(rgbR, rgbG, rgbB);
            }
            else
            {
                rgbClose();
            }
            rgbTimeCount = 0;
        }
    }
    else if (isBatteryPowered && outputState)
    {
        if (rgbTimeCount > rgbStaticInterval || lastRgbState != 3)
        {
            lastRgbState = 3;
            rgbWrite(batteryPoweredColor[0], batteryPoweredColor[1], batteryPoweredColor[2]);
            rgbTimeCount = 0;
        }
    }
    else if (isUsbPowered && outputState)
    {
        if (rgbTimeCount > rgbStaticInterval || lastRgbState != 4)
        {
            lastRgbState = 4;
            rgbWrite(usbPoweredColor[0], usbPoweredColor[1], usbPoweredColor[2]);
            rgbTimeCount = 0;
        }
    }
    else if (!outputState)
    {
        if (rgbTimeCount > rgbStaticInterval || lastRgbState != 5)
        {
            lastRgbState = 5;
            rgbClose();
            rgbTimeCount = 0;
        }
    }
    else
    {
        if (rgbTimeCount > rgbStaticInterval || lastRgbState != 6)
        {
            lastRgbState = 6;
            rgbWrite(120, 120, 120);
            rgbTimeCount = 0;
        }
    }
}

// 电源管理/电源数据更新  定时器1
// =================================================================
// 定时器周期 = 12*（65535 - 重装载值）/ 系统工作频率
// 16位定时器 65535，系统工作频率 12MHz, 默认12T模式（每12个时钟计数+1）
// 周期 = 12*（6,5535 - 60,535）/12,000,000 = 0.02 s , 20ms
//
// #define Time1Preiod 20 // 定时器1 周期 20ms
// #define TIME1_CR 45535 // 重装载值

// 定时器周期 = 12*（65535 - 重装载值）/ 系统工作频率
// 16位定时器 65535，系统工作频率 12MHz, 默认12T模式（每12个时钟计数+1）
// 周期 = 12*（6,5535 - 60,535）/12,000,000 = 0.02 s , 20ms
//
#define Time1Preiod 10 // 定时器1 周期 20ms
#define TIME1_CR 45535 // 重装载值

extern u16 idata batteryVoltage;
extern int16 idata batteryCurrent;
extern u16 idata usbVoltage;
extern u16 idata outputCurrrent;
extern u16 idata vccVoltage;
extern u8 idata batteryPercentage;

extern int idata dac_vol;
extern int idata pidOutput;

/**
 定时器1初始化 (模式0， 16位自动重装载， 用做定时)
 */
void TM1_INIT()
{
    TMOD &= 0x0F;              // 定时器1，模式0，用作定时器（对内部系统时钟进行计数）；16位自动重装载
    TH1 = (u8)(TIME1_CR >> 8); // 配置重装载值，设置周期
    TL1 = (u8)(TIME1_CR);
    TR1 = 1; // 启动定时器
    ET1 = 1; // 使能定时器中断
    EA = 1;
}

/** 定时器0 中断服务
 */
void TM1_Isr() interrupt 3
{
    // 每 20ms 执行一次
    // ---- 读取数据 ----
    batteryVoltage = BatteryVoltageRead(); // mV
    dataBuffer[0] = batteryVoltage >> 8 & 0xFF;
    dataBuffer[1] = batteryVoltage & 0xFF;

    batteryCurrent = BatteryCurrentRead(); // mA
    dataBuffer[2] = ((u16)batteryCurrent) >> 8 & 0xFF;
    dataBuffer[3] = ((u16)batteryCurrent) & 0xFF;

    usbVoltage = UsbVoltageRead(); // mV
    dataBuffer[4] = usbVoltage >> 8 & 0xFF;
    dataBuffer[5] = usbVoltage & 0xFF;

    outputCurrrent = OutputCurrentRead(); // mA
    dataBuffer[6] = outputCurrrent >> 8 & 0xFF;
    dataBuffer[7] = outputCurrrent & 0xFF;

    vccVoltage = VccVoltageRead(); // mV
    dataBuffer[8] = vccVoltage >> 8 & 0xFF;
    dataBuffer[9] = vccVoltage & 0xFF;

    // ---- 电池电量 ----
    if (batteryVoltage > 8400)
    {
        batteryPercentage = 100;
    }
    else if (batteryVoltage < 6200)
    {
        batteryPercentage = 0;
    }
    else
    {
        batteryPercentage = (u8)(100L * (batteryVoltage - 6200) / 2200);
    }
    dataBuffer[14] = batteryPercentage & 0xFF;

    // // ---- 判断供电和电池状态 ----
    // isCharging
    if (batteryCurrent > 15)
        isCharging = 1;
    else
        isCharging = 0;

    dataBuffer[12] = isCharging & 0xFF;
    // isLowPower
    if (batteryPercentage < 20)
        isLowPower = 1;
    else
        isLowPower = 0;

    // isUsbPowered or isBatteryPowered
    if (batteryCurrent < -10)
    {
        // isUsbPowered = 0;
        isBatteryPowered = 1;
    }
    else
    {
        // isUsbPowered = 1;
        isBatteryPowered = 0;
    }

    if (usbVoltage < 3000)
    {
        isUsbPowered = 0;
        // isBatteryPowered = 1;
    }
    else
    {
        isUsbPowered = 1;
    }

    dataBuffer[10] = isBatteryPowered & 0xFF;
    dataBuffer[11] = isUsbPowered & 0xFF;

    // ---- 充电管理 ---
    ChargeManager(usbVoltage);
    // dataBuffer[16] = dac_vol >> 8 & 0xFF;
    // dataBuffer[17] = dac_vol & 0xFF;
    // dataBuffer[18] = ((u16)pidOutput) >> 8 & 0xFF;
    // dataBuffer[19] = ((u16)pidOutput) & 0xFF;
    // if (outputCurrrent == 0)
    // {
    // setDac(0);
    // }
    // else
    // {
    //     setDac(1000);
    // }
}

void FanControl()
{

    if (lastFanSpeed != fanSpeed)
    {
        lastFanSpeed = fanSpeed;
        fanSetSpeed(fanSpeed);
        // dataBuffer[13] = fanSpeed;
    }
    // if (fanSpeed != controlBuffer[0])
    // {
    //     // fanSpeed = controlBuffer[0];
    //     fanSpeed = *controlBuffer;
    //     fanSetSpeed(fanSpeed);
    //     dataBuffer[12] = fanSpeed;
    // }
}

void RpiShutdownHandler()
{
    // if (RPI_STATE == 0)
    // {
    //     PowerOutClose();
    //     PowerInClose();
    // }
}

//   Init
// =================================================================
void init()
{
    PowerIoInit();
    PowerInHold(); // 将PWR 引脚高电平，维持电池给单片机供电
    PowerManagerAtStart();
    rgbInit();             // 初始化rgb灯
    rgbWrite(120, 100, 0); // 工作指示
    delayMs(250);

    btnInit();
    fanInit();
    AdcInit();
    DacInit();
    I2C_Init();
    TM0_INIT();
    TM1_INIT();
}

// main
// =================================================================
/*
1. 上电，PWR 拉高， 维持单片机供电
2. 判断eeprom 的 掉电记忆，选择是否给树莓派供电
3. 循环
    a. 按键事件 （中断/查询）
    b. i2c 通讯 （中断）
    c. 数据读取
    d. 风扇控制
    e. 充电管理
    f. 树莓派触发关机 （中断/查询）

灯的颜色：

红色长亮 -> 待机
绿色长亮 -> 输出端供电（给树莓派供电）
红色闪烁 -> 电池低电提示
蓝灯呼吸 -> 正在充电
-------------------------------------------------------------- */
#if 1
void main()
{
    init();

    while (1)
    {

        KeyHandler();
        RgbHandler();
        FanControl();
        RpiShutdownHandler();
        delayMs(20);
    }
}
#endif
