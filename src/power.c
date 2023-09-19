#include "power.h"

bool idata outputState = 0;
u8 idata isUsbPowered = 0;
u8 idata isBatteryPowered = 0;
u8 idata isLowPower = 0;
u8 idata isCharging = 0;

u16 idata batteryVoltage = 0;
int16 idata batteryCurrent = 0;
u16 idata usbVoltage = 0;
u16 idata outputCurrrent = 0;
u16 idata vccVoltage = 0;

u8 idata batteryPercentage = 0;
// =========================== 初始化 ==================================
void PowerIoInit()
{
    // output 推挽输出
    P0M0 |= 0x01;
    P0M1 &= ~0x01; // EN, P0.0 推挽输出
    P3M0 |= 0x04;
    P3M1 &= ~0x04; // PWR, P3.2 推挽输出
    // input 高阻输入
    P0M0 &= ~0x02;
    P0M1 |= 0x02; // CHG, P0.1 高阻输入
    P1M0 &= ~0x08;
    P1M1 |= 0x08; // Power_Source, P1.3 高阻输入
    P2M0 &= ~0x08;
    P2M1 |= 0x08; // RPI_STATE, P2.3 高阻输入
    P2M0 &= ~0x20;
    P2M1 |= 0x20; // ALWAYS_ON, P2.5 高阻输入
    // dac 推挽输出
    P1M0 |= 0x01;
    P1M1 &= ~0x01; // DAC, P1.0 推挽输出
    // adc
    P1M0 &= ~0x40;
    P1M1 |= 0x40; // VBUS_3V3, P1.6 高阻输入
    P1M0 &= ~0x80;
    P1M1 |= 0x80; // BT_LV_3V3, P1.7 高阻输入
    P0M0 &= ~0x04;
    P0M1 |= 0x04; // BATTERY_CURRENT, P0.2 高阻输入
    P0M0 &= ~0x08;
    P0M1 |= 0x08; // OUTPUT_CURRENT, P0.3 高阻输入
}

/** 维持电池给单片机供电 */
void PowerInHold()
{
    PWR = 1; // 维持高电平， 维持给单片机供电
    // delayMs(100); // 等待电源稳定 > 60 ms
}

void PowerInClose()
{
    PWR = 0; // 单片机断电
}

// ======================== 5V 输出相关 ==================================

/** 打开5v输出,记忆状态 */
void PowerOutOpen()
{
    EN = 1;
    outputState = 1;
    IapErase(POWER_MEMORY_ADDR);         // 擦除EEPROM扇区
    IapProgram(POWER_MEMORY_ADDR, 0x02); // 记忆输出打开
}

/** 打开5v输出,记忆状态 */
void PowerOutClose()
{
    EN = 0;
    outputState = 0;
    IapErase(POWER_MEMORY_ADDR);         // 擦除EEPROM扇区
    IapProgram(POWER_MEMORY_ADDR, 0x01); // 记忆输出关闭
}

/** 上电时恢复电源记忆 */
void PowerManagerAtStart()
{
    /*当 ALWAYS_ON 为有效时（低电平）时，power 输出(),
     *否则：
     *    读取eeprom值
     *    如果值为 0x01，power不输出
     *    如果值为 0x02，power不输出
     *    如果值错误，默认power不输出
     */
    char stat = 0;

    if (ALWAYS_ON == 0)
    {
        delayUs(500); // 消抖
        if (ALWAYS_ON == 0)
        {
            PowerOutOpen(); // power输出
            return;
        }
    }
    // else
    // {
    //     PowerOutClose();
    //     return;
    // }

    stat = IapRead(POWER_MEMORY_ADDR);
    // EEPROM 的写操作只能将字节中的 1 写为 0，当需要将字节中的 0 写为 1，则必须执行扇区 擦除操作。
    switch (stat)
    {
    case 0x01: // 关闭输出
        PowerOutClose();
        break;
    case 0x02: // 开启输出
        PowerOutOpen();
        break;
    default: // 默认power不输出
        PowerOutClose();
        break;
    }
}

// ========================  adc读值相关 ==================================
/* ------------------------------------------------------------
ADC_CONTR: ADC_CONTR ADC 控制寄存器
        7           6            5          4          3 2 1 0
        ADC_POWER  ADC_START   ADC_FLAG    ADC_EPWMT   ADC_CHS[3:0]
    ADC_CHS[3:0]： 通道选择

ADC_RES： ADC 转换结果高位寄存器
ADC_RESL： ADC 转换结果低位寄存器
ADCCFG：ADC 配置寄存器
        5
       RESFMT
ADCTIM: ADC 时序控制寄存器

------------------------------------------------------------ */
u16 refVoltage = 0;
int *BGV;
u16 VCC = 0;

/** 初始化adc */
void AdcInit()
{
    P_SW2 |= 0x80;    // 使能扩展寄存器(XFR)访问
    ADCTIM = 0x3f;    // 设置ADC内部时序
    ADCCFG = 0x0f;    // 设置ADC时钟为系统时钟/2/16
    ADCCFG |= 0x20;   // RESFMT 结果右对齐
    ADC_CONTR = 0x80; // ADC_POWER, 使能ADC模块

    BGV = (int idata *)0xef;
    VccVoltageRead();
}

/** adc读值 */
u16 AdcRead(u8 channel)
{
    u16 adc_val = 0;

    ADC_CONTR &= 0xF0;    // 复位通道
    ADC_CONTR |= channel; // 选择adc通道

    // adc 读取
    ADC_CONTR |= 0x40; // 启动AD转换
    _nop_();
    _nop_();
    while (!(ADC_CONTR & 0x20))
        ;                              // 等待ADC完成
    ADC_CONTR &= ~0x20;                // 清完成标志
    adc_val = ADC_RES << 8 | ADC_RESL; // 读取ADC结果
    // 计算电压毫伏
    return (u16)(adc_val); // 10位adc 分辨率为 1024
}

u16 AdcReadVoltage(u8 channel)
{
    u16 voltage = 0;

    refVoltage = AdcRead(REF_VOLTAGE_CHANNEL);
    voltage = AdcRead(channel);
    voltage = (((float)*BGV / refVoltage) * voltage);
    return (u16)(voltage);
}

u16 VccVoltageRead()
{
    refVoltage = AdcRead(REF_VOLTAGE_CHANNEL);
    VCC = (u16)((1024L * *BGV / refVoltage));
    return VCC;
}

/** 读取USB输入电压(mV)*/
u16 UsbVoltageRead()
{
    // VBUS_3V3 -> ADC6 -> P1.6
    return (u16)(AdcReadVoltage(USB_VOLTAGE_CHANNEL) * USB_VOLTAGE_GAIN); // 返回读值
}

/** 读取Battery输出电压(mV) */
u16 BatteryVoltageRead()
{
    // BT_LV_3V3 -> ADC7 -> P1.7
    return (u16)(AdcReadVoltage(BATTERY_VOLTAGE_CHANNEL) * BATTERY_VOLTAGE_GAIN); // 返回读值
}

/** 读取Battery输出电流(mA)
 * 正值 充电
 * 负值 输出
 */
int16 BatteryCurrentRead()
{

    // BATTERY_CURRENT -> ADC10 -> P0.2
    return (int16)(-(AdcReadVoltage(BATTERY_CURRENT_CHANNEL) - 1250) / 0.005 / 100); // 返回读值 电流 = （测量电压-1.25V）/100/0.05
    // 差分放大100倍，除以0.005欧姆电阻
    // return (u16)((AdcReadVoltage(BATTERY_CURRENT_CHANNEL))); // 返回读值 电流 = （测量电压-1.25V）/100/0.05
}

/** 读取Output输出电流(mA)*/
u16 OutputCurrentRead()
{
    // OUTPUT_CURRENT -> ADC11 -> P0.3
    return (u16)((AdcReadVoltage(OUTPUT_CURRENT_CHANNEL)) / 0.005 / 100); // 返回读值 电流 = （测量电压-1.25V）/100/0.05
    // 差分放大100倍，除以0.005欧姆电阻

    // return (u16)((AdcReadVoltage(OUTPUT_CURRENT_CHANNEL))); // 返回读值 电流 = （测量电压-1.25V）/100/0.05
}

// ======================== 电池充电相关 ==================================

/** 电池充电管理 */
int idata dac_vol = 1500;
int idata pidOutput = 0;
void ChargeManager(u16 current_vol)
{
    /* 如果,不需要对外部供电, 则:最大电流对电池充电
     * 否则:根据usb输入电压，使用pid调节dac电压，进而调节充电电流
     */
    if (outputState == 0) // usb不需要输出给5V, 最大电流去充电
    {
        setDac(0);
        // DAC = 0;
    }
    else
    {
        // setDac(1000);
        pidOutput = (int)pidCalculate(current_vol);
        dac_vol -= pidOutput;
        if (dac_vol > 1500)
            dac_vol = 1500;
        else if (dac_vol < 0)
            dac_vol = 0;
        setDac((u16)dac_vol);
        // setDac(1500);
    }
}

/** dac初始化 */
void DacInit()
{
    /* DAC -> PWM1P -> P1.0
     * 注意 dac 和 fan 都使用 PWMA, 初始化时 预分频 和 自动重装载值要一致，或者仅初始化一次
     */
    P1M0 |= 0x01;
    P1M1 &= ~0x01; // DAC, P1.0 推挽输出

    P_SW2 |= 0x80; // 使能扩展寄存器(XFR)访问

    PWMA_CCER1 = 0x00; // 写CCMRx前必须先清零CCERx关闭通道
    PWMA_CCMR1 = 0x68; // 设置PWM1输出模式 1, 使能预装载功能
    PWMA_CCER1 = 0x01; // 使能PWM1P通道, 极性为 高电平有效

    PWMA_PSCRH = (u8)((_PWMA_PSCR - 1) >> 8); // 设置PWMA预分频
    PWMA_PSCRL = (u8)(_PWMA_PSCR - 1);
    PWMA_ARRH = (u8)((_PWMA_PERIOD - 1) >> 8); // 设置周期时间， PWMA_ARR自动重装值
    PWMA_ARRL = (u8)(_PWMA_PERIOD - 1);

    PWMA_CCR1H = (u8)(1500 >> 8); // 初始化PWM1占空比时间 1.5v 停止充电
    PWMA_CCR1L = (u8)(1500);

    PWMA_ENO |= 0x01; // 使能PWM1P输出

    PWMA_BKR = 0x80; // 使能PWMA主输出
    PWMA_CR1 = 0x01; // 使能PWMA计数器，开始计时, (00 边沿对齐模式)
}

/** 设置dac输出电压（mV）*/
void setDac(u16 voltage)
{
    u16 ccr = 0;
    ccr = ((u32)_PWMA_PERIOD * voltage / VCC); // 满级电压3.3V

    P_SW2 |= 0x80;               // 使能扩展寄存器(XFR)访问
    PWMA_CCR1H = (u8)(ccr >> 8); // 设置PWM1占空比
    PWMA_CCR1L = (u8)(ccr);
}

/** pid计算
 * 输入：充电口(usb)电压（mV）
 * 返回：充电电流调节电压（mV）
 */
int idata lastError = 0;
long idata sumError = 0;

float pidCalculate(u16 current_vol)
{
    float error, dError, output;

    error = (int)current_vol - targetVoltage; // 偏差

    // sumError += error;                   // 积分
    // if (sumError < -330000)
    //     sumError = -330000; // 积分限幅
    // else if (sumError > 330000)
    //     sumError = 330000;
    dError = error - lastError; // 微分
    lastError = error;

    // output = kp * error + ki * sumError + kd * dError;
    output = kp * error + kd * dError;
    // if (output < 0)
    //     output = 0;
    // else if (output > maxValue)
    //     output = maxValue;

    return output;
}

// ========================  电池电量相关 ==================================
#define CheckPoint0 6800 // 电池 7% 电量测量点电压
#define CheckPoint1 6500 // 电池 3% 电量测量点电压
#define CheckPoint3 6200 // 电池 0% 电量测量点电压
#define MaxVoltage 8400  // mV 100% 时电压

u16 MaxCapacity = 2000;       // mAh 电池最大容量
u16 RemainingCapacity = 2000; // mAh 电池剩余容量
u16 BatteryIR = 50;           // mOhm 电池内阻

/** 计算电池内阻 */
u16 calculateBatteryIR()
{
    /* 通过测量开启负载后的前后压差和电流差，计算内阻
     * IR = (U2 - U1)/(I2 - I1)
     */
    u16 U1, U2, I1, I2, IR;

    // ---- 计算内阻
    // closeRL(); // 关闭负载电阻
    EN = 0;
    U1 = (u16)BatteryVoltageRead(); // mV
    I1 = (u16)BatteryCurrentRead(); // mAh

    // openRL(); // 打开负载电阻
    EN = 1;
    U2 = (u16)BatteryVoltageRead();
    I2 = (u16)BatteryCurrentRead();

    IR = 1000 * (u32)(U2 - U1) / (I2 - I1); // mOhm

    BatteryIR = IR;
    return IR;
}

/** 上电时初始化电池剩余容量 */
void RemainingCapacityInit()
{
    /* 根据电压与电容的对应关系，粗略估算电池电压

    */
    u16 vol, current;

    vol = BatteryVoltageRead();
    current = BatteryCurrentRead();
}

/** 更新电量百分比
 * 返回 百分比 0 ~ 100
 */
u8 UpdateBatteryPercentage()
{
}

/* ----- 低电量时 -----

*/
void PowerManagerInLowBattery()
{
}

/* ----- 电源数据读取 -----

*/
void PowerMonitor()
{
}
