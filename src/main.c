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

bit isda; // 设备地址标志
bit isma; // 存储地址标志
u8 addr;  // i2c buffer address

u8 data_buffer[] = {
    0, // 电池电压高位 mV
    0, // 电池电压低位 mV
    0, // 输出电流高位 mA
    0, // 输出电流低位 mA
    0, // Power_Source, 0 usb 供电，1 电池供电
    0, // CHG, 0 电池未充电， 1 正在充电
    0, // 风扇速度 0 ~ 100
    0, // PWM CCR1H
    0, // PWM CCR1H
};

u8 control_buffer[] = {
    0, // 风扇速度 0 ~ 100
};

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
    addr = 0;
    I2CTXD = data_buffer[addr];
}

#if 1 // ------------ I2C_Isr (中断号 24)------------------------
void I2C_Isr() interrupt 24
{
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
        { // 处理RECV事件（RECV DEVICE ADDR）
            isda = 0;
        }
        else if (isma)
        { // 处理RECV事件（RECV MEMORY ADDR）
            isma = 0;
            addr = I2CRXD;
            I2CTXD = data_buffer[addr];
        }
        else
        { // 处理RECV事件（RECV DATA）
            if (addr >= 0x80)
            {
                control_buffer[addr - 0x80] = I2CRXD;
            }
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
            I2CTXD = data_buffer[++addr];
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
}
#endif

//  ------------------ Init ------------------
void init()
{
    PowerIoInit();
    PowerInHold(); // 将PWR 引脚高电平，维持电池给单片机供电
    PowerManagerAtStart();

    rgbInit(); // 初始化rgb灯
    // rgbWrite(0, 0, 0); // 工作指示

    // btnInit();
    // fanInit();
    AdcInit();
    I2C_Init();
}

/* --------------------- main ---------------------
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
    u16 count = 0;
    u8 fan_speed = 0;
    u8 stat = 0;
    u16 battery_voltage = 0;
    u16 output_currrent = 0;
    u16 _ccr = 0;

    init();

    // rgbTest();
    // btnTest();
    // fanTest();
    // fanSetSpeed(50);

    // P2M0 &= ~0x10; P2M1 &= ~0x10;    // P2.4 准双向口
    // P2M0 |= 0x10; P2M1 &= ~0x10;
    // FAN = 0;

    while (1)
    {
        // P1M0 &= ~0x80; P1M1 &= ~0x80;

        // AdcInit();

        // // ADCCFG |= 0x20;                             //RESFMT 结果右对齐
        // ADCCFG = 0x00;                             //RESFMT 结果左对齐
        // ADC_CONTR &=  0xF0;                     //复位通道
        // ADC_CONTR |= 0x07;                   //选择adc通道

        // // adc 读取
        // ADC_CONTR |= 0x40;                      //启动AD转换
        // _nop_();
        // _nop_();
        // while (!(ADC_CONTR & 0x20));            //等待ADC完成
        // ADC_CONTR &= ~0x20;                     //清完成标志
        // // adc_val = ADC_RES<<8 | ADC_RESL;        //读取ADC结果
        // // battery_voltage = (ADC_RES << 2)| (ADC_RESL >> 6);

        // data_buffer[0] = ADC_RES;
        // data_buffer[1] = ADC_RESL;

        battery_voltage = BatteryVoltageRead(); // mV
        data_buffer[0] = battery_voltage >> 8 & 0xFF;
        data_buffer[1] = battery_voltage & 0xFF;

        output_currrent = BatteryCurrentRead(); // mA
        data_buffer[2] = output_currrent >> 8 & 0xFF;
        data_buffer[3] = output_currrent & 0xFF;
        delayMs(20);
    }

    // while(1) {
    // rgbWrite(1, 1, 0);
    // if (btnIsPressed() == 1) {
    //     count++;
    //     if( count > 8) count = 0;
    //     rgbWrite(rgb_styles[count][0], rgb_styles[count][1], rgb_styles[count][2]);
    //     delayMs(200);

    //     // fan_speed = !fan_speed;
    //     // fanSetSpeed(fan_speed);
    // }

    // delayMs(20);

    //     // 电压
    //     battery_voltage = BatteryVoltageRead();  // mV
    //     data_buffer[0] = battery_voltage >> 8 & 0xFF;
    //     data_buffer[1] = battery_voltage & 0xFF;

    //     // 输出电流
    //     output_currrent = OutputCurrentRead(); // mA
    //     data_buffer[2] = output_currrent >> 8 & 0xFF;
    //     data_buffer[3] = output_currrent & 0xFF;

    //     // Power_Source, 是否电池供电
    //     data_buffer[4] = Power_Source;

    //     // CHG, 是否在给电池充电
    //     data_buffer[5] = CHG;

    //     // 风扇速度
    //     // data_buffer[6] = fan_speed;
    //     data_buffer[6] = control_buffer[0];
    //     // fan_speed = control_buffer[0];
    //     // fan_speed = data_buffer[6];

    //     // fanSetSpeed(50);

    //     if (fan_speed != control_buffer[0]) {
    //         fan_speed = control_buffer[0];

    //         _ccr = ((u32)_PWMA_PERIOD*fan_speed/100);
    //         data_buffer[7] = (u8)(_ccr >> 8);
    //         data_buffer[8] = (u8)(_ccr);

    //         fanSetSpeed(fan_speed);
    //     }

    //     delayMs(20);
    // }
}
#endif
