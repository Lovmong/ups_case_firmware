/*============================================================================


STC8H1K08:





==============================================================================*/


#include "stc8h.h"
#include "intrins.h"

#include "types.h"
#include "delay.h"

#include "btn.h"
#include "rgb.h"
#include "fan.h"
#include "power.h"
// #include "i2c.h"



/* --- i2c data registers --- */
#define I2C_SLAVE_ADDRESS 0x5A

bit isda;                                       //设备地址标志
bit isma;                                       //存储地址标志

u8 addr;
u8 i2c_buffer[] = {
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

u8 i2c_write_buffer[] = {
    0, // 风扇速度 0 ~ 100
};


const u8 rgb_styles[8][3] = {
    {0, 1, 1},
    {1, 0, 1},
    {1, 1, 0},
    {0, 0, 0},
    {0, 0, 1},
    {0, 1, 0},
    {1, 0, 0},
    {1, 1, 1},
};


void I2CInit(u8 addr) {
    P1M0 &= ~0x30; P1M1 &= ~0x30;  // P1.4, P1.5 准双向口

    P_SW2 = 0x80;

    I2CCFG = 0x81;                              //使能I2C从机模式
    I2CSLADR = (addr << 1) & 0xff;              //设置从机设备地址寄存器I2CSLADR=0101_1010B
                                                //设置从机设备地址寄存器I2CSLADR=0101_1010B
                                                //即I2CSLADR[7:1]=010_1101B,MA=0B。
                                                //由于MA为0,主机发送的的设备地址必须与
                                                //I2CSLADR[7:1]相同才能访问此I2C从机设备。
                                                //主机若需要写数据则要发送5AH(0101_1010B)
                                                //主机若需要读数据则要发送5BH(0101_1011B)
    I2CSLST = 0x00;
    I2CSLCR = 0x78;                             //使能从机模式中断
    EA = 1;

    isda = 1;                                   //用户变量初始化
    isma = 1;
    addr = 0;
    I2CTXD = i2c_buffer[addr];
}


/* ------------------------------------------------------------
ADC_CONTR: ADC_CONTR ADC 控制寄存器
        7           6            5          4          3 2 1 0
        ADC_POWER  ADC_START   ADC_FLAG    ADC_EPWMT   ADC_CHS[3:0]

ADC_RES： ADC 转换结果高位寄存器
ADC_RESL： ADC 转换结果低位寄存器
ADCCFG：ADC 配置寄存器
        5
       RESFMT
ADCTIM: ADC 时序控制寄存器

------------------------------------------------------------ */
u16 BatteryVoltageRead() {
    u16 adc_val = 0;

    // 使能 adc
    P3M0 &= ~0x02; P3M1 &= ~0x02; // P3.1 准双向口
    P_SW2 |= 0x80;
    ADCTIM = 0x3f;                              //设置ADC内部时序
    // P_SW2 &= 0x7f;
    ADCCFG = 0x0f;                              //设置ADC时钟为系统时钟/2/16
    ADCCFG |= 0x20;                             // RESFMT 结果右对齐
    ADC_CONTR = 0x80;                           //ADC_POWER, 使能ADC模块 
    ADC_CONTR |= 0x09;                          // ADC_9, 选择通道9

    // adc 读取
    ADC_CONTR |= 0x40;                      //启动AD转换
    _nop_();
    _nop_();
    while (!(ADC_CONTR & 0x20));            //查询ADC完成标志
    ADC_CONTR &= ~0x20;                     //清完成标志
    adc_val = ADC_RES<<8 | ADC_RESL;        //读取ADC结果

    // 计算电压毫伏
    return (u16)(3300 * (adc_val/1024.0)); // 10位adc 分辨率为 1024
}


u16 OutputCurrentRead() {
    u16 adc_val = 0;

    // 使能 adc
    P3M0 &= ~0x01; P3M1 &= ~0x01; // P3.0 准双向口
    P_SW2 |= 0x80;
    ADCTIM = 0x3f;                              //设置ADC内部时序
    // P_SW2 &= 0x7f;
    ADCCFG = 0x0f;                              //设置ADC时钟为系统时钟/2/16
    ADCCFG |= 0x20;                             //RESFMT 结果右对齐
    ADC_CONTR = 0x80;                           //ADC_POWER, 使能ADC模块 
    ADC_CONTR |= 0x08;                          //ADC_8, 选择通道8

    // adc 读取
    ADC_CONTR |= 0x40;                      //启动AD转换
    _nop_();
    _nop_();
    while (!(ADC_CONTR & 0x20));            //查询ADC完成标志
    ADC_CONTR &= ~0x20;                     //清完成标志
    adc_val = ADC_RES<<8 | ADC_RESL;        //读取ADC结果
    
    // 计算电流mA: voltage/100/0.05
    return (u16)(3300 * (adc_val/1024.0)/100/0.05); // 10位adc 分辨率为 1024
}


void init() {
    PowerIoInit();
    PowerInHold(); // 将PWR 引脚高电平，维持电池给单片机供电
    rgbInit(); // 初始化rgb灯
    // rgbWrite(0, 1, 1); // 工作指示

    btnInit();
    fanInit();

    PowerManagerAtStart();
    
    I2CInit(I2C_SLAVE_ADDRESS);
}

/* --------------------------------------------------------------

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
void main() {
    u8 count = 0;
    u8 fan_speed = 0;
    u8 stat = 0;
    u16 battery_voltage = 0;
    u16 output_currrent = 0;
    u16 _ccr = 0;
    
    init();
    rgbWrite(0, 0, 1); // 绿+红
    EN = 1;
    // while (1);
    // rgbWrite(0, 1, 1);
    

    // while (1) {
    //     rgbWrite(0, 1, 1);
    //     delayMs(1000);
    //     rgbWrite(1, 0, 1);
    //     delayMs(1000);
    //     rgbWrite(1, 1, 0);
    //     delayMs(1000);
    // }
    

    while(1) {
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

        // 电压
        battery_voltage = BatteryVoltageRead();  // mV
        i2c_buffer[0] = battery_voltage >> 8 & 0xFF;
        i2c_buffer[1] = battery_voltage & 0xFF;

        // 输出电流
        output_currrent = OutputCurrentRead(); // mA
        i2c_buffer[2] = output_currrent >> 8 & 0xFF;
        i2c_buffer[3] = output_currrent & 0xFF;

        // Power_Source, 是否电池供电
        i2c_buffer[4] = Power_Source;

        // CHG, 是否在给电池充电
        i2c_buffer[5] = CHG;

        // 风扇速度
        // i2c_buffer[6] = fan_speed;
        i2c_buffer[6] = i2c_write_buffer[0];
        // fan_speed = i2c_write_buffer[0];
        // fan_speed = i2c_buffer[6];

        // fanSetSpeed(50);
        
        if (fan_speed != i2c_write_buffer[0]) {
            fan_speed = i2c_write_buffer[0];

            _ccr = ((u32)PWM_PERIOD*fan_speed/100);
            i2c_buffer[7] = (u8)(_ccr >> 8);
            i2c_buffer[8] = (u8)(_ccr);

            fanSetSpeed(fan_speed);
        }

        delayMs(20);
    }

}
#endif

#if 0
void main()
{
    u8 i = 0;

    init();
    EN = 1;
    // FAN = 1;
    // fanSetSpeed(20);
    while (1) {
        for(i=0; i<100; i++) {
            fanSetSpeed((u8)i);
            delayMs(200);
        }
    }
}
#endif

#if 1
void I2C_Isr() interrupt 24
{
    _push_(P_SW2);
    P_SW2 |= 0x80;

    if (I2CSLST & 0x40)
    {
        I2CSLST &= ~0x40;                       //处理START事件
    }
    else if (I2CSLST & 0x20)
    {
        I2CSLST &= ~0x20;                       //处理RECV事件
        if (isda)
        {
            isda = 0;                           //处理RECV事件（RECV DEVICE ADDR）
        }
        else if (isma)
        {
            isma = 0;                           //处理RECV事件（RECV MEMORY ADDR）
            addr = I2CRXD;
            I2CTXD = i2c_buffer[addr];
        }
        else
        {
            //处理RECV事件（RECV DATA）
   
            if (addr >= 0x80) {
                i2c_write_buffer[addr-0x80] = I2CRXD;
            }
            
        }
    }
    else if (I2CSLST & 0x10)
    {
        I2CSLST &= ~0x10;                       //处理SEND事件
        if (I2CSLST & 0x02)
        {
            I2CTXD = 0xff;                      //接收到NAK则停止读取数据
        }
        else
        {
            I2CTXD = i2c_buffer[++addr];            //接收到ACK则继续读取数据

        }
    }
    else if (I2CSLST & 0x08)
    {
        I2CSLST &= ~0x08;                       //处理STOP事件
        isda = 1;
        isma = 1;
    }

    _pop_(P_SW2);
}
#endif

