#include "i2c.h"

void I2CInit() {
    P1M0 &= ~0x30; P1M1 &= ~0x30;  // P1.4, P1.5 准双向口

    P_SW2 = 0x80;

    I2CCFG = 0x81;                              //使能I2C从机模式
    I2CSLADR = I2C_SLAVE_ADDR;                  //设置从机设备地址寄存器I2CSLADR=0101_1010B
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
    I2CTXD = sendbuffer[0];
    I2CRXD = buffer[addr];

}

// void I2C_Isr() interrupt 24
// {
//     _push_(P_SW2);
//     P_SW2 |= 0x80;

//     if (I2CSLST & 0x40)
//     {
//         I2CSLST &= ~0x40;                       //处理START事件
//     }
//     else if (I2CSLST & 0x20)
//     {
//         I2CSLST &= ~0x20;                       //处理RECV事件
//         if (isda)
//         {
//             isda = 0;                           //处理RECV事件（RECV DEVICE ADDR）
//         }
//         else if (isma)
//         {
//             isma = 0;                           //处理RECV事件（RECV MEMORY ADDR）
//             addr = I2CRXD;
//             I2CTXD = buffer[addr];
//         }
//         else
//         {
//             buffer[addr++] = I2CRXD;            //处理RECV事件（RECV DATA）
//         }
//     }
//     else if (I2CSLST & 0x10)
//     {
//         I2CSLST &= ~0x10;                       //处理SEND事件
//         if (I2CSLST & 0x02)
//         {
//             I2CTXD = 0xff;                      //接收到NAK则停止读取数据
//         }
//         else
//         {
//             I2CTXD = buffer[++addr];            //接收到ACK则继续读取数据
//         }
//     }
//     else if (I2CSLST & 0x08)
//     {
//         I2CSLST &= ~0x08;                       //处理STOP事件
//         isda = 1;
//         isma = 1;
//     }

//     _pop_(P_SW2);
// }


