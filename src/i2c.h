#ifndef __I2C_H
#define __I2C_H

#include "stc8h.h"
#include "types.h"

#define I2C_SLAVE_ADDR  0x5A

// #define SDA P14
// #define SCL P15

bit isda;                                       //设备地址标志
bit isma;                                       //存储地址标志
unsigned char addr;
unsigned char pdata buffer[256];

unsigned char sendbuffer[5] = {0x01, 0x02, 0x03, 0x04, 0x5};


void I2CInit();


#endif

