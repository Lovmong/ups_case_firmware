#include "rgb.h"

void rgbInit() {
    P1M0 |= 0x4c; P1M1 &= ~0x4c;  // 设置P1.6, P1.3, P1.2为推挽输出模式
}


void rgbWrite(u8 r, u8 g, u8 b) {
    RGB_R = r;
    RGB_G = g;
    RGB_B = b;
}

void rgbClose() {
    RGB_R = 0;
    RGB_G = 0;
    RGB_B = 0;   
}
