#include "btn.h"

void btnInit() {
    P3M0 &= ~0x80; P3M1 |= 0x80; // 设置 P3.7 为高阻输入
    // P3M0 &= ~0x80; P3M1 &= ~0x80;  // 设置 P3.7 为准双向口
}


u8 btnIsPressed(void) {
    if (BTN == 0) {
        delayMs(5); // 消抖
        if (BTN == 0) return 1;
        else return 0;
    } else {
        return 0;
    }
}

