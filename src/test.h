#include "stc8h.h"
#include "intrins.h"

#include "types.h"
#include "delay.h"

#include "btn.h"
#include "rgb.h"
#include "fan.h"

#include "power.h"


void rgbTest() {
    u8 count = 0;
    while (1) {
        for (count = 0; count < 255; count++) {
            rgbWrite(count, 0, 0);
            delayMs(50);
        }
        for (count = 0; count < 255; count++) {
            rgbWrite(0, count, 0);
            delayMs(50);
        }
        for (count = 0; count < 255; count++) {
            rgbWrite(0, 0, count);
            delayMs(50);
        }
    }
}

void btnTest() {
    u8 i = 0;
    while (1) {
        if (btnIsPressed() == 1) {
            i++;
            if( i > 7) i = 0;
            rgbWrite(i*32, i*32, i*32);
            delayMs(200);
        }
    }
    
}

void fanTest() {
    u8 i = 0;
    while (1) {
        if (btnIsPressed() == 1) {
            i+=10;
            if( i > 100) i = 0;
            fanSetSpeed(i);
            rgbWrite(i*2, i*2, i*2);
            delayMs(200);
        }
    }
    
}