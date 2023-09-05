#ifndef __RGB_H
#define __RGB_H

#include "stc8h.h"
#include "types.h"

#define RGB_R P16
#define RGB_G P13
#define RGB_B P12

void rgbInit();
void rgbWrite(u8 r, u8 g, u8 b);
void rgbClose();

#endif
