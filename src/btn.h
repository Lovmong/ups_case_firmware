#ifndef __BTN_H
#define __BTN_H

#include "stc8h.h"
#include "types.h"
#include "delay.h"

#define BTN P37

void btnInit();
u8 btnIsPressed(void);

#endif
