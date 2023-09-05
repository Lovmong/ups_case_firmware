#ifndef __EEPROM_H
#define __EEPROM_H

#include "stc8h.h"

/*------------- EEPROM -------------------------------------------

型号      大小 扇区  IAP方式读/写/擦除          MOVC读取 
                    起始地址 结束地址      起始地址  结束地址
STC8H1K08 4K   8     0000h   0FFFh        2000h   2FFFh

----------------------------------------------------------------*/

void IapIdle();
char IapRead(int addr);
void IapProgram(int addr, char dat);
void IapErase(int addr);

#endif

