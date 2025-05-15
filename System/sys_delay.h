#ifndef __SYS_DELAY_H
#define __SYS_DELAY_H

#include "stm32f10x.h"

void SysTick_Init(void);                   // SysTick初始化函数
void sys_Delay_us(uint32_t nus);               // 微秒延时函数
void sys_Delay_ms(uint32_t nms);               // 毫秒延时函数
void sys_Delay_s(uint32_t ns);                 // 秒延时函数


#endif /* __SYS_DELAY_H */
