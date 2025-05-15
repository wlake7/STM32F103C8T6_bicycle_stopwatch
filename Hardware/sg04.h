#ifndef __SG04_H
#define __SG04_H
#include "stm32f10x.h"    


#define HCSR04_TIMEOUT      30000   // 30ms超时
#define HCSR04_MAX_DISTANCE 450     // 最大距离450cm
#define HCSR04_MIN_DISTANCE 2       // 最小距离2cm
#define SAMPLE_COUNT        4       // 采样次数
#define FILTER_THRESHOLD    50      // 异常值过滤阈值(cm)
#define MAX_DEVIATION       20      // 最大允许偏差(cm)

#define HCSR04_Trig GPIO_Pin_1
#define HCSR04_Echo GPIO_Pin_6
#define LED_T GPIO_Pin_15
#define LED_Reminder 50  //LED提醒距离

extern uint8_t HCSR04_CompleteFlag;
extern float HCSR04_Distance;

void HCSR04_led_Init(void);
void HCSR04_StartMeasure(void);
float HCSR04_GetDistance(void);

#endif
