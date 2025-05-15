#ifndef __STIME_H
#define __STIME_H

#include "stm32f10x.h"

// 时间数据结构
typedef struct {
    uint32_t totalTimeMs;          // 总骑行时间(ms)
    uint32_t lastUpdateTimeMs;     // 上次更新时间(ms)
    uint8_t isRunning;             // 是否正在计时
    
    // 用于显示的格式化时间
    uint8_t hours;// 小时部分
    uint8_t minutes;// 分钟部分
    uint8_t seconds;// 秒部分
    uint16_t milliseconds;  // 毫秒部分
} RidingTime_t;

// 初始化函数
void STime_Init(void);

// 时间操作函数
void STime_Start(void);
void STime_Stop(void);
void STime_Reset(void);
void STime_Update(void);

// 获取时间信息函数
uint32_t STime_GetTotalRidingTimeMs(void);

void STime_GetFormattedTime(uint8_t* hours, uint8_t* minutes, uint8_t* seconds, uint16_t* milliseconds);
uint8_t STime_IsRunning(void);

// 添加全局锁存状态函数
void STime_SetDataLocked(uint8_t locked);
uint8_t STime_IsDataLocked(void);

extern RidingTime_t ridingTime;
extern uint8_t g_dataLocked; // 添加全局锁存状态变量

#endif /* __STIME_H */
