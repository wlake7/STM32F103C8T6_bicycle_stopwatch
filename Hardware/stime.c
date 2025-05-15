#include "stime.h"
#include "Hall.h"

// 全局变量
RidingTime_t ridingTime = {0};
volatile uint32_t systemTimeMs = 0; // 系统时间，单位ms
uint8_t g_dataLocked = 0; // 全局锁存状态标志，0表示未锁存，1表示已锁存

// 初始化时间模块
void STime_Init(void)
{
    TIM_TimeBaseInitTypeDef TIM_TimeBaseStructure;
    NVIC_InitTypeDef NVIC_InitStructure;
    
    // 使能TIM1时钟
    RCC_APB2PeriphClockCmd(RCC_APB2Periph_TIM1, ENABLE);
    
    // 配置TIM1定时为1ms
    TIM_TimeBaseStructure.TIM_Period = 1000 - 1;  // 自动重装载值
    TIM_TimeBaseStructure.TIM_Prescaler = 72 - 1; // 分频系数，72MHz / 72 = 1MHz，这两个共同作用决定了每1ms进入一次中断
    TIM_TimeBaseStructure.TIM_ClockDivision = TIM_CKD_DIV1;
    TIM_TimeBaseStructure.TIM_CounterMode = TIM_CounterMode_Up;
    TIM_TimeBaseInit(TIM1, &TIM_TimeBaseStructure);
    
    // 配置TIM1更新中断
    NVIC_InitStructure.NVIC_IRQChannel = TIM1_UP_IRQn;
    NVIC_InitStructure.NVIC_IRQChannelPreemptionPriority = 1;
    NVIC_InitStructure.NVIC_IRQChannelSubPriority = 1;
    NVIC_InitStructure.NVIC_IRQChannelCmd = ENABLE;
    NVIC_Init(&NVIC_InitStructure);
    
    // 使能TIM1更新中断
    TIM_ITConfig(TIM1, TIM_IT_Update, ENABLE);
    
    // 启动TIM1
    TIM_Cmd(TIM1, ENABLE);
    
    // 初始化时间变量
    ridingTime.totalTimeMs = 0;
    ridingTime.lastUpdateTimeMs = 0;
    ridingTime.isRunning = 0;
    ridingTime.hours = 0;
    ridingTime.minutes = 0;
    ridingTime.seconds = 0;
    ridingTime.milliseconds = 0;
}

// 启动计时
void STime_Start(void)
{
    // 只有在未锁存状态下才允许启动
    if (!g_dataLocked) {
        ridingTime.isRunning = 1;
        ridingTime.lastUpdateTimeMs = systemTimeMs;
    }
}

// 停止计时
void STime_Stop(void)
{
    ridingTime.isRunning = 0;
}

// 重置计时器
void STime_Reset(void)
{
    ridingTime.totalTimeMs = 0;
    ridingTime.lastUpdateTimeMs = systemTimeMs;
    ridingTime.hours = 0;
    ridingTime.minutes = 0;
    ridingTime.seconds = 0;
    ridingTime.milliseconds = 0;
}

// 更新计时(在主循环中调用)
void STime_Update(void)
{
    uint32_t currentTime = systemTimeMs;
    
    // 检查骑行状态，如果数据已锁存，则不更新

    if (g_dataLocked) {
        return;
    }

    // 检查骑行状态
    RidingState_t currentRidingState = Hall_GetRidingState();
    
    // 如果是首次检测到骑行活动，自动开始计时
    /*
    if (currentRidingState == RIDING_ACTIVE && !ridingTime.isRunning) {
        ridingTime.isRunning = 1;
        ridingTime.lastUpdateTimeMs = currentTime;
    }
    */
    // 如果计时器正在运行，更新时间（不再判断骑行状态）
    if (ridingTime.isRunning) {
        uint32_t elapsedTime = currentTime - ridingTime.lastUpdateTimeMs; // 计算时间差
        
        ridingTime.totalTimeMs += elapsedTime;
        
        ridingTime.lastUpdateTimeMs = currentTime;
        
        // 格式化时间为时分秒
        ridingTime.milliseconds = ridingTime.totalTimeMs % 1000;
        uint32_t totalSeconds = ridingTime.totalTimeMs / 1000;
        ridingTime.seconds = totalSeconds % 60;
        uint32_t totalMinutes = totalSeconds / 60;
        ridingTime.minutes = totalMinutes % 60;
        ridingTime.hours = totalMinutes / 60;
    }
    else {
        // 如果骑行状态为停止，更新时间为0
        ridingTime.lastUpdateTimeMs = currentTime;
    }
    
    // 如果骑行状态变为停止且计时正在运行，可以选择是否自动停止计时
    // 这里保留计时，但您可以根据需求取消注释下面的代码
    /*
    if (currentRidingState == RIDING_STOPPED && ridingTime.isRunning) {
        ridingTime.isRunning = 0;
    }
    */
}

// 获取总骑行时间(ms)
uint32_t STime_GetTotalRidingTimeMs(void)
{
    return ridingTime.totalTimeMs;
}

// 获取格式化时间
void STime_GetFormattedTime(uint8_t* hours, uint8_t* minutes, uint8_t* seconds, uint16_t* milliseconds)
{
    *hours = ridingTime.hours;
    *minutes = ridingTime.minutes;
    *seconds = ridingTime.seconds;
    *milliseconds = ridingTime.milliseconds;
}

// 获取计时器运行状态
uint8_t STime_IsRunning(void)
{
    return ridingTime.isRunning;
}

// 设置数据锁存状态
void STime_SetDataLocked(uint8_t locked)
{
    g_dataLocked = locked;
}

// 获取数据锁存状态
uint8_t STime_IsDataLocked(void)
{
    return g_dataLocked;
}

// TIM1中断回调函数 - 只包含简单整数运算
void STime_TimerCallback(void)
{
    systemTimeMs++; // 简单整数递增
    
    // 每10ms检查一次霍尔传感器超时 (简单整数运算)
    if (systemTimeMs % 10 == 0) {
        Hall_TimeoutCheck();
    }
}

// TIM1更新中断服务函数
void TIM1_UP_IRQHandler(void)
{
    if (TIM_GetITStatus(TIM1, TIM_IT_Update) != RESET) {
        STime_TimerCallback(); // 调用包含简单计算的回调
		TIM_ClearITPendingBit(TIM1, TIM_IT_Update);
    }
}
