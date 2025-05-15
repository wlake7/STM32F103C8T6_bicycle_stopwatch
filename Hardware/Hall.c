#include "Hall.h"
#include "stime.h"
#include <math.h>
#include <stdlib.h> // 用于 qsort 或自定义排序
#include <string.h> // 用于 memcpy

// 新增常量定义
#define EWMA_ALPHA             0.3f      // EWMA 平滑因子 (0 < alpha <= 1)
#define TIM_PERIOD             0xFFFF    // 定时器周期
#define TIM_CLOCK_FREQ         1000000   // 定时器时钟频率 (Hz)，72MHz÷72=1MHz

// 全局变量
HallSensor_t hallSensor = {0}; // 霍尔传感器数据结构实例
//外部变量声明
extern volatile uint32_t systemTimeMs; // 系统时间，单位ms,由stime.c提供，一直自增
//static uint8_t hall_dataLocked = 0;    // 霍尔传感器数据锁存状态

// 静态函数声明
static void Hall_UpdateDistanceAndAverageSpeed(void);
// 简单的排序函数 (冒泡排序，适用于小数组)
static void sort_u32_array(uint32_t *arr, uint8_t size) {
    uint8_t i, j;
    uint32_t temp;
    for (i = 0; i < size - 1; i++) {
        for (j = 0; j < size - i - 1; j++) {
            if (arr[j] > arr[j + 1]) {
                temp = arr[j];
                arr[j] = arr[j + 1];
                arr[j + 1] = temp;
            }
        }
    }
}


// 初始化霍尔传感器
void Hall_Init(void)
{
    GPIO_InitTypeDef GPIO_InitStructure;
    TIM_TimeBaseInitTypeDef TIM_TimeBaseStructure;
    TIM_ICInitTypeDef TIM_ICInitStructure;
    NVIC_InitTypeDef NVIC_InitStructure;
    
    // 使能GPIO和TIM2时钟
    RCC_APB2PeriphClockCmd(RCC_APB2Periph_GPIOA, ENABLE);
    RCC_APB1PeriphClockCmd(RCC_APB1Periph_TIM2, ENABLE);
    
    // 配置PA2为输入模式 (TIM2_CH3)
    GPIO_InitStructure.GPIO_Pin = GPIO_Pin_2;
    GPIO_InitStructure.GPIO_Mode = GPIO_Mode_IN_FLOATING;
    GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;
    GPIO_Init(GPIOA, &GPIO_InitStructure);
    
    // 配置TIM2基本参数
    TIM_TimeBaseStructure.TIM_Period = TIM_PERIOD;
    TIM_TimeBaseStructure.TIM_Prescaler = 72-1;  // 1MHz计数频率
    TIM_TimeBaseStructure.TIM_ClockDivision = TIM_CKD_DIV1;
    TIM_TimeBaseStructure.TIM_CounterMode = TIM_CounterMode_Up;
    TIM_TimeBaseInit(TIM2, &TIM_TimeBaseStructure);
    
    // 配置TIM2_CH3输入捕获
    TIM_ICInitStructure.TIM_Channel = TIM_Channel_3;
    TIM_ICInitStructure.TIM_ICPolarity = TIM_ICPolarity_Rising;
    TIM_ICInitStructure.TIM_ICSelection = TIM_ICSelection_DirectTI;
    TIM_ICInitStructure.TIM_ICPrescaler = TIM_ICPSC_DIV1;
    TIM_ICInitStructure.TIM_ICFilter = 0x03;  // 滤波设置
    TIM_ICInit(TIM2, &TIM_ICInitStructure);
    
    // 配置TIM2中断
    NVIC_InitStructure.NVIC_IRQChannel = TIM2_IRQn;
    NVIC_InitStructure.NVIC_IRQChannelPreemptionPriority = 0;
    NVIC_InitStructure.NVIC_IRQChannelSubPriority = 0;
    NVIC_InitStructure.NVIC_IRQChannelCmd = ENABLE;
    NVIC_Init(&NVIC_InitStructure);
    
    // 使能TIM2捕获中断和溢出中断
    TIM_ITConfig(TIM2, TIM_IT_CC3 | TIM_IT_Update, ENABLE);
    
    // 启动TIM2
    TIM_Cmd(TIM2, ENABLE);
    
    // 初始化霍尔传感器变量
    hallSensor.ridingState = RIDING_STOPPED;
    hallSensor.lastCaptureValue = 0;
    hallSensor.currentCaptureValue = 0;
    hallSensor.captureDiff = 0;
    hallSensor.overflowCount = 0;
    hallSensor.lastOverflowCount = 0;
    hallSensor.totalPulses = 0;
    hallSensor.instantSpeed = 0;
    hallSensor.lastInstantSpeed = 0; // 初始化上次速度为0
    hallSensor.lastSpeedUpdateTime = 0;
    hallSensor.averageSpeed = 0;
    hallSensor.totalDistance = 0;
    hallSensor.noSignalTimeout = 0;
    hallSensor.maxSpeed = 0;
    hallSensor.acceleration = 0;
    hallSensor.maxAcceleration = 0;
    hallSensor.targetSpeed = 0;
    hallSensor.targetDistance = 0;

    // 初始化中值滤波相关变量
    memset(hallSensor.intervalBuffer, 0, sizeof(hallSensor.intervalBuffer));
    hallSensor.intervalBufferIndex = 0;
    hallSensor.intervalBufferFull = 0;
}


// 输入捕获回调函数 (在中断上下文中执行)
// 功能：捕获霍尔信号，使用中值滤波+EWMA计算速度，并计算加速度
void Hall_CaptureCallback(void)
{   
    // 如果数据已锁存，不再处理新的捕获事件
    if ( g_dataLocked) {
        return;
    }
    
    // 获取当前捕获值和溢出计数
    uint32_t captureVal = TIM_GetCapture3(TIM2);
    uint32_t currentOverflowCount = hallSensor.overflowCount;
    uint32_t interval_us; // 捕获间隔 (us)

    // 保存上次捕获值和溢出计数
    hallSensor.lastCaptureValue = hallSensor.currentCaptureValue;
    hallSensor.currentCaptureValue = captureVal;
    
    // --- 1. 计算原始脉冲时间间隔 (us)，考虑溢出 ---
    if (currentOverflowCount == hallSensor.lastOverflowCount) {
        // 未发生溢出，直接计算差值
        hallSensor.captureDiff = hallSensor.currentCaptureValue - hallSensor.lastCaptureValue;
    } else {
        // 发生了一次或多次溢出
        uint32_t overflowDiff = currentOverflowCount - hallSensor.lastOverflowCount;
        hallSensor.captureDiff = (overflowDiff * (TIM_PERIOD + 1)) - hallSensor.lastCaptureValue + hallSensor.currentCaptureValue;
    }
    
    // 更新上次溢出计数
    hallSensor.lastOverflowCount = currentOverflowCount;

    // 忽略过小或过大的间隔 (根据实际情况调整阈值)
    if (hallSensor.captureDiff < 100 || hallSensor.captureDiff > 5000000) { // 100us ~ 5s
         return; // 直接返回不计算了，等待下次捕获再计算
    }

    // --- 2. 更新中值滤波缓冲区 ---
    hallSensor.intervalBuffer[hallSensor.intervalBufferIndex] = hallSensor.captureDiff;
    hallSensor.intervalBufferIndex++;
    if (hallSensor.intervalBufferIndex >= MEDIAN_FILTER_SIZE) {
        hallSensor.intervalBufferIndex = 0;
        hallSensor.intervalBufferFull = 1; // 缓冲区已满
    }

    // --- 3. 获取用于计算的时间间隔 (中值或原始值) ---
    if (hallSensor.intervalBufferFull) {
        // 缓冲区已满，计算中位数 - 优化排序算法
        // 对于很小的数组(如3或5个元素)，可以使用更简单的排序方法
        if (MEDIAN_FILTER_SIZE == 3) {
            // 对于3个元素的数组，使用简单比较找出中间值
            uint32_t a = hallSensor.intervalBuffer[0];
            uint32_t b = hallSensor.intervalBuffer[1];
            uint32_t c = hallSensor.intervalBuffer[2];
            
            // 找出中间值
            if ((a <= b && b <= c) || (c <= b && b <= a))
                interval_us = b;
            else if ((b <= a && a <= c) || (c <= a && a <= b))
                interval_us = a;
            else
                interval_us = c;
        } else {
            // 对于较大的数组，仍使用常规排序
            uint32_t temp_buffer[MEDIAN_FILTER_SIZE];
            memcpy(temp_buffer, hallSensor.intervalBuffer, sizeof(temp_buffer));
            sort_u32_array(temp_buffer, MEDIAN_FILTER_SIZE);
            interval_us = temp_buffer[MEDIAN_FILTER_SIZE / 2]; // 取中间值
        }
    } else {
        // 缓冲区未满，使用原始间隔
        interval_us = hallSensor.captureDiff;
    }

    // 计数脉冲
    hallSensor.totalPulses++;

    // 设置骑行状态
    hallSensor.ridingState = RIDING_ACTIVE; // 中断函数中设置骑行状态为ACTIVE
    hallSensor.noSignalTimeout = 0; // 重置超时计数器
    //ridingTime.isRunning = 1;
    // --- 4. 计算瞬时速度 (基于 interval_us) ---
    if (interval_us > 0) {
        // 预计算常量部分，减少重复计算
        // 修正：WHEEL_CIRCUMFERENCE应为4.0米(400厘米)，而非400米
        const float distancePerPulse = (WHEEL_CIRCUMFERENCE / 100.0f) / MAGNET_COUNT; // 每个脉冲的距离(米)
        const float speedConvFactor = distancePerPulse * 3.6f; // 3.6 = 3600/1000 (转换为km/h)
        
        float timeInS = interval_us * 0.000001f; // 使用乘法代替除法
        
        // 计算原始瞬时速度 (km/h)
        float raw_speed_kph = speedConvFactor / timeInS;

        // --- 5. 对速度进行 EWMA 平滑 ---
        // 处理第一次计算的情况
        if (hallSensor.totalPulses <= 1 || !hallSensor.intervalBufferFull) { 
             hallSensor.instantSpeed = raw_speed_kph;
        } else {
             // 优化: 避免重复计算 (1.0f - EWMA_ALPHA)(因子越大越平滑但响应越慢，因为更加看重历史数据)
             hallSensor.instantSpeed = EWMA_ALPHA * raw_speed_kph + (1.0f - EWMA_ALPHA) * hallSensor.instantSpeed;
        }

        // 更新最大速度 (使用平滑后的速度)
        if (hallSensor.instantSpeed > hallSensor.maxSpeed) {
            hallSensor.maxSpeed = hallSensor.instantSpeed;
        }

        // --- 6. 计算瞬时加速度 (m/s²) ---
        // 使用平滑后的速度和中值滤波后的时间间隔计算
        if (hallSensor.totalPulses > 1 && hallSensor.intervalBufferFull) { 
            // 避免重复计算的转换因子
            static const float speedConvToMS = 1.0f / 3.6f; // km/h 转 m/s
            
            float currentSpeedMS = hallSensor.instantSpeed * speedConvToMS;
            float lastSpeedMS = hallSensor.lastInstantSpeed * speedConvToMS;
            
            // 避免重复计算 interval_us / 1000000.0f
            if (timeInS > 0.0001f) { 
                hallSensor.acceleration = (currentSpeedMS - lastSpeedMS) / timeInS;

                // 只更新正的最大加速度
                if (hallSensor.acceleration > hallSensor.maxAcceleration) {
                    hallSensor.maxAcceleration = hallSensor.acceleration;
                }
            } else {
                hallSensor.acceleration = 0;
            }
        } else {
            hallSensor.acceleration = 0; // 初始阶段或间隔无效时加速度为0
        }

        // --- 7. 更新上一次平滑速度 ---
        hallSensor.lastInstantSpeed = hallSensor.instantSpeed;
    } else {
        // interval_us 为 0 或无效，将速度和加速度置零
        hallSensor.instantSpeed = 0;
        hallSensor.acceleration = 0;
        hallSensor.lastInstantSpeed = 0; // 重置上次速度
    }
}

//======用在stime中======
// 超时检查函数，用于判断骑行状态
void Hall_TimeoutCheck(void)
{
    // 如果数据已锁存，不再检查超时
    if (g_dataLocked) {
        return;
    }
    
    // 如果正在骑行，检查是否超时
    if (hallSensor.ridingState == RIDING_ACTIVE) {
        hallSensor.noSignalTimeout++;//目的是判断骑行->停止，当长时间没进hall中断清零就判断为停止
        
        // 超时阈值调整为2秒 (调用周期为10ms，时间为10ms * X)
        if (hallSensor.noSignalTimeout >= 200) {
            hallSensor.ridingState = RIDING_STOPPED;
            hallSensor.instantSpeed = 0; // 确保速度清零
            hallSensor.acceleration = 0; // 停止时加速度也为0
            hallSensor.lastInstantSpeed = 0; // 重置上次速度
            hallSensor.noSignalTimeout = 0; // 重置计数器
            ridingTime.isRunning = 0; // 停止计时器 (假设有此变量),逻辑bug不影响使用
            // 重置滤波缓冲区状态
            hallSensor.intervalBufferIndex = 0;
            hallSensor.intervalBufferFull = 0;
            memset(hallSensor.intervalBuffer, 0, sizeof(hallSensor.intervalBuffer));
        }
    }
}
//==================================================================================================================================
// 功能：更新总距离和平均速度
static void Hall_UpdateDistanceAndAverageSpeed(void)
{
    // 计算总距离 (km) - 浮点运算
    float totalRotations = (float)hallSensor.totalPulses / MAGNET_COUNT;
    hallSensor.totalDistance = totalRotations * WHEEL_CIRCUMFERENCE / 1000.0f;

    // 计算平均速度 (km/h) - 浮点运算
    uint32_t totalRidingTime = STime_GetTotalRidingTimeMs();
    if (totalRidingTime > 0) {
        // 平均速度 = 总距离(km) / 总时间(小时)
        hallSensor.averageSpeed = (hallSensor.totalDistance * 3600000.0f) / totalRidingTime;
    } else {
        hallSensor.averageSpeed = 0;
    }
 
}

//==================================================================================================================================
// 数据处理函数，在主循环中调用
void Hall_Process(void)
{
    // 如果数据已锁存，不再更新计算
    if (g_dataLocked) {
        return;
    }
    
    // 实时更新总距离和平均速度，而不是等待脉冲才更新
    Hall_UpdateDistanceAndAverageSpeed();
    
    // 确保在停止状态下清零瞬时速度
    if (hallSensor.ridingState == RIDING_STOPPED) {
        hallSensor.instantSpeed = 0;
        hallSensor.acceleration = 0;
    }
    /*
    // 调整超时检查时间以适应主循环调用频率，保留选项，似乎没必要
    static uint32_t lastCheckTime = 0;
    uint32_t currentTime = systemTimeMs;
    
    if (currentTime - lastCheckTime >= 10) { // 每10ms检查一次
        Hall_TimeoutCheck();
        lastCheckTime = currentTime;
    }
    */
}

//=================
// ... (Get/Set/Check 函数保持不变) ...
// 设置目标距离
void Hall_SetTargetDistance(float distance)
{
    hallSensor.targetDistance = distance;
}

// 设置目标速度
void Hall_SetTargetSpeed(float speed)
{
    hallSensor.targetSpeed = speed;
}

// 获取瞬时速度
float Hall_GetInstantSpeed(void)//平滑后
{
    // 如果停止，返回0
    // if (hallSensor.ridingState == RIDING_STOPPED) { // TimeoutCheck会清零
    //     return 0.0f;
    // }
    return hallSensor.instantSpeed; // 直接返回平滑后的速度
}

// 获取平均速度
float Hall_GetAverageSpeed(void)
{
    return hallSensor.averageSpeed;
}

// 获取总距离
float Hall_GetTotalDistance(void)
{
    return hallSensor.totalDistance;
}

// 获取最大速度
float Hall_GetMaxSpeed(void)
{
    return hallSensor.maxSpeed;
}

// 获取当前加速度
float Hall_GetAcceleration(void)
{
     // 如果停止，返回0
    // if (hallSensor.ridingState == RIDING_STOPPED) { // TimeoutCheck会清零
    //    return 0.0f;
    // }
    return hallSensor.acceleration;
}

// 获取最大加速度
float Hall_GetMaxAcceleration(void)
{
    return hallSensor.maxAcceleration;
}

// 获取骑行状态
RidingState_t Hall_GetRidingState(void)
{
    return hallSensor.ridingState;
}

// 设置霍尔传感器数据锁存状态
/*
void Hall_SetDataLocked(uint8_t locked)
{
    hall_dataLocked = locked;
}
*/
/*
// 获取霍尔传感器数据锁存状态
uint8_t Hall_IsDataLocked(void)
{
    return hall_dataLocked;
}
*/
// 重置数据
void Hall_ResetData(void)
{
    // 如果数据已锁存，需要先解除锁存
    //hall_dataLocked = 0;
    
    // 禁用中断以保证原子性 (可选，但推荐)
    NVIC_DisableIRQ(TIM2_IRQn);

    hallSensor.totalPulses = 0;
    hallSensor.instantSpeed = 0;
    hallSensor.lastInstantSpeed = 0;
    hallSensor.lastSpeedUpdateTime = 0;
    hallSensor.averageSpeed = 0;
    hallSensor.totalDistance = 0;
    hallSensor.maxSpeed = 0;
    hallSensor.acceleration = 0;
    hallSensor.maxAcceleration = 0;
    // ridingState 不重置，由 TimeoutCheck 或 CaptureCallback 管理
    // noSignalTimeout 在状态改变时重置
    // lastCaptureValue, currentCaptureValue, captureDiff 会在下次捕获时更新

    // 重置溢出相关计数器
    hallSensor.overflowCount = 0;
    hallSensor.lastOverflowCount = 0;

    // 重置中值滤波相关变量
    memset(hallSensor.intervalBuffer, 0, sizeof(hallSensor.intervalBuffer));
    hallSensor.intervalBufferIndex = 0;
    hallSensor.intervalBufferFull = 0;


    // 如果需要重置骑行时间，需要调用stime相关函数
    // STime_ResetRidingTime(); // 假设有此函数

    // 重新使能中断
    NVIC_EnableIRQ(TIM2_IRQn);
}

// ... (IsTargetReached 函数保持不变) ...
// 检查是否达到目标距离
uint8_t Hall_IsTargetDistanceReached(void)
{
    if (hallSensor.targetDistance <= 0) return 0;
    return (hallSensor.totalDistance >= hallSensor.targetDistance) ? 1 : 0;
}

// 检查是否达到目标速度
uint8_t Hall_IsTargetSpeedReached(void)
{
    if (hallSensor.targetSpeed <= 0) return 0;
    // 使用当前获取的瞬时速度进行判断
    return (Hall_GetInstantSpeed() >= hallSensor.targetSpeed) ? 1 : 0;
}
//=============

// TIM2中断服务函数
void TIM2_IRQHandler(void)
{
    // 检查溢出中断
    if (TIM_GetITStatus(TIM2, TIM_IT_Update) != RESET)
    {
        // 清除溢出中断标志
        TIM_ClearITPendingBit(TIM2, TIM_IT_Update);
        
        // 增加溢出计数
        hallSensor.overflowCount++;
    }

    // 检查是否发生了通道3的捕获事件
    if (TIM_GetITStatus(TIM2, TIM_IT_CC3) != RESET)
    {
        // 调用霍尔传感器捕获回调函数
        Hall_CaptureCallback(); // 回调函数内部会读取捕获值
        
        // 清除中断标志 (官方建议在处理完中断后清除)
        TIM_ClearITPendingBit(TIM2, TIM_IT_CC3);
    }
    // 可以添加处理其他TIM2中断源的代码
}
