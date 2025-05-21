#ifndef __HALL_H
#define __HALL_H

#include "stm32f10x.h"
#include "key.h"

// 可配置参数
#define WHEEL_CIRCUMFERENCE    1.71f    // 车轮周长m，更正单位说明
#define MAGNET_COUNT           4         // 每转一圈的磁铁数量
#define MEDIAN_FILTER_SIZE     3         // 中值滤波器大小 (建议奇数, 如 3 或 5)

// 骑行状态定义
typedef enum {
    RIDING_STOPPED = 0,
    RIDING_ACTIVE  = 1
} RidingState_t;

// 霍尔传感器数据结构
typedef struct {
    //霍尔传感器捕获数据
    uint32_t lastCaptureValue;     // 上次捕获值
    uint32_t currentCaptureValue;  // 当前捕获值
    uint32_t captureDiff;          // 两次捕获的时间差 (us, 未滤波)
    volatile uint32_t overflowCount;     // 溢出计数器
    volatile uint32_t lastOverflowCount; // 上次捕获时的溢出计数
    //==========================
    uint32_t intervalBuffer[MEDIAN_FILTER_SIZE]; // 中值滤波缓冲区 (us)
    uint8_t intervalBufferIndex;   // 缓冲区当前索引
    uint8_t intervalBufferFull;    // 缓冲区是否已满标志

    uint32_t totalPulses;          // 总脉冲数

    float instantSpeed;            // 瞬时速度(km/h, EWMA平滑后)
    float lastInstantSpeed;        // 上次瞬时速度(km/h, EWMA平滑后)，用于计算加速度
    uint32_t lastSpeedUpdateTime;  // 上次速度更新时间 (ms)，用于计算加速度 (保留，但加速度计算改用median_interval)
    float averageSpeed;            // 平均速度(km/h)
    float totalDistance;           // 总距离(km)

    float acceleration;            // 当前加速度(m/s², 基于平滑速度计算)
    float maxAcceleration;         // 最大加速度(m/s²)

    RidingState_t ridingState;     // 骑行状态，0:停止，1:骑行，结构体嵌套，在前面已经定义了
    uint32_t noSignalTimeout;      // 无信号超时计数

    //判断依据
    float maxSpeed;                // 最大速度(km/h)
    float targetSpeed;             // 目标速度(km/h)
    float targetDistance;          // 目标距离(km)
} HallSensor_t;             //全局变量在hall.c中定义

// 公共接口函数
void Hall_Init(void); // 初始化霍尔传感器，已经初始化结构体了，不需要再初始化结构体
void Hall_Process(void); // 处理霍尔传感器数据
//主函数函数必要函数使用方法
//初始化，Hall_Init();，其他是功能性函数，按需调用


void Hall_SetTargetDistance(float distance); // 设置目标距离
void Hall_SetTargetSpeed(float speed); // 设置目标速度

float Hall_GetInstantSpeed(void); // 获取瞬时速度
float Hall_GetAverageSpeed(void); // 获取平均速度
float Hall_GetTotalDistance(void); // 获取总距离
float Hall_GetMaxSpeed(void); // 获取最大速度
float Hall_GetAcceleration(void);   // 获取当前加速度
float Hall_GetMaxAcceleration(void); // 获取最大加速度
RidingState_t Hall_GetRidingState(void); // 获取骑行状态

void Hall_ResetData(void); // 重置数据

uint8_t Hall_IsTargetDistanceReached(void); // 检查目标距离是否达到，达到返回1，否则返回0
uint8_t Hall_IsTargetSpeedReached(void); // 检查目标速度是否达到，达到返回1，否则返回0

// 添加锁存控制函数声明
void Hall_SetDataLocked(uint8_t locked); // 设置锁存状态
uint8_t Hall_IsDataLocked(void);         // 获取锁存状态

// 中断处理函数(需在定时器中断处理函数中调用，hall的中断用于瞬时速度和加速度的计算，且判断骑行状态，
//且只会赋正在骑行状态，不会赋停止状态，停止状态由超时函数赋值),!!每当进行一次中断时就会清零一次无信号时间
void Hall_TimeoutCheck(void);

extern HallSensor_t hallSensor;

#endif /* __HALL_H */
