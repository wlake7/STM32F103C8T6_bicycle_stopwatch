#include "key.h"
#include "stime.h"
#include "Hall.h"
#include "sg04.h"
#include "usart.h" // 添加串口头文件以使用蓝牙发送功能
#include <math.h>  // 添加数学库，提供 fabs 函数

// 添加NULL的定义
#ifndef NULL
#define NULL ((void *)0)
#endif

// 默认设置值和调整步长
#define DEFAULT_SPEED          20.0f    // 默认目标速度 (km/h)
#define DEFAULT_DISTANCE       5.0f     // 默认目标距离 (km)
#define SPEED_STEP            1.0f      // 速度调整步长 (km/h)
#define DISTANCE_STEP         0.5f      // 距离调整步长 (km)
#define SPEED_MAX             50.0f     // 最大可设置速度 (km/h)
#define SPEED_MIN             5.0f      // 最小可设置速度 (km/h)
#define DISTANCE_MAX          100.0f    // 最大可设置距离 (km)
#define DISTANCE_MIN          0.5f      // 最小可设置距离 (km)

#define LED_BLINK_INTERVAL    500       // LED闪烁间隔 (ms)

// 全局变量定义
SetState_t g_setState = SET_STATE_SPEED;  // 初始状态为设置速度
float g_targetSpeed = DEFAULT_SPEED;      // 默认目标速度
float g_targetDistance = DEFAULT_DISTANCE;// 默认目标距离
DisplayMode_t g_displayMode = DISPLAY_REALTIME; // 默认显示实时数据
uint8_t g_distanceTargetReached = 0;      // 距离目标达成标志
uint8_t g_speedTargetReached = 0;         // 速度目标达成标志

// 添加蓝牙数据发送标志位和重试次数控制
static uint8_t g_bluetoothDataSent = 0;   // 蓝牙数据发送标志，0表示未发送，1表示已发送
static uint8_t g_bluetoothSendRetries = 0; // 蓝牙发送重试次数
#define MAX_BLUETOOTH_SEND_RETRIES 3      // 最大重试次数

// 添加蓝牙数据发送时间控制
static uint32_t g_bluetoothLastSendTime = 0;   // 上次蓝牙发送时间
#define BLUETOOTH_SEND_INTERVAL 200           // 蓝牙发送间隔(毫秒)

// 锁存数据
static struct {
    float distance;         // 锁存的距离
    float averageSpeed;     // 锁存的平均速度
    float maxAcceleration;  // 锁存的最大加速度
    uint8_t hours;          // 锁存的小时
    uint8_t minutes;        // 锁存的分钟
    uint8_t seconds;        // 锁存的秒
} lockedData = {0};


// 防抖延时
#define KEY_DEBOUNCE_TIME     20        // 按键防抖时间(ms)

// 函数声明 - 添加到文件顶部以解决声明问题
void Display_LockedData(void);
void Display_RealtimeData(float slope, float direction);

// 按键初始化函数
void Key_Init(void)
{
    GPIO_InitTypeDef GPIO_InitStructure;
    
    // 启用GPIO时钟
    RCC_APB2PeriphClockCmd(RCC_APB2Periph_GPIOA | RCC_APB2Periph_GPIOB, ENABLE);
    
    // 配置PB按键引脚为输入上拉模式
    GPIO_InitStructure.GPIO_Pin = KEY_UP_PIN | KEY_DOWN_PIN;
    GPIO_InitStructure.GPIO_Mode = GPIO_Mode_IPU;  // 输入上拉
    GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;
    GPIO_Init(KEY_PORT, &GPIO_InitStructure);
    
    // 配置PA8设置按键为输入上拉模式
    GPIO_InitStructure.GPIO_Pin = KEY_SET_PIN;
    GPIO_InitStructure.GPIO_Mode = GPIO_Mode_IPU;
    GPIO_Init(KEY_SET_PORT, &GPIO_InitStructure);
    
    // 配置PA7锁存按键为输入上拉模式
    GPIO_InitStructure.GPIO_Pin = KEY_LOCK_PIN;
    GPIO_InitStructure.GPIO_Mode = GPIO_Mode_IPU;
    GPIO_Init(KEY_LOCK_PORT, &GPIO_InitStructure);
    
    // 配置LED和蜂鸣器引脚为推挽输出
    GPIO_InitStructure.GPIO_Pin = LED1_PIN | LED2_PIN | BUZZER_PIN;
    GPIO_InitStructure.GPIO_Mode = GPIO_Mode_Out_PP;  // 推挽输出
    GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;
    GPIO_Init(LED_BUZZER_PORT, &GPIO_InitStructure);
    
    // 初始状态: LED关闭，蜂鸣器关闭
    GPIO_ResetBits(LED_BUZZER_PORT, LED1_PIN | LED2_PIN );
    GPIO_SetBits(LED_BUZZER_PORT,BUZZER_PIN); // 关闭LED
}

// 配置按键的外部中断
void Key_EXTI_Config(void)
{
    EXTI_InitTypeDef EXTI_InitStructure;
    NVIC_InitTypeDef NVIC_InitStructure;
    
    // 启用系统配置控制器时钟和AFIO时钟
    RCC_APB2PeriphClockCmd(RCC_APB2Periph_AFIO, ENABLE);
    
    // 连接EXTI线到GPIO引脚
    GPIO_EXTILineConfig(GPIO_PortSourceGPIOB, GPIO_PinSource0);  // PB0 (UP)
    GPIO_EXTILineConfig(GPIO_PortSourceGPIOB, GPIO_PinSource1);  // PB1 (DOWN)
    GPIO_EXTILineConfig(GPIO_PortSourceGPIOA, GPIO_PinSource8);  // PA8 (SET) - 修改为PA8
    GPIO_EXTILineConfig(GPIO_PortSourceGPIOA, GPIO_PinSource7);  // PA7 (LOCK)
    
    // 配置EXTI线0 (PB0 - UP键)
    EXTI_InitStructure.EXTI_Line = EXTI_Line0;
    EXTI_InitStructure.EXTI_Mode = EXTI_Mode_Interrupt;
    EXTI_InitStructure.EXTI_Trigger = EXTI_Trigger_Falling;  // 下降沿触发（按键按下）
    EXTI_InitStructure.EXTI_LineCmd = ENABLE;
    EXTI_Init(&EXTI_InitStructure);
    
    // 配置EXTI线1 (PB1 - DOWN键)
    EXTI_InitStructure.EXTI_Line = EXTI_Line1;
    EXTI_Init(&EXTI_InitStructure);
    
    // 配置EXTI线7 (PA7 - LOCK键)
    EXTI_InitStructure.EXTI_Line = EXTI_Line7;
    EXTI_Init(&EXTI_InitStructure);
    
    // 配置EXTI线8 (PA8 - SET键) - 新增
    EXTI_InitStructure.EXTI_Line = EXTI_Line8;
    EXTI_Init(&EXTI_InitStructure);
    
    // 配置NVIC优先级
    NVIC_InitStructure.NVIC_IRQChannel = EXTI0_IRQn;  // UP键
    NVIC_InitStructure.NVIC_IRQChannelPreemptionPriority = 1;
    NVIC_InitStructure.NVIC_IRQChannelSubPriority = 0;
    NVIC_InitStructure.NVIC_IRQChannelCmd = ENABLE;
    NVIC_Init(&NVIC_InitStructure);
    
    NVIC_InitStructure.NVIC_IRQChannel = EXTI1_IRQn;  // DOWN键
    NVIC_Init(&NVIC_InitStructure);
    
    NVIC_InitStructure.NVIC_IRQChannel = EXTI9_5_IRQn; // SET键和LOCK键共用
    NVIC_Init(&NVIC_InitStructure);
}

// 显示设置界面
void Key_DisplaySettings(void)
{
    OLED_Clear();
    
    // 显示标题
    OLED_ShowString(1, 1, "Settings");
    
    // 根据当前状态显示不同内容
    switch(g_setState)
    {
        case SET_STATE_SPEED:
        {
            OLED_ShowString(2, 1, "Speed:");
            OLED_ShowString(3, 5, "km/h");
            
            // 显示速度值 (分整数和小数部分)
            uint32_t speed_int = (uint32_t)g_targetSpeed;
            uint32_t speed_frac = (uint32_t)((g_targetSpeed - speed_int) * 10) % 10;
            OLED_ShowNum(3, 1, speed_int, 2);
            OLED_ShowChar(3, 3, '.');
            OLED_ShowNum(3, 4, speed_frac, 1);
            
            OLED_ShowString(4, 1, "UP to change");
            break;
        }
            
        case SET_STATE_DISTANCE:
        {
            OLED_ShowString(2, 1, "Distance:");
            OLED_ShowString(3, 5, "km");
            
            // 显示距离值 (分整数和小数部分)
            uint32_t dist_int = (uint32_t)g_targetDistance;
            uint32_t dist_frac = (uint32_t)((g_targetDistance - dist_int) * 10) % 10;
            OLED_ShowNum(3, 1, dist_int, 2);
            OLED_ShowChar(3, 3, '.');
            OLED_ShowNum(3, 4, dist_frac, 1);
            
            OLED_ShowString(4, 1, "SET to save");
            break;
        }
            
        case SET_STATE_DONE:
        {
            OLED_ShowString(2, 1, "OK");
            OLED_ShowString(3, 1, "Speed:");
            OLED_ShowString(4, 1, "Dist:");
            
            // 显示最终设置值
            uint32_t final_speed_int = (uint32_t)g_targetSpeed;
            uint32_t final_speed_frac = (uint32_t)((g_targetSpeed - final_speed_int) * 10) % 10;
            OLED_ShowNum(3, 7, final_speed_int, 2);
            OLED_ShowChar(3, 9, '.');
            OLED_ShowNum(3, 10, final_speed_frac, 1);
            
            uint32_t final_dist_int = (uint32_t)g_targetDistance;
            uint32_t final_dist_frac = (uint32_t)((g_targetDistance - final_dist_int) * 10) % 10;
            OLED_ShowNum(4, 6, final_dist_int, 2);
            OLED_ShowChar(4, 8, '.');
            OLED_ShowNum(4, 9, final_dist_frac, 1);
            break;
        }
    }
}

// 设置菜单主函数 (阻塞式)
void Key_SettingsMenu(void)
{
    // 初始化按键和外部中断
    Key_Init();
    Key_EXTI_Config();
    
    // 初始化设置状态和默认值
    g_setState = SET_STATE_SPEED;
    g_targetSpeed = DEFAULT_SPEED;
    g_targetDistance = DEFAULT_DISTANCE;
    
    // 显示初始设置界面
    Key_DisplaySettings();
    
    // 阻塞等待设置完成
    while(g_setState != SET_STATE_DONE)
    {
        // 等待中断
        __WFI();  // 等待中断发生
    }
    
    // 设置完成后保存设置值
    Hall_SetTargetSpeed(g_targetSpeed);
    Hall_SetTargetDistance(g_targetDistance);
    
    // 显示最终设置界面
    Key_DisplaySettings();
    
    // 延时一段时间再退出，让用户看到设置已保存
    sys_Delay_ms(1500);
    
    // 清屏，准备进入主功能界面
    OLED_Clear();
}

// 中断服务函数 - UP按键 (PB0)
void EXTI0_IRQHandler(void)
{
    // 检查是否为EXTI0中断
    if(EXTI_GetITStatus(EXTI_Line0) != RESET)
    {
        // 按键消抖
        sys_Delay_ms(KEY_DEBOUNCE_TIME);
        
        // 确认按键仍然按下
        if(GPIO_ReadInputDataBit(KEY_PORT, KEY_UP_PIN) == Bit_RESET)
        {
            // 根据当前状态处理UP键
            switch(g_setState)
            {
                case SET_STATE_SPEED:
                    // 增加目标速度
                    g_targetSpeed += SPEED_STEP;
                    if(g_targetSpeed > SPEED_MAX)
                        g_targetSpeed = SPEED_MAX;
                    Key_DisplaySettings();
                    break;
                    
                case SET_STATE_DISTANCE:
                    // 增加目标距离
                    g_targetDistance += DISTANCE_STEP;
                    if(g_targetDistance > DISTANCE_MAX)
                        g_targetDistance = DISTANCE_MAX;
                    Key_DisplaySettings();
                    break;
                    
                default:
                    break;
            }
        }
        
        // 清除中断标志位
        EXTI_ClearITPendingBit(EXTI_Line0);
    }
}

// 中断服务函数 - DOWN按键 (PB1)
void EXTI1_IRQHandler(void)
{
    // 检查是否为EXTI1中断
    if(EXTI_GetITStatus(EXTI_Line1) != RESET)
    {
        // 按键消抖
        sys_Delay_ms(KEY_DEBOUNCE_TIME);
        
        // 确认按键仍然按下
        if(GPIO_ReadInputDataBit(KEY_PORT, KEY_DOWN_PIN) == Bit_RESET)
        {
            // 根据当前状态处理DOWN键
            switch(g_setState)
            {
                case SET_STATE_SPEED:
                    // 减少目标速度
                    g_targetSpeed -= SPEED_STEP;
                    if(g_targetSpeed < SPEED_MIN)
                        g_targetSpeed = SPEED_MIN;
                    Key_DisplaySettings();
                    break;
                    
                case SET_STATE_DISTANCE:
                    // 减少目标距离
                    g_targetDistance -= DISTANCE_STEP;
                    if(g_targetDistance < DISTANCE_MIN)
                        g_targetDistance = DISTANCE_MIN;
                    Key_DisplaySettings();
                    break;
                    
                default:
                    break;
            }
        }
        
        // 清除中断标志位
        EXTI_ClearITPendingBit(EXTI_Line1);
    }
}

// 中断服务函数 - SET按键 (PA8) 和 LOCK按键 (PA7) 共享的EXTI9_5
void EXTI9_5_IRQHandler(void)
{
    // 检查是否为EXTI7中断 (LOCK键 PA7)
    if(EXTI_GetITStatus(EXTI_Line7) != RESET)
    {
        // 延长按键消抖时间，确保稳定
        sys_Delay_ms(KEY_DEBOUNCE_TIME);
        
        if(GPIO_ReadInputDataBit(KEY_LOCK_PORT, KEY_LOCK_PIN) == Bit_RESET)
        {
            // LOCK键按下处理
            if(g_setState == SET_STATE_DONE && g_displayMode == DISPLAY_REALTIME && !STime_IsDataLocked())
            {
                // 保存当前数据到锁存结构体
                lockedData.distance = Hall_GetTotalDistance();
                lockedData.averageSpeed = Hall_GetAverageSpeed();
                lockedData.maxAcceleration = Hall_GetMaxAcceleration();
                
                // 获取时间
                STime_GetFormattedTime(&lockedData.hours, &lockedData.minutes, 
                                      &lockedData.seconds, 0);  // 使用 0 替代 NULL
                
                // 设置锁存状态，确保所有数据处理都停止
                STime_SetDataLocked(1);
                
                // 停止计时
                STime_Stop();
                
                // 切换到锁定显示模式
                g_displayMode = DISPLAY_LOCKED;
                
                // 清屏准备显示锁存数据
                OLED_Clear();
                
                // 重置蓝牙数据发送标志和重试计数
                g_bluetoothDataSent = 0;
                g_bluetoothSendRetries = 0;
                
                // 重置蓝牙数据发送标志和发送时间
                g_bluetoothLastSendTime = 0;
            }
        }
        
        // 确保足够的延时，等待按键稳定释放
        sys_Delay_ms(20);
        
        // 清除中断标志位
        EXTI_ClearITPendingBit(EXTI_Line7);
    }
    
    // 检查是否为EXTI8中断 (SET键 PA8)
    if(EXTI_GetITStatus(EXTI_Line8) != RESET)
    {
        // 按键消抖
        sys_Delay_ms(KEY_DEBOUNCE_TIME);
        
        // 确认SET键仍然按下
        if(GPIO_ReadInputDataBit(KEY_SET_PORT, KEY_SET_PIN) == Bit_RESET)
        {
            // SET键按下处理
            if (g_setState == SET_STATE_DONE && g_displayMode == DISPLAY_LOCKED && 
                STime_IsDataLocked()) {
                // 在锁定模式下，SET键用于开始新骑行
                // 解除数据锁存
                STime_SetDataLocked(0);
                
                // 重置数据
                Hall_ResetData();
                STime_Reset();
                
                // 切换回实时显示模式
                g_displayMode = DISPLAY_REALTIME;
                OLED_Clear();    
                // 开始新的骑行
                STime_Start();
                
                // 重置蓝牙数据发送标志和重试计数
                g_bluetoothDataSent = 0;
                g_bluetoothSendRetries = 0;
                
                // 切换回实时显示模式时重置蓝牙相关变量
                g_bluetoothLastSendTime = 0;
            } 
            else {
                // 在设置模式下的处理
                switch(g_setState)
                {
                    case SET_STATE_SPEED:
                        // 从设置速度切换到设置距离
                        g_setState = SET_STATE_DISTANCE;
                        Key_DisplaySettings();
                        break;
                        
                    case SET_STATE_DISTANCE:
                        // 完成设置流程
                        g_setState = SET_STATE_DONE;
                        Key_DisplaySettings();
                        break;
                        
                    default:
                        break;
                }
            }
        }
        
        // 清除中断标志位
        EXTI_ClearITPendingBit(EXTI_Line8);
    }
}

// LED控制函数
void LED_Control(uint16_t LED_Pin, uint8_t state)
{
    if(state)
    {
        GPIO_SetBits(LED_BUZZER_PORT, LED_Pin);
    }
    else
    {
        GPIO_ResetBits(LED_BUZZER_PORT, LED_Pin);
    }
}

// LED翻转状态
void LED_Toggle(uint16_t LED_Pin)
{
    if(GPIO_ReadOutputDataBit(LED_BUZZER_PORT, LED_Pin))
    {
        GPIO_ResetBits(LED_BUZZER_PORT, LED_Pin);
    }
    else
    {
        GPIO_SetBits(LED_BUZZER_PORT, LED_Pin);
    }
}

// 蜂鸣器控制函数
void Buzzer_OFF(void)
{
    GPIO_SetBits(LED_BUZZER_PORT, BUZZER_PIN);//设置电平为
}

void Buzzer_ON(void)
{
    GPIO_ResetBits(LED_BUZZER_PORT, BUZZER_PIN);
}

// 检查按键是否被按下
uint8_t Key_IsPressed(GPIO_TypeDef* GPIOx, uint16_t GPIO_Pin)
{
    // 消抖处理
    if(GPIO_ReadInputDataBit(GPIOx, GPIO_Pin) == Bit_RESET)
    {
        sys_Delay_ms(KEY_DEBOUNCE_TIME);
        if(GPIO_ReadInputDataBit(GPIOx, GPIO_Pin) == Bit_RESET)
        {
            return 1;  // 按键被按下
        }
    }
    return 0;  // 按键未被按下
}

// 锁定数据显示函数
void Display_LockedData(void)
{   

    // 显示标题
    OLED_ShowString(1, 1, "Locked Data");
    
    // 显示锁定的骑行距离
    OLED_ShowString(2, 1, "Dist:");
    
    // 显示距离，小数点分隔
    uint32_t dist_int = (uint32_t)lockedData.distance;
    uint32_t dist_frac = (uint32_t)((lockedData.distance - dist_int) * 10) % 10;
    OLED_ShowNum(2, 7, dist_int, 2);
    OLED_ShowChar(2, 9, '.');
    OLED_ShowNum(2, 10, dist_frac, 1);
    OLED_ShowString(2, 12, "km");
    
    // 显示平均速度
    OLED_ShowString(3, 1, "AvgV:");
    uint32_t avgv_int = (uint32_t)lockedData.averageSpeed;
    uint32_t avgv_frac = (uint32_t)((lockedData.averageSpeed - avgv_int) * 10) % 10;
    OLED_ShowNum(3, 7, avgv_int, 2);
    OLED_ShowChar(3, 9, '.');
    OLED_ShowNum(3, 10, avgv_frac, 1);
    OLED_ShowString(3, 12, "km/h");
    
    // 显示骑行时间
    OLED_ShowString(4, 1, "Time:");
    OLED_ShowNum(4, 7, lockedData.hours, 2);
    OLED_ShowChar(4, 9, ':');
    OLED_ShowNum(4, 10, lockedData.minutes, 2);
    OLED_ShowChar(4, 12, ':');
    OLED_ShowNum(4, 13, lockedData.seconds, 2);
}

// 实时数据显示函数
void Display_RealtimeData(float slope, float direction)
{
    
    // 显示骑行状态
    RidingState_t rideState = Hall_GetRidingState();
    if(rideState == RIDING_ACTIVE)
    {
        OLED_ShowString(1, 1, "S");
    }
    else
    {
        OLED_ShowString(1, 1, "T");
    }
    // 显示当前速度
    float speed = Hall_GetAverageSpeed(); // 获取平均速度
    uint32_t speed_int = (uint32_t)speed;
    uint32_t speed_frac = (uint32_t)((speed - speed_int) * 10) % 10;
    OLED_ShowString(1, 4, "V:");
    //OLED_ShowFloat(1, 6, speed, 3, 2); // 显示速度，保留两位整数和一位小数
    OLED_ShowNum(1, 6, speed_int, 2);
    OLED_ShowChar(1, 8, '.');
    OLED_ShowNum(1, 9, speed_frac, 1);
    OLED_ShowString(1, 10, "km/h");
    
    // 显示骑行距离 - 修改为更精确的显示方式
    float distance = Hall_GetTotalDistance();
    OLED_ShowString(2, 1, "D:");
    
    // 增加小数点后两位显示，增强精度
    uint32_t dist_int = (uint32_t)distance;
    uint32_t dist_frac = (uint32_t)((distance - dist_int) * 100) % 100; // 显示两位小数
    OLED_ShowNum(2, 3, dist_int, 2);
    OLED_ShowChar(2, 5, '.');
    // 确保小数部分显示两位数字
    if (dist_frac < 10) {
        OLED_ShowChar(2, 6, '0');
        OLED_ShowNum(2, 7, dist_frac, 1);
    } else {
        OLED_ShowNum(2, 6, dist_frac, 2);
    }
    OLED_ShowString(2, 9, "km");
    

    
    // 显示坡度和方向
    uint32_t slope_int = (uint32_t)fabs(slope);  // 使用标准fabs替代fabsf
    uint32_t slope_frac = (uint32_t)((fabs(slope) - slope_int) * 10) % 10;
    OLED_ShowString(3, 1, "G:");
    if(slope < 0)
    {
        OLED_ShowChar(3, 3, '-');
    }
    else
    {
        OLED_ShowChar(3, 3, '+');
    }
    OLED_ShowNum(3, 4, slope_int, 2);
    OLED_ShowChar(3, 6, '.');
    OLED_ShowNum(3, 7, slope_frac, 1);
    OLED_ShowString(3, 8, "%");
    
    // 显示方向 (简化为8个基本方向)
    if(direction >= 337.5f || direction < 22.5f)
        OLED_ShowString(3, 10, "North");
    else if(direction >= 22.5f && direction < 67.5f)
        OLED_ShowString(3, 10, "NE");
    else if(direction >= 67.5f && direction < 112.5f)
        OLED_ShowString(3, 10, "East");
    else if(direction >= 112.5f && direction < 157.5f)
        OLED_ShowString(3, 10, "SE");
    else if(direction >= 157.5f && direction < 202.5f)
        OLED_ShowString(3, 10, "South");
    else if(direction >= 202.5f && direction < 247.5f)
        OLED_ShowString(3, 10, "SW");
    else if(direction >= 247.5f && direction < 292.5f)
        OLED_ShowString(3, 10, "West");
    else
        OLED_ShowString(3, 10, "NW");
    // 显示骑行时间 - 改为更直观的格式，同时显示秒数以方便验证
    OLED_ShowString(4, 1, "T:");
    
    // 显示时:分:秒格式
    OLED_ShowNum(4, 3, ridingTime.hours, 2);
    OLED_ShowChar(4, 5, ':');
    OLED_ShowNum(4, 6, ridingTime.minutes, 2);
    OLED_ShowChar(4, 8, ':');
    OLED_ShowNum(4, 9, ridingTime.seconds, 2);
    /*
    // 显示总秒数以便验证 (调试用)
    uint32_t totalSec = ridingTime.totalTimeMs / 100;
    OLED_ShowString(4, 12, "s:");
    OLED_ShowNum(4, 14, totalSec, 3);
    */
}

// 让LED闪烁指定次数
void LED_Blink(uint16_t LED_Pin, uint8_t times)
{
    for (uint8_t i = 0; i < times; i++)
    {
        // 打开LED
        LED_Control(LED_Pin, 1);
        sys_Delay_ms(200);  // 亮200ms
        
        // 关闭LED
        LED_Control(LED_Pin, 0);
        
        // 如果不是最后一次循环，则延时
        if (i < times - 1)
        {
            sys_Delay_ms(200);  // 灭200ms
        }
    }
}

// 发送锁存数据包函数
void Serial_SendLockedPacket(void)
{
    // 初始化数据包
    Serial_TxPacket[0] = 0xA5;  // 包头 (1字节)
    Serial_TxPacket[TX_Data_Len-1] = 0x5A;  // 包尾 (1字节)
    

    // 字节型数据装配 (3个字节, 共3字节)
    Serial_Byte(lockedData.hours, &Serial_TxPacket[1]);  // 小时 (1字节)
    Serial_Byte(lockedData.minutes, &Serial_TxPacket[2]);  // 分钟 (1字节)
    Serial_Byte(lockedData.seconds, &Serial_TxPacket[3]);  // 秒 (1字节)
        // 浮点型数据装配 (3个float, 共12字节)
    Serial_float(lockedData.distance, &Serial_TxPacket[4]);  // 距离 (4字节)
    Serial_float(lockedData.averageSpeed, &Serial_TxPacket[8]);  // 平均速度 (4字节)
    Serial_float(lockedData.maxAcceleration, &Serial_TxPacket[12]);  // 最大加速度 (4字节)
    
    
    // 计算校验和并自动装配
    Serial_check();
    
    // 发送数据包
    Serial_SendArray(Serial_TxPacket, TX_Data_Len);
}

// 按键处理函数(主循环中调用)
void Key_Process(void)
{

    

    // 处理目标距离达成逻辑 - LED1
    if (Hall_IsTargetDistanceReached() && !g_distanceTargetReached )
    {
        g_distanceTargetReached = 1;  // 设置距离目标已达成标志
        
        // 距离目标达成后LED1闪烁3次
        LED_Blink(LED1_PIN, 7);
        

    }
    
    // 处理目标速度达成逻辑 - LED2
    if (Hall_IsTargetSpeedReached())
    {
        if (!g_speedTargetReached)  // 只在首次达到或从未达到状态转为达到状态时闪烁
        {
            g_speedTargetReached = 1;  // 设置速度目标已达成标志
            
            // 速度目标达成后LED2闪烁2次
            LED_Blink(LED2_PIN, 7);
        }
    }
    else
    {
        // 速度低于目标时重置标志，这样下次达到时会再次闪烁
        g_speedTargetReached = 0;
    }
    

    // 修改为锁存状态下持续发送蓝牙数据
    if (g_displayMode == DISPLAY_LOCKED) {
        // 获取当前系统时间
        //extern volatile uint32_t systemTimeMs;
        //uint32_t currentTime = systemTimeMs;
        
        // 每隔BLUETOOTH_SEND_INTERVAL发送一次数据
        //if (currentTime - g_bluetoothLastSendTime >= BLUETOOTH_SEND_INTERVAL) {
            // 发送锁存数据到蓝牙
            Serial_SendLockedPacket();
            
            // 更新最后发送时间
            //g_bluetoothLastSendTime = currentTime;
            
            // 可选：发送指示 - LED闪烁提示正在发送
            //LED_Toggle(LED1_PIN);
        //}
    }

}
