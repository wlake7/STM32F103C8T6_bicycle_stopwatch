#ifndef __KEY_H
#define __KEY_H

#include "stm32f10x.h"
#include "sys_delay.h"
#include "OLED.h"
#include "Hall.h"

// 按键引脚定义
#define KEY_UP_PIN      GPIO_Pin_0   // PB0 - 上/增加按钮
#define KEY_DOWN_PIN    GPIO_Pin_1   // PB1 - 下/减少按钮
#define KEY_SET_PIN     GPIO_Pin_8   // PA8 - 设置/确认按钮 (修改为PA8)
#define KEY_LOCK_PIN    GPIO_Pin_7   // PA7 - 锁存按钮

// LED和蜂鸣器引脚定义
#define LED1_PIN        GPIO_Pin_3   // PA3 - LED1
#define LED2_PIN        GPIO_Pin_4   // PA4 - LED2
#define BUZZER_PIN      GPIO_Pin_5   // PA5 - 蜂鸣器

#define KEY_PORT        GPIOB        // PB端口上的按键
#define KEY_SET_PORT    GPIOA        // PA8设置按键 (新增)
#define KEY_LOCK_PORT   GPIOA        // PA7锁存按键
#define LED_BUZZER_PORT GPIOA        // LED和蜂鸣器端口

// 设置状态枚举
typedef enum {
    SET_STATE_SPEED,      // 正在设置目标速度
    SET_STATE_DISTANCE,   // 正在设置目标距离
    SET_STATE_DONE        // 设置完成
} SetState_t;

// 显示模式枚举
typedef enum {
    DISPLAY_REALTIME,     // 实时数据显示
    DISPLAY_LOCKED        // 锁定的历史数据显示
} DisplayMode_t;

// 全局变量声明
extern SetState_t g_setState;
extern float g_targetSpeed;
extern float g_targetDistance;
extern DisplayMode_t g_displayMode;
extern uint8_t g_distanceTargetReached; // 距离目标达成标志
extern uint8_t g_speedTargetReached;    // 速度目标达成标志

// 函数声明
void Key_Init(void);                      // 初始化所有按键、LED和蜂鸣器
void Key_EXTI_Config(void);               // 配置按键外部中断
void Key_DisplaySettings(void);           // 显示当前设置界面
void Key_SettingsMenu(void);              // 设置菜单主函数（阻塞式）
void LED_Toggle(uint16_t LED_Pin);        // LED翻转状态
void LED_Control(uint16_t LED_Pin, uint8_t state); // LED控制
void Buzzer_ON(void);                     // 打开蜂鸣器
void Buzzer_OFF(void);                    // 关闭蜂鸣器
uint8_t Key_IsPressed(GPIO_TypeDef* GPIOx, uint16_t GPIO_Pin); // 检查按键是否被按下
void Key_Process(void);                   // 按键处理函数(主循环中调用)
//显示函数显示且获取数据
#endif /* __KEY_H */
