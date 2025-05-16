#ifndef __USART_H
#define __USART_H

#include "stm32f10x.h"
#include <stdio.h>
//串口调试工具
/* 串口配置宏定义，方便移植和修改 */
#define USART_PORT              USART1              // 使用的串口
#define USART_GPIO              GPIOA               // 串口GPIO组
#define USART_TX_PIN            GPIO_Pin_9          // 串口TX引脚
#define USART_RX_PIN            GPIO_Pin_10         // 串口RX引脚
#define USART_BAUDRATE          9600                // 波特率
#define USART_RCC_APB_PERIPH    RCC_APB2Periph_USART1  // 串口时钟
#define USART_GPIO_RCC_PERIPH   RCC_APB2Periph_GPIOA   // GPIO时钟

/* 串口接收缓冲区大小 */
#define USART_RX_BUFFER_SIZE    64

/* 协议相关宏定义 */
#define CH_COUNT                3    // 通道数量：AX, AY, AZ, GX, GY, GZ
#define FRAME_TAIL_SIZE         4    // 帧尾大小

/* 函数声明 */
void Serial_Init(void);                         // 串口初始化
void Serial_SendByte(uint8_t Byte);             // 发送单个字节
void Serial_SendString(char *String);           // 发送字符串
void Serial_SendArray(uint8_t *Array, uint16_t Length); // 发送数组
int8_t Serial_ReceiveByte(void);                // 接收单个字节
uint8_t Serial_GetRxFlag(void);                 // 获取接收标志位
void Serial_ClearRxFlag(void);                  // 清除接收标志位
uint8_t* Serial_GetRxBuffer(void);              // 获取接收缓冲区

/* 协议相关函数 */
void Serial_SendFloatFrame(float *data, uint8_t channels);  // 发送浮点数帧
void Serial_SendIMUData(float Pitch,float Roll,float Yaw); // 发送IMU数据
void Serial_SendIMUG(int16_t MagX, int16_t MagY, int16_t MagZ); // 发送磁力计数据
void Serial_Sendms2(float pitch,float roll,float yaw,float xy_ms2); // 发送水平加速度数据

// 添加蓝牙锁存数据发送函数声明
void Serial_SendLockedDataPacket(float distance, float avgSpeed, float maxAccel, uint8_t hours, uint8_t minutes, uint8_t seconds);

/* 重定向printf函数所需 */
int fputc(int ch, FILE *f);


//=============上位机串口使用方法====================
//Serial_SendIMUData(float Pitch,float Roll,float Yaw)在该函数中调用

#endif
