#ifndef __USART_H
#define __USART_H

#include "stm32f10x.h"
#include <stdio.h>
/*
		字符型（char）：		字节数：1字节（8位）
		短整型（short int）：	字节数：2字节（16位）
		整型（int）：			字节数：4字节（32位）
		浮点型（float）：		字节数：4字节（32位）
		双精度浮点型（double）：字节数：8字节（64位）
		布尔型（_Bool）：		字节数：1字节
*/

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

// 数据包相关声明
#define TX_Data_Len 18  // 定义发送蓝牙数据包长度：包头(1)+数据(15)+校验和(1)+包尾(1)=18字节
extern uint8_t Serial_TxPacket[TX_Data_Len];  // 定义缓存区的数组

/* 函数声明 */
void Serial_Init(void);                         // 串口初始化
void Serial_SendByte(uint8_t Byte);             // 发送单个字节
uint8_t Serial_SendByteWithTimeout(uint8_t Byte);  // 发送单个字节(带超时)
void Serial_SendString(char *String);           // 发送字符串
void Serial_SendArray(uint8_t *Array, uint16_t Length); // 发送数组
uint16_t Serial_SendArrayWithTimeout(uint8_t *Array, uint16_t Length); // 发送数组(带超时)
int8_t Serial_ReceiveByte(void);                // 接收单个字节
uint8_t Serial_GetRxFlag(void);                 // 获取接收标志位
void Serial_ClearRxFlag(void);                  // 清除接收标志位
uint8_t* Serial_GetRxBuffer(void);              // 获取接收缓冲区

// 数据包装配函数
void Serial_Int(int num, uint8_t *byte);      // 整形装配
void Serial_float(float num, uint8_t *byte);  // 浮点型装配
void Serial_short(short s, uint8_t *byte);    // short型装配
void Serial_Byte(uint8_t data, uint8_t *byte); // byte型装配
void Serial_check(void);                      // 计算校验和，并自动装配

/* 协议相关函数 */
void Serial_SendFloatFrame(float *data, uint8_t channels);  // 发送浮点数帧
void Serial_SendIMUData(float Pitch,float Roll,float Yaw); // 发送IMU数据
void Serial_SendIMUG(int16_t MagX, int16_t MagY, int16_t MagZ); // 发送磁力计数据
void Serial_Sendms2(float pitch,float roll,float yaw,float xy_ms2); // 发送水平加速度数据

// 修改蓝牙锁存数据发送函数声明，增加返回值
uint8_t Serial_SendLockedDataPacket(float distance, float avgSpeed, float maxAccel, uint8_t hours, uint8_t minutes, uint8_t seconds);

/* 重定向printf函数所需 */
int fputc(int ch, FILE *f);


//=============上位机串口使用方法====================
//Serial_SendIMUData(float Pitch,float Roll,float Yaw)在该函数中调用

#endif
