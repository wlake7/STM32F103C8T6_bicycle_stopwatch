#ifndef __I2C_H
#define __I2C_H

#include "stm32f10x.h"

// I2C端口定义
#define I2C_PORT        I2C2
#define I2C_CLK         RCC_APB1Periph_I2C2
#define I2C_GPIO_CLK    RCC_APB2Periph_GPIOB
#define I2C_SCL_PORT    GPIOB
#define I2C_SDA_PORT    GPIOB
#define I2C_SCL_PIN     GPIO_Pin_10
#define I2C_SDA_PIN     GPIO_Pin_11

// I2C速度设置
#define I2C_SPEED       400000  // 400KHz

// I2C超时定义
#define I2C_TIMEOUT     10000   // 超时时间

// I2C操作结果
typedef enum {
    I2C_OK = 0,
    I2C_TIMEOUT_ERROR,
    I2C_BUS_ERROR,
    I2C_NACK_ERROR
} I2C_Status;

// 函数声明
void I2C_Configuration(void);
void I2C_Reset(void);
I2C_Status I2C_Start(void);
void I2C_Stop(void);
I2C_Status I2C_SendByte(uint8_t byte);
I2C_Status I2C_ReceiveByte(uint8_t* byte, uint8_t ack);
I2C_Status I2C_WriteByte(uint8_t slave_addr, uint8_t reg_addr, uint8_t data);
I2C_Status I2C_ReadByte(uint8_t slave_addr, uint8_t reg_addr, uint8_t* data);
I2C_Status I2C_WriteMultiByte(uint8_t slave_addr, uint8_t reg_addr, uint8_t* data, uint8_t len);
I2C_Status I2C_ReadMultiByte(uint8_t slave_addr, uint8_t reg_addr, uint8_t* data, uint8_t len);

#endif /* __I2C_H */
