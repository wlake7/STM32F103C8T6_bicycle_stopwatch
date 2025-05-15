#include "i2c.h"
#include "stm32f10x.h"
#include "stm32f10x_i2c.h"
#include "stm32f10x_rcc.h"
#include "stm32f10x_gpio.h"

/**
  * @brief  I2C 配置初始化
  * @param  无
  * @retval 无
  */
void I2C_Configuration(void)
{
    GPIO_InitTypeDef GPIO_InitStructure;
    I2C_InitTypeDef I2C_InitStructure;
    
    // 使能I2C和GPIO时钟
    RCC_APB1PeriphClockCmd(I2C_CLK, ENABLE);
    RCC_APB2PeriphClockCmd(I2C_GPIO_CLK | RCC_APB2Periph_AFIO, ENABLE);
    
    // 配置I2C引脚
    GPIO_InitStructure.GPIO_Pin = I2C_SCL_PIN | I2C_SDA_PIN;
    GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;
    GPIO_InitStructure.GPIO_Mode = GPIO_Mode_AF_OD;  // 复用开漏输出
    GPIO_Init(I2C_SCL_PORT, &GPIO_InitStructure);
    
    // 复位I2C
    I2C_DeInit(I2C_PORT);
    
    // 配置I2C
    I2C_InitStructure.I2C_Mode = I2C_Mode_I2C;
    I2C_InitStructure.I2C_DutyCycle = I2C_DutyCycle_2;
    I2C_InitStructure.I2C_OwnAddress1 = 0x00;  // 自身地址(作为主机，可以设置为0)
    I2C_InitStructure.I2C_Ack = I2C_Ack_Enable;
    I2C_InitStructure.I2C_AcknowledgedAddress = I2C_AcknowledgedAddress_7bit;
    I2C_InitStructure.I2C_ClockSpeed = I2C_SPEED;
    
    // 初始化I2C
    I2C_Init(I2C_PORT, &I2C_InitStructure);
    
    // 使能I2C
    I2C_Cmd(I2C_PORT, ENABLE);
}

/**
  * @brief  I2C 复位
  * @param  无
  * @retval 无
  */
void I2C_Reset(void)
{
    I2C_DeInit(I2C_PORT);
    I2C_Configuration();
}

/**
  * @brief  I2C 产生起始信号
  * @param  无
  * @retval I2C_Status: 操作状态
  */
I2C_Status I2C_Start(void)
{
    uint32_t timeout = I2C_TIMEOUT;
    
    // 产生起始条件
    I2C_GenerateSTART(I2C_PORT, ENABLE);
    
    // 等待起始条件产生完成
    while (!I2C_CheckEvent(I2C_PORT, I2C_EVENT_MASTER_MODE_SELECT))
    {
        if ((timeout--) == 0)
            return I2C_TIMEOUT_ERROR;
    }
    
    return I2C_OK;
}

/**
  * @brief  I2C 产生停止信号
  * @param  无
  * @retval 无
  */
void I2C_Stop(void)
{
    // 产生停止条件
    I2C_GenerateSTOP(I2C_PORT, ENABLE);
}

/**
  * @brief  I2C 发送一个字节
  * @param  byte: 要发送的字节
  * @retval I2C_Status: 操作状态
  */
I2C_Status I2C_SendByte(uint8_t byte)
{
    uint32_t timeout = I2C_TIMEOUT;
    
    // 发送数据
    I2C_SendData(I2C_PORT, byte);
    
    // 等待数据发送完成
    while (!I2C_CheckEvent(I2C_PORT, I2C_EVENT_MASTER_BYTE_TRANSMITTED))
    {
        // 检查NACK错误
        if (I2C_GetFlagStatus(I2C_PORT, I2C_FLAG_AF))
        {
            I2C_ClearFlag(I2C_PORT, I2C_FLAG_AF);
            return I2C_NACK_ERROR;
        }
        
        // 检查超时
        if ((timeout--) == 0)
            return I2C_TIMEOUT_ERROR;
    }
    
    return I2C_OK;
}

/**
  * @brief  I2C 接收一个字节
  * @param  byte: 接收数据的指针
  * @param  ack: 是否发送ACK (1:ACK, 0:NACK)
  * @retval I2C_Status: 操作状态
  */
I2C_Status I2C_ReceiveByte(uint8_t* byte, uint8_t ack)
{
    uint32_t timeout = I2C_TIMEOUT;
    
    // 设置ACK
    if (ack)
        I2C_AcknowledgeConfig(I2C_PORT, ENABLE);
    else
        I2C_AcknowledgeConfig(I2C_PORT, DISABLE);
    
    // 等待接收数据
    while (!I2C_CheckEvent(I2C_PORT, I2C_EVENT_MASTER_BYTE_RECEIVED))
    {
        if ((timeout--) == 0)
            return I2C_TIMEOUT_ERROR;
    }
    
    // 读取数据
    *byte = I2C_ReceiveData(I2C_PORT);
    
    return I2C_OK;
}
/*
============================【进阶】==============================================
============================【进阶】============================================
*/
/**
  * @brief  写一个字节数据到I2C设备的寄存器
  * @param  slave_addr: 从机地址(7位)
  * @param  reg_addr: 寄存器地址
  * @param  data: 要写入的数据
  * @retval I2C_Status: 操作状态
  */
I2C_Status I2C_WriteByte(uint8_t slave_addr, uint8_t reg_addr, uint8_t data)
{
    I2C_Status status;
    
    // 产生起始条件
    status = I2C_Start();
    if (status != I2C_OK)
        return status;
    
    // 发送从机地址(写)
    I2C_SendData(I2C_PORT, (slave_addr << 1) | 0x00); // 左移1位后使用或运算设置最低位为0(写模式)

    // 等待地址发送完成
    uint32_t timeout = I2C_TIMEOUT;
    while (!I2C_CheckEvent(I2C_PORT, I2C_EVENT_MASTER_TRANSMITTER_MODE_SELECTED))
    {
        // 检查NACK错误
        if (I2C_GetFlagStatus(I2C_PORT, I2C_FLAG_AF))
        {
            I2C_ClearFlag(I2C_PORT, I2C_FLAG_AF);
            I2C_Stop();
            return I2C_NACK_ERROR;
        }
        
        if ((timeout--) == 0)
        {
            I2C_Stop();
            return I2C_TIMEOUT_ERROR;
        }
    }
    
    // 发送寄存器地址
    status = I2C_SendByte(reg_addr);
    if (status != I2C_OK)
    {
        I2C_Stop();
        return status;
    }
    
    // 发送数据
    status = I2C_SendByte(data);
    
    // 产生停止条件
    I2C_Stop();
    
    return status;
}

/**
  * @brief  从I2C设备的寄存器读一个字节数据
  * @param  slave_addr: 从机地址(7位)
  * @param  reg_addr: 寄存器地址
  * @param  data: 读取数据的指针
  * @retval I2C_Status: 操作状态
  */
I2C_Status I2C_ReadByte(uint8_t slave_addr, uint8_t reg_addr, uint8_t* data)
{
    I2C_Status status;
    uint32_t timeout = I2C_TIMEOUT;
    
    // 产生起始条件
    status = I2C_Start();
    if (status != I2C_OK)
        return status;
    
    // 发送从机地址(写), 用于设置寄存器地址
    I2C_SendData(I2C_PORT, (slave_addr << 1) | 0x00); // 左移1位后使用或运算设置最低位为0(写模式)
    
    // 等待地址发送完成
    timeout = I2C_TIMEOUT;
    while (!I2C_CheckEvent(I2C_PORT, I2C_EVENT_MASTER_TRANSMITTER_MODE_SELECTED))
    {
        if (I2C_GetFlagStatus(I2C_PORT, I2C_FLAG_AF))
        {
            I2C_ClearFlag(I2C_PORT, I2C_FLAG_AF);
            I2C_Stop();
            return I2C_NACK_ERROR;
        }
        
        if ((timeout--) == 0)
        {
            I2C_Stop();
            return I2C_TIMEOUT_ERROR;
        }
    }
    
    // 发送寄存器地址
    status = I2C_SendByte(reg_addr);
    if (status != I2C_OK)
    {
        I2C_Stop();
        return status;
    }
    
    // 产生重新开始条件
    status = I2C_Start();
    if (status != I2C_OK)
    {
        I2C_Stop();
        return status;
    }
    
    // 发送从机地址(读)
    I2C_SendData(I2C_PORT, (slave_addr << 1) | 0x01);   // 左移1位后使用或运算设置最低位为1(读模式)
    
    // 等待地址发送完成
    timeout = I2C_TIMEOUT;
    while (!I2C_CheckEvent(I2C_PORT, I2C_EVENT_MASTER_RECEIVER_MODE_SELECTED))
    {
        if (I2C_GetFlagStatus(I2C_PORT, I2C_FLAG_AF))
        {
            I2C_ClearFlag(I2C_PORT, I2C_FLAG_AF);
            I2C_Stop();
            return I2C_NACK_ERROR;
        }
        
        if ((timeout--) == 0)
        {
            I2C_Stop();
            return I2C_TIMEOUT_ERROR;
        }
    }
    
    // 接收数据(发送NACK)
    status = I2C_ReceiveByte(data, 0);
    
    // 产生停止条件
    I2C_Stop();
    
    return status;
}

/**
  * @brief  写多个字节数据到I2C设备
  * @param  slave_addr: 从机地址(7位)
  * @param  reg_addr: 寄存器起始地址
  * @param  data: 要写入的数据数组
  * @param  len: 数据长度
  * @retval I2C_Status: 操作状态
  */
I2C_Status I2C_WriteMultiByte(uint8_t slave_addr, uint8_t reg_addr, uint8_t* data, uint8_t len)
{
    I2C_Status status;
    uint8_t i;
    
    // 产生起始条件
    status = I2C_Start();
    if (status != I2C_OK)
        return status;
    
    // 发送从机地址(写)
    I2C_SendData(I2C_PORT, (slave_addr << 1) | 0x00); // 左移1位后使用或运算设置最低位为0(写模式)
    
    // 等待地址发送完成
    uint32_t timeout = I2C_TIMEOUT;
    while (!I2C_CheckEvent(I2C_PORT, I2C_EVENT_MASTER_TRANSMITTER_MODE_SELECTED))
    {
        if (I2C_GetFlagStatus(I2C_PORT, I2C_FLAG_AF))
        {
            I2C_ClearFlag(I2C_PORT, I2C_FLAG_AF);
            I2C_Stop();
            return I2C_NACK_ERROR;
        }
        
        if ((timeout--) == 0)
        {
            I2C_Stop();
            return I2C_TIMEOUT_ERROR;
        }
    }
    
    // 发送寄存器地址
    status = I2C_SendByte(reg_addr);
    if (status != I2C_OK)
    {
        I2C_Stop();
        return status;
    }
    
    // 循环发送数据
    for (i = 0; i < len; i++)
    {
        status = I2C_SendByte(data[i]);
        if (status != I2C_OK)
        {
            I2C_Stop();
            return status;
        }
    }
    
    // 产生停止条件
    I2C_Stop();
    
    return I2C_OK;
}

/**
  * @brief  从I2C设备读多个字节数据
  * @param  slave_addr: 从机地址(7位)
  * @param  reg_addr: 寄存器起始地址
  * @param  data: 读取数据的指针
  * @param  len: 数据长度
  * @retval I2C_Status: 操作状态
  */
I2C_Status I2C_ReadMultiByte(uint8_t slave_addr, uint8_t reg_addr, uint8_t* data, uint8_t len)
{
    I2C_Status status;
    uint8_t i;
    uint32_t timeout = I2C_TIMEOUT;
    
    // 产生起始条件
    status = I2C_Start();
    if (status != I2C_OK)
        return status;
    
    // 发送从机地址(写), 用于设置寄存器地址
    I2C_SendData(I2C_PORT, (slave_addr << 1) | 0x00); // 左移1位后使用或运算设置最低位为0(写模式)
    
    // 等待地址发送完成
    timeout = I2C_TIMEOUT;
    while (!I2C_CheckEvent(I2C_PORT, I2C_EVENT_MASTER_TRANSMITTER_MODE_SELECTED))
    {
        if (I2C_GetFlagStatus(I2C_PORT, I2C_FLAG_AF))
        {
            I2C_ClearFlag(I2C_PORT, I2C_FLAG_AF);
            I2C_Stop();
            return I2C_NACK_ERROR;
        }
        
        if ((timeout--) == 0)
        {
            I2C_Stop();
            return I2C_TIMEOUT_ERROR;
        }
    }
    
    // 发送寄存器地址
    status = I2C_SendByte(reg_addr);
    if (status != I2C_OK)
    {
        I2C_Stop();
        return status;
    }
    
    // 产生重新开始条件
    status = I2C_Start();
    if (status != I2C_OK)
    {
        I2C_Stop();
        return status;
    }
    
    // 发送从机地址(读)
    I2C_SendData(I2C_PORT, (slave_addr << 1) | 0x01); // 左移1位后使用或运算设置最低位为1(读模式)
    
    // 等待地址发送完成
    timeout = I2C_TIMEOUT;
    while (!I2C_CheckEvent(I2C_PORT, I2C_EVENT_MASTER_RECEIVER_MODE_SELECTED))
    {
        if (I2C_GetFlagStatus(I2C_PORT, I2C_FLAG_AF))
        {
            I2C_ClearFlag(I2C_PORT, I2C_FLAG_AF);
            I2C_Stop();
            return I2C_NACK_ERROR;
        }
        
        if ((timeout--) == 0)
        {
            I2C_Stop();
            return I2C_TIMEOUT_ERROR;
        }
    }
    
    // 循环接收数据
    for (i = 0; i < len; i++)
    {
        // 最后一个字节发送NACK，其他字节发送ACK
        if (i == len - 1)
            status = I2C_ReceiveByte(&data[i], 0);
        else
            status = I2C_ReceiveByte(&data[i], 1);
        
        if (status != I2C_OK)
        {
            I2C_Stop();
            return status;
        }
    }
    
    // 产生停止条件
    I2C_Stop();
    
    return I2C_OK;
}
