#ifndef __DH11_H
#define __DH11_H

#include "stm32f10x.h"
#include "sys_delay.h"

// DH11数据结构体
typedef struct
{
    uint8_t humidity_int;     // 湿度整数部分
    uint8_t humidity_dec;     // 湿度小数部分
    uint8_t temperature_int;  // 温度整数部分
    uint8_t temperature_dec;  // 温度小数部分
    uint8_t check_sum;        // 校验和
    uint8_t error;            // 错误标志
} DHT11_Data_TypeDef;

// 引脚定义
#define DH11_PORT              GPIOB
#define DH11_PIN               GPIO_Pin_14
#define DH11_RCC               RCC_APB2Periph_GPIOB

// 引脚操作宏定义
#define DH11_DATA_OUT_HIGH()   GPIO_SetBits(DH11_PORT, DH11_PIN)
#define DH11_DATA_OUT_LOW()    GPIO_ResetBits(DH11_PORT, DH11_PIN)
#define DH11_DATA_IN()         GPIO_ReadInputDataBit(DH11_PORT, DH11_PIN)

// 错误码定义
#define DH11_OK                0
#define DH11_ERROR_CHECKSUM    1
#define DH11_ERROR_TIMEOUT     2


// 函数声明
void DH11_Init(void);                              // 初始化DH11
uint8_t DH11_Read_Data(DHT11_Data_TypeDef* data);  // 读取DH11数据
uint8_t DH11_Read_Byte(void);                      // 读取一个字节
void DH11_Mode_IPU(void);                          // 配置引脚为上拉输入模式
void DH11_Mode_Out_PP(void);                       // 配置引脚为推挽输出模式
uint8_t DH11_IsReady(void);                        // 检查DH11是否就绪
uint8_t DH11_Check_Sum(DHT11_Data_TypeDef* data);  // 校验和检查
uint8_t DH11_Read_Data_Periodic(DHT11_Data_TypeDef* data, uint16_t interval); // 周期性读取数据
//readme
// 1. DH11_Init()：初始化DH11传感器所使用的IO引脚，设置为推挽输出模式，并将引脚初始状态设置为高电平。
// 2. 创建DHT11_Data_TypeDef类型的变量来存储读取的数据。
// 3. DH11_Read_Data_Periodic(DHT11_Data_TypeDef* data, uint16_t interval)：读取DH11传感器的数据，返回0表示成功，其他值表示错误。
// 4. 根据返回值进行错误处理，检查data.error确定读取结果。==0: 成功读取, 1: 校验和错误, 2: 超时错误, 3: 本次未读取数据。==


#endif /* __DH11_H */
