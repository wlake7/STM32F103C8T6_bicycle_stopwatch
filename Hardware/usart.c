#include "stm32f10x.h"                  // Device header
#include "usart.h"

/* 串口接收缓冲区和标志位 */
static uint8_t Serial_RxBuffer[USART_RX_BUFFER_SIZE];  // 接收缓冲区
static uint16_t Serial_RxIndex = 0;                   // 接收缓冲区当前索引
static uint8_t Serial_RxFlag = 0;                     // 接收完成标志位

/**
  * 函    数：串口初始化
  * 参    数：无
  * 返 回 值：无
  * 说    明：根据宏定义配置串口参数，方便移植
  */
void Serial_Init(void)
{
    /* 开启时钟 */
    RCC_APB2PeriphClockCmd(USART_RCC_APB_PERIPH, ENABLE);  // 开启USART的时钟
    RCC_APB2PeriphClockCmd(USART_GPIO_RCC_PERIPH, ENABLE); // 开启GPIO的时钟
    
    /* GPIO初始化 - TX引脚配置为复用推挽输出 */
    GPIO_InitTypeDef GPIO_InitStructure;
    GPIO_InitStructure.GPIO_Mode = GPIO_Mode_AF_PP;
    GPIO_InitStructure.GPIO_Pin = USART_TX_PIN;
    GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;
    GPIO_Init(USART_GPIO, &GPIO_InitStructure);
    
    /* GPIO初始化 - RX引脚配置为上拉输入 */
    GPIO_InitStructure.GPIO_Mode = GPIO_Mode_IPU;
    GPIO_InitStructure.GPIO_Pin = USART_RX_PIN;
    GPIO_Init(USART_GPIO, &GPIO_InitStructure);
    
    /* USART初始化 */
    USART_InitTypeDef USART_InitStructure;
    USART_InitStructure.USART_BaudRate = USART_BAUDRATE;            // 波特率
    USART_InitStructure.USART_HardwareFlowControl = USART_HardwareFlowControl_None;  // 无硬件流控制
    USART_InitStructure.USART_Mode = USART_Mode_Tx | USART_Mode_Rx; // 收发模式
    USART_InitStructure.USART_Parity = USART_Parity_No;             // 无校验位
    USART_InitStructure.USART_StopBits = USART_StopBits_1;          // 1位停止位
    USART_InitStructure.USART_WordLength = USART_WordLength_8b;     // 8位数据位
    USART_Init(USART_PORT, &USART_InitStructure);
    
    /* 中断配置 */
    USART_ITConfig(USART_PORT, USART_IT_RXNE, ENABLE);              // 开启接收中断
    
    /* NVIC中断优先级配置 */
    NVIC_InitTypeDef NVIC_InitStructure;
    NVIC_InitStructure.NVIC_IRQChannel = USART1_IRQn;               // 中断通道
    NVIC_InitStructure.NVIC_IRQChannelCmd = ENABLE;                 // 使能中断通道
    NVIC_InitStructure.NVIC_IRQChannelPreemptionPriority = 1;       // 抢占优先级
    NVIC_InitStructure.NVIC_IRQChannelSubPriority = 1;              // 子优先级
    NVIC_Init(&NVIC_InitStructure);
    
    /* USART使能 */
    USART_Cmd(USART_PORT, ENABLE);
}

/**
  * 函    数：串口发送一个字节
  * 参    数：Byte 要发送的一个字节
  * 返 回 值：无
  */
void Serial_SendByte(uint8_t Byte)
{
    USART_SendData(USART_PORT, Byte);   // 将字节数据写入数据寄存器
    while (USART_GetFlagStatus(USART_PORT, USART_FLAG_TXE) == RESET); // 等待发送完成
}

/**
  * 函    数：串口发送字符串
  * 参    数：String 要发送的字符串
  * 返 回 值：无
  */
void Serial_SendString(char *String)
{
    uint16_t i;
    for (i = 0; String[i] != '\0'; i++)
    {
        Serial_SendByte(String[i]);
    }
}

/**
  * 函    数：串口发送数组
  * 参    数：Array 要发送的数组，Length 数组长度
  * 返 回 值：无
  */
void Serial_SendArray(uint8_t *Array, uint16_t Length)
{
    uint16_t i;
    for (i = 0; i < Length; i++)
    {
        Serial_SendByte(Array[i]);
    }
}
//=====================
//==============================
/**
  * 函    数：发送浮点数帧
  * 参    数：data 浮点数据数组，channels 通道数量
  * 返 回 值：无
  * 说    明：按照小端浮点数组协议发送数据
  */
void Serial_SendFloatFrame(float *data, uint8_t channels)
{   
    // 发送帧尾
      uint8_t tail[4] = {0x00, 0x00, 0x80, 0x7F};
    // 发送浮点数据
    Serial_SendArray((uint8_t *)data, sizeof(float) * channels);
  
    Serial_SendArray(tail, 4);
}

/**
  * 函    数：发送IMU数据
  * 参    数：AX, AY, AZ, GX, GY, GZ - 六轴传感器数据
  * 返 回 值：无
  * 说    明：将int16_t类型的IMU数据转换为float并发送
  */
void Serial_SendIMUData(float Pitch,float Roll,float Yaw)
{
    float data[CH_COUNT];
    
    // 将int16_t数据转换为float (可根据需要调整转换因子)
    //data[0] = (float)AX;
    //data[1] = (float)AY;
    //data[2] = (float)AZ;
    //data[3] = (float)GX;
    //data[5] = (float)GZ;
    data[0] = Pitch;  // 俯仰角
    data[1] = Roll;   // 横滚角
    data[2] = Yaw;    // 偏航角
    
    // 发送数据帧
    Serial_SendFloatFrame(data, CH_COUNT);
}

    /**
      * 函    数：发送磁力计数据
      * 参    数：MagX, MagY, MagZ - int16_t类型的磁力计数据
      * 返 回 值：无
      * 说    明：将int16_t类型的磁力计数据转换为float并发送
      */
     void Serial_SendIMUG(int16_t MagX, int16_t MagY, int16_t MagZ)
     {
         // Define the number of channels for magnetometer data
         // If CH_COUNT is defined globally and should be used, ensure it's set to 3
         // Otherwise, using a local constant or literal 3 is safer here.
         float data[3]; // 3 channels for magnetometer data
 
         // Convert int16_t magnetometer data to float
         data[0] = (float)MagX;
         data[1] = (float)MagY;
         data[2] = (float)MagZ;
 
         // Send data frame using the existing float frame function
         Serial_SendFloatFrame(data, 3);
     }
void Serial_Sendms2(float pitch,float roll,float yaw,float xy_ms2)
{
  float data[4]; 
  // 将int16_t数据转换为float (可根据需要调整转换因子)
  data[0] = pitch;  // 俯仰角
  data[1] = roll;   // 横滚角
  data[2] = yaw;    // 偏航角
  data[3] = xy_ms2; // 水平加速度
  // 发送数据帧
  Serial_SendFloatFrame(data, 4);
}

/**
  * 函    数：发送锁存数据包
  * 参    数：distance - 锁存的距离(km)
  * 参    数：avgSpeed - 锁存的平均速度(km/h)
  * 参    数：maxAccel - 锁存的最大加速度(m/s^2)
  * 参    数：hours - 锁存的小时
  * 参    数：minutes - 锁存的分钟
  * 参    数：seconds - 锁存的秒
  * 返 回 值：无
  * 数据格式：包头(0xA5) + 数据 + 校验和 + 包尾(0x5A)
  */
void Serial_SendLockedDataPacket(float distance, float avgSpeed, float maxAccel, uint8_t hours, uint8_t minutes, uint8_t seconds)
{
    uint8_t dataBuffer[15]; // 数据缓冲区: 3个float(12字节) + 3个uint8_t(3字节) = 15字节
    uint8_t packet[18];     // 完整数据包: 包头(1) + 数据(15) + 校验和(1) + 包尾(1) = 18字节
    uint8_t checksum = 0;   // 校验和
    uint8_t i;
    
    // 1. 构造数据部分 (将浮点数和时间值复制到数据缓冲区)
    memcpy(&dataBuffer[0], &distance, sizeof(float));  // 距离 (4字节)
    memcpy(&dataBuffer[4], &avgSpeed, sizeof(float));  // 平均速度 (4字节)
    memcpy(&dataBuffer[8], &maxAccel, sizeof(float));  // 最大加速度 (4字节)
    dataBuffer[12] = hours;                            // 小时 (1字节)
    dataBuffer[13] = minutes;                          // 分钟 (1字节)
    dataBuffer[14] = seconds;                          // 秒 (1字节)
    
    // 2. 计算校验和 (所有数据字节之和的低8位)
    for (i = 0; i < 15; i++) {
        checksum += dataBuffer[i];
    }
    
    // 3. 组装完整数据包
    packet[0] = 0xA5;  // 包头
    
    // 复制数据部分
    for (i = 0; i < 15; i++) {
        packet[1 + i] = dataBuffer[i];
    }
    
    packet[16] = checksum;  // 校验和
    packet[17] = 0x5A;      // 包尾
    
    // 4. 发送数据包
    Serial_SendArray(packet, 18);
}
//=====================
//==============================
/**
  * 函    数：串口接收一个字节
  * 参    数：无
  * 返 回 值：接收到的一个字节
  * 说    明：阻塞式接收，直到接收到数据
  */
int8_t Serial_ReceiveByte(void)
{
    while (USART_GetFlagStatus(USART_PORT, USART_FLAG_RXNE) == RESET); // 等待接收完成
    return USART_ReceiveData(USART_PORT);  // 读取接收到的数据
}

/**
  * 函    数：获取接收标志位
  * 参    数：无
  * 返 回 值：接收标志位
  */
uint8_t Serial_GetRxFlag(void)
{
    return Serial_RxFlag;
}

/**
  * 函    数：清除接收标志位
  * 参    数：无
  * 返 回 值：无
  */
void Serial_ClearRxFlag(void)
{
    Serial_RxFlag = 0;
    Serial_RxIndex = 0;
}

/**
  * 函    数：获取接收缓冲区指针
  * 参    数：无
  * 返 回 值：接收缓冲区指针
  */
uint8_t* Serial_GetRxBuffer(void)
{
    return Serial_RxBuffer;
}

/**
  * 函    数：USART1中断处理函数
  * 参    数：无
  * 返 回 值：无
  * 说    明：处理接收到的数据并保存到缓冲区
  */
 /*
void USART1_IRQHandler(void)
{
    if (USART_GetITStatus(USART_PORT, USART_IT_RXNE) == SET)
    {
        uint8_t RxData = USART_ReceiveData(USART_PORT);  // 读取接收到的数据
        
        // 将数据存入接收缓冲区
        if (Serial_RxIndex < USART_RX_BUFFER_SIZE)
        {
            Serial_RxBuffer[Serial_RxIndex++] = RxData;
            
            // 如果接收到回车或换行符，则表示一帧数据接收完毕
            if (RxData == '\r' || RxData == '\n')
            {
                Serial_RxFlag = 1;  // 设置接收完成标志位
            }
        }
        else
        {
            // 缓冲区溢出，重置缓冲区索引
            Serial_RxIndex = 0;
        }
        
        USART_ClearITPendingBit(USART_PORT, USART_IT_RXNE);  // 清除中断标志位
    }
}
*/
/**
  * 函    数：重定向printf函数到串口
  * 参    数：ch 字符，f 文件指针
  * 返 回 值：发送的字符
  * 说    明：使用printf函数打印到串口
  */
int fputc(int ch, FILE *f)
{
    Serial_SendByte(ch);
    return ch;
}
