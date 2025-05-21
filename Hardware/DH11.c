#include "DH11.H"
/**
 * @brief  初始化DH11所使用的IO引脚
 * @param  无
 * @retval 无
 */
void DH11_Init(void)
{
    GPIO_InitTypeDef GPIO_InitStructure;
    
    // 使能DH11_PORT端口时钟
    RCC_APB2PeriphClockCmd(DH11_RCC, ENABLE);
    
    // 配置DH11_PIN为推挽输出模式
    GPIO_InitStructure.GPIO_Pin = DH11_PIN;
    GPIO_InitStructure.GPIO_Mode = GPIO_Mode_Out_PP;
    GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;
    GPIO_Init(DH11_PORT, &GPIO_InitStructure);
    
    // 初始状态为高电平
    DH11_DATA_OUT_HIGH();
}

/**
 * @brief  配置DH11_PIN为上拉输入模式
 * @param  无
 * @retval 无
 */
void DH11_Mode_IPU(void)
{
    GPIO_InitTypeDef GPIO_InitStructure;
    
    GPIO_InitStructure.GPIO_Pin = DH11_PIN;
    GPIO_InitStructure.GPIO_Mode = GPIO_Mode_IPU;
    GPIO_Init(DH11_PORT, &GPIO_InitStructure);
}

/**
 * @brief  配置DH11_PIN为推挽输出模式
 * @param  无
 * @retval 无
 */
void DH11_Mode_Out_PP(void)
{
    GPIO_InitTypeDef GPIO_InitStructure;
    
    GPIO_InitStructure.GPIO_Pin = DH11_PIN;
    GPIO_InitStructure.GPIO_Mode = GPIO_Mode_Out_PP;
    GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;
    GPIO_Init(DH11_PORT, &GPIO_InitStructure);
}

/**
 * @brief  检查DH11是否就绪
 * @param  无
 * @retval 0: 就绪, 1: 未就绪
 */
uint8_t DH11_IsReady(void)
{
    uint8_t retry = 0;
    
    // 等待DHT11拉低电平响应信号（应该是80us低电平）
    while (DH11_DATA_IN() && retry < 100)
    {
        retry++;
        sys_Delay_us(1);
    }
    
    if(retry >= 100)
        return 1; // 超时，未检测到响应信号
    
    retry = 0;
    
    // 等待DHT11结束低电平响应信号
    while (!DH11_DATA_IN() && retry < 100)
    {
        retry++;
        sys_Delay_us(1);
    }
    
    if(retry >= 100)
        return 1; // 超时，响应信号异常
    
    retry = 0;
    
    // 等待DHT11结束高电平响应信号（约80us）
    while (DH11_DATA_IN() && retry < 100)
    {
        retry++;
        sys_Delay_us(1);
    }
    
    if(retry >= 100)
        return 1; // 超时，响应信号异常
        
    return 0; // DHT11已就绪，可以开始数据传输
}

/**
 * @brief  从DH11读取一个字节
 * @param  无
 * @retval 读取到的数据
 */
uint8_t DH11_Read_Byte(void)
{
    uint8_t i, temp = 0;
    
    for(i = 0; i < 8; i++)
    {
        // 等待50us低电平
        while(DH11_DATA_IN() == 0);
        
        // 延时30us，判断高电平持续时间以确定数据位
        sys_Delay_us(30);
        
        // 如果此时还是高电平，说明是数据'1'，否则是'0'
        if(DH11_DATA_IN() == 1)
        {
            temp |= (uint8_t)(0x01 << (7 - i)); // 高位在前
            // 等待高电平结束
            while(DH11_DATA_IN() == 1);
        }
    }
    
    return temp;
}

/**
 * @brief  验证DH11数据的校验和
 * @param  data: 指向存储读取数据的结构体的指针
 * @retval 0: 校验通过, 1: 校验不通过
 */
uint8_t DH11_Check_Sum(DHT11_Data_TypeDef* data)
{
    uint8_t sum;
    // 计算校验和
    sum = data->humidity_int + data->humidity_dec + 
          data->temperature_int + data->temperature_dec;
    
    // 校验
    if(sum == data->check_sum)
        return 0; // 校验通过
    else
        return 1; // 校验不通过
}

/**
 * @brief  读取DH11传感器数据
 * @param  data: 指向存储读取数据的结构体的指针
 * @retval 0: 成功, 1: 校验和错误, 2: 超时错误
 */
uint8_t DH11_Read_Data(DHT11_Data_TypeDef* data)
{
    // 复位数据结构
    data->humidity_int = 0;
    data->humidity_dec = 0;
    data->temperature_int = 0;
    data->temperature_dec = 0;
    data->check_sum = 0;
    data->error = DH11_OK;
    
    // 设置为输出模式
    DH11_Mode_Out_PP();
    // 主机发送开始信号，拉低至少18ms
    DH11_DATA_OUT_LOW();
    sys_Delay_ms(20); // 等待20ms
    // 拉高20-40us
    DH11_DATA_OUT_HIGH();
    sys_Delay_us(30);
    
    // 设置为输入模式，准备接收DHT11响应
    DH11_Mode_IPU();
    
    // 检查DHT11是否正确响应（包含等待完整的80us低+80us高响应信号）
    if(DH11_IsReady())
    {
        data->error = DH11_ERROR_TIMEOUT;
        return data->error;
    }
    
    // DHT11已准备好发送数据，开始读取40位数据
    data->humidity_int = DH11_Read_Byte();
    data->humidity_dec = DH11_Read_Byte();
    data->temperature_int = DH11_Read_Byte();
    data->temperature_dec = DH11_Read_Byte();
    data->check_sum = DH11_Read_Byte();
    
    // 设置为输出模式，释放总线
    DH11_Mode_Out_PP();
    DH11_DATA_OUT_HIGH();
    
    // 检查校验和
    if(DH11_Check_Sum(data))
    {
        data->error = DH11_ERROR_CHECKSUM;
        return data->error;
    }
    
    return DH11_OK;
}

/**
 * @brief  周期性读取DH11传感器数据
 * @param  data: 指向存储读取数据的结构体的指针
 * @param  interval: 读取间隔次数
 * @retval 0: 成功读取, 1: 校验和错误, 2: 超时错误, 3: 本次未读取数据
 * @note   该函数在每次调用时会检查计数器是否达到指定的间隔次数，如果达到，则读取数据并重置计数器；如果未达到，则返回3表示本次未读取数据。
 */
uint8_t DH11_Read_Data_Periodic(DHT11_Data_TypeDef* data, uint16_t interval)
{
    static uint16_t counter = 0;
    
    counter++;
    
    // 当计数达到间隔值时读取数据
    if(counter >= interval)
    {
        counter = 0;
        return DH11_Read_Data(data); // 成功读取数据
    }
    
    return 3; // 表示本次未读取数据
}
