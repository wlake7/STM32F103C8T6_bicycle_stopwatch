#include "stm32f10x.h"
#include "sys_delay.h"
#include "sg04.h"

uint8_t HCSR04_CompleteFlag;  // 测量完成标志
float HCSR04_Distance;        // 测量距离结果
static uint32_t HCSR04_OverflowCount = 0;  // 溢出次数计数





void HCSR04_led_Init(void)
{
    // 时钟使能
    RCC_APB2PeriphClockCmd(RCC_APB2Periph_GPIOA|RCC_APB2Periph_GPIOB, ENABLE);
    RCC_APB1PeriphClockCmd(RCC_APB1Periph_TIM3, ENABLE);
    //LED引脚配置
    GPIO_InitTypeDef GPIO_InitLED;
    GPIO_InitLED.GPIO_Mode = GPIO_Mode_Out_PP;
    GPIO_InitLED.GPIO_Pin = LED_T;
    GPIO_InitLED.GPIO_Speed = GPIO_Speed_50MHz;
    GPIO_Init(GPIOB, &GPIO_InitLED);
    
    // Trig引脚配置
    GPIO_InitTypeDef GPIO_InitTrig;
    GPIO_InitTrig.GPIO_Mode = GPIO_Mode_Out_PP;
    GPIO_InitTrig.GPIO_Pin = HCSR04_Trig;
    GPIO_InitTrig.GPIO_Speed = GPIO_Speed_50MHz;
    GPIO_Init(GPIOA, &GPIO_InitTrig);
    
    // Echo引脚配置为输入捕获
    GPIO_InitTypeDef GPIO_InitEcho;
    GPIO_InitEcho.GPIO_Speed = GPIO_Speed_50MHz;
    GPIO_InitEcho.GPIO_Mode = GPIO_Mode_IPD;
    GPIO_InitEcho.GPIO_Pin = HCSR04_Echo;
    GPIO_Init(GPIOA, &GPIO_InitEcho);
    
    // 定时器3初始化
    TIM_TimeBaseInitTypeDef TIM_TimeBaseStructure;
    TIM_TimeBaseStructure.TIM_Period = 0xFFFF;
    TIM_TimeBaseStructure.TIM_Prescaler = 72 - 1;  // 1MHz计数频率
    TIM_TimeBaseStructure.TIM_ClockDivision = TIM_CKD_DIV1;
    TIM_TimeBaseStructure.TIM_CounterMode = TIM_CounterMode_Up;
    TIM_TimeBaseInit(TIM3, &TIM_TimeBaseStructure);
    
    // 输入捕获配置
    TIM_ICInitTypeDef TIM_ICInitStructure;
    TIM_ICInitStructure.TIM_Channel = TIM_Channel_1;
    TIM_ICInitStructure.TIM_ICPolarity = TIM_ICPolarity_Rising;   // 上升沿触发
    TIM_ICInitStructure.TIM_ICSelection = TIM_ICSelection_DirectTI;
    TIM_ICInitStructure.TIM_ICPrescaler = TIM_ICPSC_DIV1;
    TIM_ICInitStructure.TIM_ICFilter = 0x00;
    TIM_ICInit(TIM3, &TIM_ICInitStructure);
    
    // NVIC配置
    NVIC_InitTypeDef NVIC_InitStructure;
    NVIC_InitStructure.NVIC_IRQChannel = TIM3_IRQn;
    NVIC_InitStructure.NVIC_IRQChannelPreemptionPriority = 2;
    NVIC_InitStructure.NVIC_IRQChannelSubPriority = 0;
    NVIC_InitStructure.NVIC_IRQChannelCmd = ENABLE;
    NVIC_Init(&NVIC_InitStructure);
    
    // 使能定时器3中断
    TIM_ITConfig(TIM3, TIM_IT_CC1 | TIM_IT_Update, ENABLE);
    TIM_Cmd(TIM3, ENABLE);
}

void HCSR04_StartMeasure(void)
{
    HCSR04_CompleteFlag = 0;    // 清除完成标志
    HCSR04_OverflowCount = 0;   // 清除溢出计数
    
    // 增加测量前的延时，确保传感器稳定
    sys_Delay_ms(10);
    
    // 发送触发信号
    GPIO_ResetBits(GPIOA, HCSR04_Trig);
    sys_Delay_us(2);
    GPIO_SetBits(GPIOA, HCSR04_Trig);
    sys_Delay_us(12);
    GPIO_ResetBits(GPIOA, HCSR04_Trig);
    
    TIM_SetCounter(TIM3, 0);  // 清零计数器
}


// 添加自定义绝对值函数
static float My_fabs(float x)
{
    return x < 0 ? -x : x;
}
void led_on(void)
{

}



// 修改为使用平均值的新函数(放入循环中)
float HCSR04_GetDistance(void)
{
    float samples[SAMPLE_COUNT];
    float avg = 0;
    uint8_t valid_count = 0;
    
    // 获取多次采样
    for(uint8_t i = 0; i < SAMPLE_COUNT; i++)   //valid_count是有效数据的个数，绝对比SAMPLE_COUNT小
    {
        HCSR04_StartMeasure();//进行测距的开始地方，此时开始和中断函数进行配合获取数据并处理，每次测距都要使用这个起始信号
        while(!HCSR04_CompleteFlag);
        if(HCSR04_Distance > 0)
        {
            samples[i] = HCSR04_Distance;
            avg += HCSR04_Distance;
            valid_count++;
        }
        sys_Delay_ms(30);  // 采样间隔
    }
    
    if(valid_count == 0) return 0;  // 没有有效数据
    
    // 计算初步平均值
    avg /= valid_count;
    
    // 第二次平均，排除偏差过大的值
    float final_avg = 0;
    valid_count = 0;
    
    for(uint8_t i = 0; i < SAMPLE_COUNT; i++) {
        if(samples[i] > 0 && My_fabs(samples[i] - avg) < MAX_DEVIATION) {  // 使用 My_fabs 替换 fabs
            final_avg += samples[i];
            valid_count++;
        }
    }
    
    return (valid_count > 0) ? (final_avg / valid_count) : 0;
}

// TIM3中断服务函数（获取数据并处理得到距离）
void TIM3_IRQHandler(void)
{
    static uint16_t risingValue = 0;  // 上升沿计数值
    uint32_t totalTime;
    
    // 处理更新中断（溢出）
    if(TIM_GetITStatus(TIM3, TIM_IT_Update) != RESET)
    {
        HCSR04_OverflowCount++;
        if(HCSR04_OverflowCount > (HCSR04_TIMEOUT / 65536))  // 超时检测
        {
            TIM_OC1PolarityConfig(TIM3, TIM_ICPolarity_Rising);  // 重置为上升沿捕获
            HCSR04_Distance = 0;  // 距离置0表示测量失败
            HCSR04_CompleteFlag = 1;
            HCSR04_OverflowCount = 0;
        }
        TIM_ClearITPendingBit(TIM3, TIM_IT_Update);
    }
    
    if(TIM_GetITStatus(TIM3, TIM_IT_CC1) != RESET)
    {
        if(GPIO_ReadInputDataBit(GPIOA, GPIO_Pin_6) == 1)  // 上升沿
        {
            risingValue = TIM_GetCapture1(TIM3);  // 记录上升沿时间
            TIM_OC1PolarityConfig(TIM3, TIM_ICPolarity_Falling);  // 切换为下降沿捕获
            HCSR04_OverflowCount = 0;  // 重置溢出计数
        }
        else  // 下降沿(距离捕获成功)
        {
            uint16_t fallingValue = TIM_GetCapture1(TIM3);
            // 计算总时间，考虑溢出
            totalTime = HCSR04_OverflowCount * 65536 + fallingValue - risingValue;
            
            // 简化温度补偿计算，使用固定声速值
            float sound_speed = 346.0;  // 25℃时的声速，单位：m/s
            HCSR04_Distance = (float)totalTime * (sound_speed / 20000.0);
            
            // 距离限制
            if(HCSR04_Distance > HCSR04_MAX_DISTANCE)
            {
                HCSR04_Distance = HCSR04_MAX_DISTANCE;
            }
            else if(HCSR04_Distance < HCSR04_MIN_DISTANCE)
            {
                HCSR04_Distance = 0;  // 小于最小距离视为无效
            }
            if (HCSR04_Distance<LED_Reminder)
            {
                GPIO_SetBits(GPIOB, LED_T);  // 点亮LED灯
            }
            else
            {
                GPIO_ResetBits(GPIOB, LED_T);  // 熄灭LED灯
            }
            TIM_OC1PolarityConfig(TIM3, TIM_ICPolarity_Rising);   // 切换回上升沿捕获
            HCSR04_CompleteFlag = 1;  // 设置完成标志

        }

        
        TIM_ClearITPendingBit(TIM3, TIM_IT_CC1);
    }
}

