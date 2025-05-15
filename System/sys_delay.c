#include "sys_delay.h"

static volatile uint32_t g_systickCount = 0;

/**
 * @brief  SysTick初始化函数
 * @param  无
 * @retval 无
 */
void SysTick_Init(void)
{
    // 配置SysTick中断优先级
    NVIC_SetPriority(SysTick_IRQn, 0x0F);
    
    // 配置SysTick, 72MHz/8=9MHz -> 9滴答/微秒
    SysTick_CLKSourceConfig(SysTick_CLKSource_HCLK_Div8);
    
    // 初始化滴答计数器
    g_systickCount = 0;
    
    // 使能SysTick中断，每1ms触发一次
    SysTick_Config(9000); // 9000滴答 = 1ms
}

/**
 * @brief  微秒延时函数
 * @param  nus: 延时的微秒数
 * @retval 无
 */
void sys_Delay_us(uint32_t nus)
{
    uint32_t temp;
    // 确保加载值不超过SysTick最大值
    if (nus > 0) {
        // 配置SysTick为9MHz，每微秒9个时钟
        SysTick->LOAD = 9 * nus;
        SysTick->VAL = 0x00;
        SysTick->CTRL |= SysTick_CTRL_ENABLE_Msk;
        
        // 等待倒计时完成
        do {
            temp = SysTick->CTRL;
        } while((temp & 0x01) && !(temp & (1 << 16)));
        
        // 关闭SysTick
        SysTick->CTRL &= ~SysTick_CTRL_ENABLE_Msk;
        SysTick->VAL = 0x00;
    }
}

/**
 * @brief  毫秒延时函数
 * @param  nms: 延时的毫秒数
 * @retval 无
 */
void sys_Delay_ms(uint32_t nms)
{
    for(uint32_t i = 0; i < nms; i++)
    {
        sys_Delay_us(1000);
    }
}

/**
 * @brief  秒延时函数
 * @param  ns: 延时的秒数
 * @retval 无
 */
void sys_Delay_s(uint32_t ns)
{
    for(uint32_t i = 0; i < ns; i++)
    {
        sys_Delay_ms(1000);
    }
}

