#include "stm32f10x.h"                  // Device header
#include "OLED.h"
#include "sys_delay.h"
#include "MPU6050.h"
//#include "HMC5843.h"
#include "Hall.h"
#include "sg04.h"
#include "stime.h"
#include "usart.h"
#include "key.h"                        //初始化不由My_SystemInit()函数执行，
#include "math.h"   
#include "DH11.h"                    
// 全局变量,用于欧拉角的姿态解算
int16_t AX, AY, AZ, GX, GY, GZ;
//int16_t MagX, MagY, MagZ; 
float Pitch, Roll, Yaw;
float Slope; // 存储计算出的坡度值
// 系统初始化函数
void My_SystemInit(void);
#define OBSTACLE_DISTANCE     150.0f      // 障碍物警报距离 (m)
// 计算坡度(单位: %)
float CalculateSlope(float pitch);

// 显示函数声明
extern void Display_RealtimeData(float slope, float direction);
extern void Display_LockedData(void);

// 数据采集和处理函数
void CollectSensorData(void);
DHT11_Data_TypeDef dht11_data; // 定义DHT11数据结构体
int main(void)
{   
    // 系统初始化
    My_SystemInit();
    
    // 进入设置菜单，设置目标速度和距离
    Key_SettingsMenu();
 
    // 确保启动骑行时间计时 - 取消注释
    //STime_Start();
    
    // 主循环
    while (1)
    {
        // 1. 采集传感器数据 (MPU6050, HMC5843, 超声波, DHT11)
        CollectSensorData();
        
        // 2. 计算坡度 (根据pitch角度)
        Slope = CalculateSlope(Pitch);
        //Slope = Pitch; // 直接使用pitch角度作为坡度
        float distance = HCSR04_GetDistance();
            if(distance>1&&(distance < OBSTACLE_DISTANCE)) 
            {
                Buzzer_ON();

            } else {
                Buzzer_OFF();
            }
        OLED_ShowNum(4, 12, (uint32_t)distance, 3);
    
        // 5. 处理按键和警报
        Key_Process();
            // 处理超声波测距警报
            

        // 3. 只有在骑行未结束且未锁存时更新时间
         STime_Update();//计算总时间
        // 4. 处理霍尔传感器数据 - 只在未锁存状态下进行
        if (!STime_IsDataLocked()) {
            Hall_Process();
        }


        // 6. 根据当前显示模式更新显示
        if (g_displayMode == DISPLAY_REALTIME) {
            Display_RealtimeData(Slope, Yaw);
        } else {

            Display_LockedData();
        }
        
    }

}

// 系统初始化函数
void My_SystemInit(void)
{
    // 设置中断优先级分组
    NVIC_PriorityGroupConfig(NVIC_PriorityGroup_2);
    
    // 初始化延时函数
    SysTick_Init();
    
    // 初始化OLED显示屏
    OLED_Init();
    
    // 显示启动信息
    OLED_ShowString(1, 1, "Bike Computer");
    OLED_ShowString(2, 1, "Initializing...");
    
    // 初始化硬件
    MPU6050_Init();
    //HMC5843_Init();
    Hall_Init();
    STime_Init();
    HCSR04_led_Init();
    Key_Init(); // 初始化按键、LED和蜂鸣器
    Serial_Init();                         // 串口初始化
    DH11_Init();                           // 初始化DH11传感器

    

}

// 计算坡度
float CalculateSlope(float pitch)
{
    // 仅使用pitch角度就可以近似计算坡度
    // 坡度(%) = tan(pitch) * 100
    return tanf(pitch * 0.0174533f) * 100.0f; // 转化为弧度并计算坡度百分比
}

// 采集传感器数据
void CollectSensorData(void)
{
    // 获取MPU6050加速度计和陀螺仪数据
    MPU6050_GetData(&AX, &AY, &AZ, &GX, &GY, &GZ);
    
    // 获取HMC5843磁力计校准后的数据
    //HMC5843_GetCalibratedData(&MagX, &MagY, &MagZ);
    
    // 综合处理IMU和磁力计数据，计算欧拉角
    //IMU_ComputeEulerAngles(AX, AY, AZ, GX, GY, GZ, MagX, MagY, MagZ, &Pitch, &Roll, &Yaw);
    MPU6050_ProcessData(&AX, &AY, &AZ, &GX, &GY, &GZ, &Pitch, &Roll, &Yaw);
    // 启动超声波测量
    HCSR04_StartMeasure();
    // 获取DH11传感器数据 - 每2秒读取一次(大约200次循环)
    DH11_Read_Data_Periodic(&dht11_data, 200);
}

