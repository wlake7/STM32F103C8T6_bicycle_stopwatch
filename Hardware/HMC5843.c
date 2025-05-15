#include "stm32f10x.h"
#include "HMC58_Reg.h"
#include "HMC58_Reg.h"  // 保留原始文件名
#include "MPU6050.h"  // 引入通用I2C函数
#include "math.h"

// 导入MPU6050中定义的等待事件函数
extern void I2C_WaitEvent(I2C_TypeDef* I2Cx, uint32_t I2C_EVENT);

// 磁力计校准参数 - 硬磁偏移矫正
#define MAG_OFFSET_X 7.0f   // 硬磁偏移 X轴
#define MAG_OFFSET_Y -98.0f // 硬磁偏移 Y轴
#define MAG_OFFSET_Z -75.0f // 硬磁偏移 Z轴

// 磁力计校准参数 - 软磁缩放因子
#define MAG_SCALE_X 0.91637f // 软磁缩放 X轴
#define MAG_SCALE_Y 1.00855f // 软磁缩放 Y轴
#define MAG_SCALE_Z 1.09026f // 软磁缩放 Z轴

/**
  * 函    数：HMC5843写寄存器，使用通用I2C写函数
  * 参    数：RegAddress 寄存器地址
  * 参    数：Data 要写入寄存器的数据
  * 返 回 值：无
  */
void HMC5843_WriteReg(uint8_t RegAddress, uint8_t Data)
{
    I2C_WriteReg(HMC5883L_ADDRESS, RegAddress, Data);
}

/**
  * 函    数：HMC5843读寄存器，使用通用I2C读函数
  * 参    数：RegAddress 寄存器地址
  * 返 回 值：读取寄存器的数据
  */
uint8_t HMC5843_ReadReg(uint8_t RegAddress)
{
     return I2C_ReadReg(HMC5883L_ADDRESS, RegAddress);
}

/**
  * 函    数：检查HMC5843(5883L)是否就绪
  * 参    数：无
  * 返 回 值：1表示数据就绪，0表示数据未就绪
  */
uint8_t HMC5843_IsDataReady(void)
{
    uint8_t status = HMC5843_ReadReg(HMC5883L_STATUS_REG);
    return (status & 0x01); // 返回状态寄存器的第0位(RDY位)
}

/**
  * 函    数：HMC5843(5883L)初始化
  * 参    数：无
  * 返 回 值：无
  */
 void HMC5843_Init(void)
 {
     // 注意：I2C2时钟与GPIO配置已在MPU6050_Init中完成，此处不需要重复配置
     // 配置HMC5883L
     
     // 1. 配置寄存器A：设置8次平均, 15Hz输出速率, 正常测量模式
     HMC5843_WriteReg(HMC5883L_CONFIG_REG_A, HMC5883L_AVG_8 | HMC5883L_DATARATE_15_HZ | HMC5883L_MEASURE_NORMAL);
     
     // 2. 配置寄存器B：设置增益为1.3Ga
     HMC5843_WriteReg(HMC5883L_CONFIG_REG_B, HMC5883L_GAIN_1_3GA);
     
     // 3. 设置工作模式为连续测量模式
     HMC5843_WriteReg(HMC5883L_MODE_REG, HMC5883L_MODE_CONTINUOUS);
 }

/**
  * 函    数：HMC5843(5883L)自测试
  * 参    数：无
  * 返 回 值：1表示测试通过，0表示测试失败
  */
uint8_t HMC5843_SelfTest(void)
{
    uint8_t id_a = HMC5843_ReadReg(HMC5883L_ID_REG_A);
    uint8_t id_b = HMC5843_ReadReg(HMC5883L_ID_REG_B);
    uint8_t id_c = HMC5843_ReadReg(HMC5883L_ID_REG_C);
    
    // 检查ID寄存器值是否为"H43" (也适用于HMC5883L)
    return (id_a == 0x48 && id_b == 0x34 && id_c == 0x33);
}

/**
  * 函    数：设置HMC5843(5883L)工作模式
  * 参    数：mode 工作模式
  * 返 回 值：无
  */
void HMC5843_SetMode(uint8_t mode)
{
    HMC5843_WriteReg(HMC5883L_MODE_REG, mode);
}

/**
  * 函    数：获取HMC5843(5883L)磁力计数据
  * 参    数：MagX MagY MagZ 磁力计三轴数据的指针
  * 返 回 值：无
  * 注    意：HMC5883L的数据寄存器顺序是 X, Z, Y
  */
void HMC5843_GetData(int16_t *MagX, int16_t *MagY, int16_t *MagZ)
{
    uint8_t DataH, DataL;
    
    // 读取X轴磁力计数据
    DataH = HMC5843_ReadReg(HMC5883L_DATA_OUT_X_MSB);
    DataL = HMC5843_ReadReg(HMC5883L_DATA_OUT_X_LSB);
    *MagX = ((int16_t)DataH << 8) | DataL;
    
    // 读取Z轴磁力计数据 (注意: HMC5883L的数据顺序是X,Z,Y)
    DataH = HMC5843_ReadReg(HMC5883L_DATA_OUT_Z_MSB);
    DataL = HMC5843_ReadReg(HMC5883L_DATA_OUT_Z_LSB);
    *MagZ = ((int16_t)DataH << 8) | DataL;
    
    // 读取Y轴磁力计数据
    DataH = HMC5843_ReadReg(HMC5883L_DATA_OUT_Y_MSB);
    DataL = HMC5843_ReadReg(HMC5883L_DATA_OUT_Y_LSB);
    *MagY = ((int16_t)DataH << 8) | DataL;
}

/**
  * 函    数：获取校准后的HMC5843(5883L)磁力计数据
  * 参    数：MagX MagY MagZ 校准后磁力计三轴数据的指针
  * 返 回 值：无
  * 说    明：应用硬磁偏移和软磁缩放校准
  */
void HMC5843_GetCalibratedData(int16_t *MagX, int16_t *MagY, int16_t *MagZ)
{
    int16_t rawX, rawY, rawZ;
    float calX, calY, calZ;
    
    // 获取原始磁力计数据
    HMC5843_GetData(&rawX, &rawY, &rawZ);//注意一下这里已经调用了获取数据的函数，所以这个函数是在getdata函数的基础上进行的校准
    
    // 应用硬磁偏移和软磁缩放校准
    calX = (rawX - MAG_OFFSET_X) * MAG_SCALE_X;
    calY = (rawY - MAG_OFFSET_Y) * MAG_SCALE_Y;
    calZ = (rawZ - MAG_OFFSET_Z) * MAG_SCALE_Z;
    
    // 将校准后的浮点值转换回整数并返回
    *MagX = (int16_t)calX;
    *MagY = (int16_t)calY;
    *MagZ = (int16_t)calZ;
}

/**
  * 函    数：使用磁力计数据更新偏航角
  * 参    数：MagX MagY MagZ 磁力计三轴数据
  * 参    数：Pitch Roll 当前的俯仰角和横滚角（弧度）
  * 参    数：Yaw 偏航角指针，更新后的偏航角将存储于此
  * 返 回 值：无
  */
void UpdateYawWithMag(int16_t MagX, int16_t MagY, int16_t MagZ, 
                      float Pitch, float Roll, float *Yaw)
{
    // 将姿态角转换为弧度，用于后续计算
    float pitch_rad = Pitch * 0.0174533f; // 角度转弧度
    float roll_rad = Roll * 0.0174533f;   // 角度转弧度
    
    // 磁力计数据归一化（这里简化处理，实际应用中可能需要校准）
    float norm = sqrtf(MagX*MagX + MagY*MagY + MagZ*MagZ);
    if (norm == 0) return;
    
    float mx = MagX / norm;
    float my = MagY / norm;
    float mz = MagZ / norm;
    
    // 应用倾角补偿(Tilt Compensation)，补偿地磁向量在倾斜坐标系下的投影
    // 将磁力计数据从传感器坐标系转换到水平坐标系
    float cos_roll = cosf(roll_rad);
    float sin_roll = sinf(roll_rad);
    float cos_pitch = cosf(pitch_rad);
    float sin_pitch = sinf(pitch_rad);
    
    // 补偿计算
    float Xh = mx * cos_pitch + mz * sin_pitch;
    float Yh = mx * sin_roll * sin_pitch + my * cos_roll - mz * sin_roll * cos_pitch;
    
    // 计算偏航角（磁北方向）
    float heading = atan2f(Yh, Xh);
    
    // 转换为角度
    heading = heading * 57.295779f;  // 弧度转角度
    
    // 确保结果在0-359.99度范围内
    if (heading < 0)
        heading += 360.0f;
    
    // 更新偏航角
    *Yaw = heading;
}

/**
  * 函    数：使用校准后的磁力计数据更新偏航角
  * 参    数：Pitch Roll 当前的俯仰角和横滚角（角度）
  * 参    数：Yaw 偏航角指针，更新后的偏航角将存储于此
  * 返 回 值：无
  * 说    明：内部自动获取并校准磁力计数据
  */
void UpdateCalibratedYaw(float Pitch, float Roll, float *Yaw)
{
    int16_t MagX, MagY, MagZ;
    
    // 获取校准后的磁力计数据
    HMC5843_GetCalibratedData(&MagX, &MagY, &MagZ);
    
    // 使用校准后的数据更新偏航角
    UpdateYawWithMag(MagX, MagY, MagZ, Pitch, Roll, Yaw);
}
