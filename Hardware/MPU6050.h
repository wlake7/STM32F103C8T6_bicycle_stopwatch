#ifndef __MPU6050_H
#define __MPU6050_H
// 通用I2C函数声明
void I2C_WaitEvent(I2C_TypeDef* I2Cx, uint32_t I2C_EVENT);
void I2C_WriteReg(uint8_t DevAddress, uint8_t RegAddress, uint8_t Data);
uint8_t I2C_ReadReg(uint8_t DevAddress, uint8_t RegAddress);

// MPU6050特定函数
void MPU6050_WriteReg(uint8_t RegAddress, uint8_t Data);
uint8_t MPU6050_ReadReg(uint8_t RegAddress);

void MPU6050_Init(void);
uint8_t MPU6050_GetID(void);
void MPU6050_GetData(int16_t *AccX, int16_t *AccY, int16_t *AccZ, 
						int16_t *GyroX, int16_t *GyroY, int16_t *GyroZ);
void MPU6050_ProcessData(int16_t *AX, int16_t *AY, int16_t *AZ, int16_t *GX, int16_t *GY, int16_t *GZ, float *Pitch, float *Roll, float *Yaw);

// 新增：综合处理IMU和磁力计数据，计算欧拉角
void IMU_ComputeEulerAngles(
    int16_t AccX, int16_t AccY, int16_t AccZ,
    int16_t GyroX, int16_t GyroY, int16_t GyroZ,
    int16_t MagX, int16_t MagY, int16_t MagZ,
    float *Pitch, float *Roll, float *Yaw);
void MPU6050_GetHorizontalAcceleration(
    int16_t AccX, int16_t AccY, int16_t AccZ,
    float Pitch, float Roll, float Yaw,
    float *HorizontalAcc_mps2);
#endif
//mpu6050函数使用方法
//1.在main.c中包含头文件：#include "MPU6050.h"
//2.初始化结构体
//3.在main函数中调用MPU6050_Init()函数进行初始化
//4.在循环中调用MPU6050_GetData()函数获取加速度计和陀螺仪数据，存储在AX, AY, AZ, GX, GY, GZ变量中
//5.调用MPU6050_ProcessData()函数进行数据处理，得到Pitch, Roll, Yaw角度值
//综合姿态解算使用方法
//1.在main.c中包含两个外设的头文件
//2.依次调用函数HMC5843_GetCalibratedData(&MagX, &MagY, &MagZ);
//3.MPU6050_GetData(&AX, &AY, &AZ, &GX, &GY, &GZ);
//4.IMU_ComputeEulerAngles(AX, AY, AZ, GX, GY, GZ, MagX, MagY, MagZ, &Pitch, &Roll, &Yaw);
