#ifndef __HMC5843_H
#define __HMC5843_H

#include "stm32f10x.h"

// 函数声明
void HMC5843_WriteReg(uint8_t RegAddress, uint8_t Data);
uint8_t HMC5843_ReadReg(uint8_t RegAddress);

void HMC5843_Init(void);
void HMC5843_GetData(int16_t *MagX, int16_t *MagY, int16_t *MagZ);
uint8_t HMC5843_SelfTest(void);
void HMC5843_SetMode(uint8_t mode);
uint8_t HMC5843_IsDataReady(void);

// 新增函数声明 - 获取校准后的磁力计数据
void HMC5843_GetCalibratedData(int16_t *MagX, int16_t *MagY, int16_t *MagZ);//调用这个就不用调用HMC5843_GetData了

// 函数声明 - 姿态融合相关
void UpdateYawWithMag(int16_t MagX, int16_t MagY, int16_t MagZ, 
                      float Pitch, float Roll, float *Yaw);
void UpdateCalibratedYaw(float Pitch, float Roll, float *Yaw);//这个是分布解算欧拉角的方法一，在mpu6050.c中还有一个方法二，直接计算欧拉角
//该函数结合了HMC5843_GetCalibratedData和UpdateYawWithMag函数的功能，直接使用校准后的磁力计数据更新偏航角
#endif
