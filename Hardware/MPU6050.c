#include "stm32f10x.h"                  // Device header
#include "MPU6050_Reg.h"
#include "math.h"
#define MPU6050_ADDRESS		0xD0		//MPU6050的I2C从机地址

/**
  * 函    数：I2C等待事件
  * 参    数：同I2C_CheckEvent
  * 返 回 值：无
  */
void I2C_WaitEvent(I2C_TypeDef* I2Cx, uint32_t I2C_EVENT)
{
	uint32_t Timeout;
	Timeout = 10000;									//给定超时计数时间
	while (I2C_CheckEvent(I2Cx, I2C_EVENT) != SUCCESS)	//循环等待指定事件
	{
		Timeout --;										//等待时，计数值自减
		if (Timeout == 0)								//自减到0后，等待超时
		{
			/*超时的错误处理代码，可以添加到此处*/
			break;										//跳出等待，不等了
		}
	}
}

/**
  * 函    数：I2C写寄存器，适用于多个I2C设备
  * 参    数：DevAddress 设备I2C地址
  * 参    数：RegAddress 寄存器地址
  * 参    数：Data 要写入寄存器的数据
  * 返 回 值：无
  */
void I2C_WriteReg(uint8_t DevAddress, uint8_t RegAddress, uint8_t Data)
{
	I2C_GenerateSTART(I2C2, ENABLE);										//硬件I2C生成起始条件
	I2C_WaitEvent(I2C2, I2C_EVENT_MASTER_MODE_SELECT);					    //等待EV5
	
	I2C_Send7bitAddress(I2C2, DevAddress, I2C_Direction_Transmitter);	    //硬件I2C发送从机地址，方向为发送
	I2C_WaitEvent(I2C2, I2C_EVENT_MASTER_TRANSMITTER_MODE_SELECTED);	    //等待EV6
	
	I2C_SendData(I2C2, RegAddress);											//硬件I2C发送寄存器地址
	I2C_WaitEvent(I2C2, I2C_EVENT_MASTER_BYTE_TRANSMITTING);			    //等待EV8
	
	I2C_SendData(I2C2, Data);												//硬件I2C发送数据
	I2C_WaitEvent(I2C2, I2C_EVENT_MASTER_BYTE_TRANSMITTED);				    //等待EV8_2
	
	I2C_GenerateSTOP(I2C2, ENABLE);											//硬件I2C生成终止条件
}

/**
  * 函    数：I2C读寄存器，适用于多个I2C设备
  * 参    数：DevAddress 设备I2C地址
  * 参    数：RegAddress 寄存器地址
  * 返 回 值：读取寄存器的数据
  */
uint8_t I2C_ReadReg(uint8_t DevAddress, uint8_t RegAddress)
{
	uint8_t Data;
	
	I2C_GenerateSTART(I2C2, ENABLE);										//硬件I2C生成起始条件
	I2C_WaitEvent(I2C2, I2C_EVENT_MASTER_MODE_SELECT);					    //等待EV5
	
	I2C_Send7bitAddress(I2C2, DevAddress, I2C_Direction_Transmitter);	    //硬件I2C发送从机地址，方向为发送
	I2C_WaitEvent(I2C2, I2C_EVENT_MASTER_TRANSMITTER_MODE_SELECTED);	    //等待EV6
	
	I2C_SendData(I2C2, RegAddress);											//硬件I2C发送寄存器地址
	I2C_WaitEvent(I2C2, I2C_EVENT_MASTER_BYTE_TRANSMITTED);				    //等待EV8_2
	
	I2C_GenerateSTART(I2C2, ENABLE);										//硬件I2C生成重复起始条件
	I2C_WaitEvent(I2C2, I2C_EVENT_MASTER_MODE_SELECT);					    //等待EV5
	
	I2C_Send7bitAddress(I2C2, DevAddress, I2C_Direction_Receiver);		    //硬件I2C发送从机地址，方向为接收
	I2C_WaitEvent(I2C2, I2C_EVENT_MASTER_RECEIVER_MODE_SELECTED);		    //等待EV6
	
	I2C_AcknowledgeConfig(I2C2, DISABLE);									//在接收最后一个字节之前提前将应答失能
	I2C_GenerateSTOP(I2C2, ENABLE);											//在接收最后一个字节之前提前申请停止条件
	
	I2C_WaitEvent(I2C2, I2C_EVENT_MASTER_BYTE_RECEIVED);				    //等待EV7
	Data = I2C_ReceiveData(I2C2);											//接收数据寄存器
	
	I2C_AcknowledgeConfig(I2C2, ENABLE);									//将应答恢复为使能，为了不影响后续可能产生的读取多字节操作
	
	return Data;
}

/**
  * 函    数：MPU6050写寄存器，基于通用I2C写函数
  * 参    数：RegAddress 寄存器地址
  * 参    数：Data 要写入寄存器的数据
  * 返 回 值：无
  */
void MPU6050_WriteReg(uint8_t RegAddress, uint8_t Data)
{
    I2C_WriteReg(MPU6050_ADDRESS, RegAddress, Data);
}

/**
  * 函    数：MPU6050读寄存器，基于通用I2C读函数
  * 参    数：RegAddress 寄存器地址
  * 返 回 值：读取寄存器的数据
  */
uint8_t MPU6050_ReadReg(uint8_t RegAddress)
{
    return I2C_ReadReg(MPU6050_ADDRESS, RegAddress);
}

/**
  * 函    数：MPU6050初始化
  * 参    数：无
  * 返 回 值：无
  */
void MPU6050_Init(void)
{
	/*开启时钟*/
	RCC_APB1PeriphClockCmd(RCC_APB1Periph_I2C2, ENABLE);		//开启I2C2的时钟
	RCC_APB2PeriphClockCmd(RCC_APB2Periph_GPIOB, ENABLE);		//开启GPIOB的时钟
	
	/*GPIO初始化*/
	GPIO_InitTypeDef GPIO_InitStructure;
	GPIO_InitStructure.GPIO_Mode = GPIO_Mode_AF_OD;
	GPIO_InitStructure.GPIO_Pin = GPIO_Pin_10 | GPIO_Pin_11;
	GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;
	GPIO_Init(GPIOB, &GPIO_InitStructure);					//将PB10和PB11引脚初始化为复用开漏输出
	
	/*I2C初始化*/
	I2C_InitTypeDef I2C_InitStructure;						//定义结构体变量
	I2C_InitStructure.I2C_Mode = I2C_Mode_I2C;				//模式，选择为I2C模式
	I2C_InitStructure.I2C_ClockSpeed = 50000;				//时钟速度，选择为50KHz
	I2C_InitStructure.I2C_DutyCycle = I2C_DutyCycle_2;		//时钟占空比，选择Tlow/Thigh = 2
	I2C_InitStructure.I2C_Ack = I2C_Ack_Enable;				//应答，选择使能
	I2C_InitStructure.I2C_AcknowledgedAddress = I2C_AcknowledgedAddress_7bit;	//应答地址，选择7位，从机模式下才有效
	I2C_InitStructure.I2C_OwnAddress1 = 0x00;				//自身地址，从机模式下才有效
	I2C_Init(I2C2, &I2C_InitStructure);						//将结构体变量交给I2C_Init，配置I2C2
	
	/*I2C使能*/
	I2C_Cmd(I2C2, ENABLE);									//使能I2C2，开始运行
	
	/*MPU6050寄存器初始化，需要对照MPU6050手册的寄存器描述配置，此处仅配置了部分重要的寄存器*/
	MPU6050_WriteReg(MPU6050_PWR_MGMT_1, 0x01);				//电源管理寄存器1，取消睡眠模式，选择时钟源为X轴陀螺仪
	MPU6050_WriteReg(MPU6050_PWR_MGMT_2, 0x00);				//电源管理寄存器2，保持默认值0，所有轴均不待机
	MPU6050_WriteReg(MPU6050_SMPLRT_DIV, 0x09);				//采样率分频寄存器，配置采样率
	MPU6050_WriteReg(MPU6050_CONFIG, 0x06);					//配置寄存器，配置DLPF
	MPU6050_WriteReg(MPU6050_GYRO_CONFIG, 0x18);			//陀螺仪配置寄存器，选择满量程为±2000°/s
	MPU6050_WriteReg(MPU6050_ACCEL_CONFIG, 0x18);			//加速度计配置寄存器，选择满量程为±16g
}


/**
  * 函    数：MPU6050获取ID号
  * 参    数：无
  * 返 回 值：MPU6050的ID号
  */
uint8_t MPU6050_GetID(void)
{
	return MPU6050_ReadReg(MPU6050_WHO_AM_I);		//返回WHO_AM_I寄存器的值
}

/**
  * 函    数：MPU6050获取数据
  * 参    数：AccX AccY AccZ 加速度计X、Y、Z轴的数据，使用输出参数的形式返回，范围：-32768~32767
  * 参    数：GyroX GyroY GyroZ 陀螺仪X、Y、Z轴的数据，使用输出参数的形式返回，范围：-32768~32767
  * 返 回 值：无
  */
void MPU6050_GetData(int16_t *AccX, int16_t *AccY, int16_t *AccZ, 
						int16_t *GyroX, int16_t *GyroY, int16_t *GyroZ)
{
	uint8_t DataH, DataL;								//定义数据高8位和低8位的变量
	
	DataH = MPU6050_ReadReg(MPU6050_ACCEL_XOUT_H);		//读取加速度计X轴的高8位数据
	DataL = MPU6050_ReadReg(MPU6050_ACCEL_XOUT_L);		//读取加速度计X轴的低8位数据
	*AccX = (DataH << 8) | DataL;						//数据拼接，通过输出参数返回
	
	DataH = MPU6050_ReadReg(MPU6050_ACCEL_YOUT_H);		//读取加速度计Y轴的高8位数据
	DataL = MPU6050_ReadReg(MPU6050_ACCEL_YOUT_L);		//读取加速度计Y轴的低8位数据
	*AccY = (DataH << 8) | DataL;						//数据拼接，通过输出参数返回
	
	DataH = MPU6050_ReadReg(MPU6050_ACCEL_ZOUT_H);		//读取加速度计Z轴的高8位数据
	DataL = MPU6050_ReadReg(MPU6050_ACCEL_ZOUT_L);		//读取加速度计Z轴的低8位数据
	*AccZ = (DataH << 8) | DataL;						//数据拼接，通过输出参数返回
	
	DataH = MPU6050_ReadReg(MPU6050_GYRO_XOUT_H);		//读取陀螺仪X轴的高8位数据
	DataL = MPU6050_ReadReg(MPU6050_GYRO_XOUT_L);		//读取陀螺仪X轴的低8位数据
	*GyroX = (DataH << 8) | DataL;						//数据拼接，通过输出参数返回
	
	DataH = MPU6050_ReadReg(MPU6050_GYRO_YOUT_H);		//读取陀螺仪Y轴的高8位数据
	DataL = MPU6050_ReadReg(MPU6050_GYRO_YOUT_L);		//读取陀螺仪Y轴的低8位数据
	*GyroY = (DataH << 8) | DataL;						//数据拼接，通过输出参数返回
	
	DataH = MPU6050_ReadReg(MPU6050_GYRO_ZOUT_H);		//读取陀螺仪Z轴的高8位数据
	DataL = MPU6050_ReadReg(MPU6050_GYRO_ZOUT_L);		//读取陀螺仪Z轴的低8位数据
	*GyroZ = (DataH << 8) | DataL;						//数据拼接，通过输出参数返回
}
/**
	* 函    数：MPU6050数据处理
	* 参    数：AX, AY, AZ, GX, GY, GZ 原始的加速度计与陀螺仪数据
	* 返 回 值：返回经过互补滤波后的俯仰角与横滚角，单位为度
	*/
void MPU6050_ProcessData(int16_t *AX, int16_t *AY, int16_t *AZ, int16_t *GX, int16_t *GY, int16_t *GZ, float *Pitch, float *Roll, float *Yaw)
{
	//========================相关参数的设定
	// 零偏校准值，需要实际测量后调整
	static int16_t AX_Offset = 4, AY_Offset = -32, AZ_Offset = 0;
	static int16_t GX_Offset = -30, GY_Offset = 26, GZ_Offset = 11;
	
	// 存储上一次的角度值
	static float Last_Pitch = 0, Last_Roll = 0;
	static float Last_Yaw = 0;
	static uint8_t Is_First = 1;  // 首次运行标志
	
	// 转换系数，根据量程确定
	// 加速度计: ±16g对应±32768，转换为重力加速度单位g
	// 陀螺仪: ±2000°/s对应±32768，转换为°/s
	const float Accel_Scale = 16.0f / 32768.0f;
	const float Gyro_Scale = 2000.0f / 32768.0f;
	
	// 互补滤波权重系数
	const float Alpha = 0.975f;
	
	// 采样周期，单位：秒
	const float Dt = 0.01f;  // 假设10ms调用一次

	//=================进行计算

	// 去除零偏,归零校准
	*AX -= AX_Offset;
	*AY -= AY_Offset;
	*AZ -= AZ_Offset;
	*GX -= GX_Offset;
	*GY -= GY_Offset;
	*GZ -= GZ_Offset;
	
	// 计算加速度计得到的角度
	float Accel_X = (*AX) * Accel_Scale;
	float Accel_Y = (*AY) * Accel_Scale;
	float Accel_Z = (*AZ) * Accel_Scale;
	
	// 计算加速度计测得的角度
	float Accel_Pitch = atan2f(Accel_X, sqrtf(Accel_Y * Accel_Y + Accel_Z * Accel_Z)) * 57.295779f; // 弧度转角度
	float Accel_Roll = atan2f(Accel_Y, Accel_Z) * 57.295779f; // 弧度转角度
	
	// 计算陀螺仪角速度，°/s
	float Gyro_X = (*GX) * Gyro_Scale;
	float Gyro_Y = (*GY) * Gyro_Scale;
	float Gyro_Z = (*GZ) * Gyro_Scale;
	
	// 首次运行时没有历史数据，直接使用加速度计的值
	if (Is_First)
	{
		*Pitch = Accel_Pitch;
		*Roll = Accel_Roll;
		*Yaw = 0;  // 偏航角初始值设为0
		Is_First = 0;
	}
	else
	{
		// 互补滤波
		*Pitch = Alpha * (Last_Pitch + Gyro_X * Dt) + (1 - Alpha) * Accel_Pitch;
		*Roll = Alpha * (Last_Roll + Gyro_Y * Dt) + (1 - Alpha) * Accel_Roll;
		
		// 偏航角只能通过陀螺仪积分得到，没有加速度计校正
		// 根据姿态角，补偿陀螺仪数据的坐标系转换
		float Gyro_Z_Corrected = Gyro_Z + sinf(Last_Roll * 0.0174533f) * Gyro_X + 
								cosf(Last_Roll * 0.0174533f) * sinf(Last_Pitch * 0.0174533f) * Gyro_Y;
		
		// 陀螺仪积分得到偏航角变化量
		*Yaw = Last_Yaw + Gyro_Z_Corrected * Dt;
		
		// 注意：此偏航角会随时间漂移，需要通过磁力计数据在外部进行校正
		// 在main函数中会调用HMC5843的UpdateYawWithMag函数来更新精确的偏航角
	}
	
	// 保持偏航角在-180°到180°之间
	while(*Yaw > 180) *Yaw -= 360;
	while(*Yaw < -180) *Yaw += 360;
	
	// 保存当前角度作为下次计算的历史值
	Last_Pitch = *Pitch;
	Last_Roll = *Roll;
	Last_Yaw = *Yaw;
}

/**
  * 函    数：综合处理IMU和磁力计数据，直接计算欧拉角
  * 参    数：AccX, AccY, AccZ - 加速度计原始数据
  * 参    数：GyroX, GyroY, GyroZ - 陀螺仪原始数据
  * 参    数：MagX, MagY, MagZ - 磁力计原始数据
  * 参    数：Pitch, Roll, Yaw - 计算得到的欧拉角指针(单位：度)
  * 返 回 值：无
  */
void IMU_ComputeEulerAngles(
    int16_t AccX, int16_t AccY, int16_t AccZ,
    int16_t GyroX, int16_t GyroY, int16_t GyroZ,
    int16_t MagX, int16_t MagY, int16_t MagZ,
    float *Pitch, float *Roll, float *Yaw)
{
    //========================相关参数的设定
    // 零偏校准值，需要实际测量后调整
    static int16_t AX_Offset = 4, AY_Offset = -32, AZ_Offset = 0;
    static int16_t GX_Offset = -30, GY_Offset = 26, GZ_Offset = 11;
    
    // 存储上一次的角度值
    static float Last_Pitch = 0, Last_Roll = 0, Last_Yaw = 0;
    static uint8_t Is_First = 1;  // 首次运行标志
    
    // 转换系数，根据量程确定
    const float Accel_Scale = 16.0f / 32768.0f;  // 加速度计: ±16g对应±32768
    const float Gyro_Scale = 2000.0f / 32768.0f;  // 陀螺仪: ±2000°/s对应±32768
    
    // 互补滤波权重系数
    const float Alpha = 0.975f;       // 加速度计和陀螺仪的融合系数
    const float Mag_Alpha = 0.95f;    // 磁力计融合系数，稍微低些因为磁力计可能有更多噪声
    
    // 采样周期，单位：秒
    const float Dt = 0.01f;  // 假设10ms调用一次

    //=================进行计算

    // 去除零偏
    AccX -= AX_Offset;
    AccY -= AY_Offset;
    AccZ -= AZ_Offset;
    GyroX -= GX_Offset;
    GyroY -= GY_Offset;
    GyroZ -= GZ_Offset;
    
    // 1.处理加速度计数据
    float Accel_X = AccX * Accel_Scale;
    float Accel_Y = AccY * Accel_Scale;
    float Accel_Z = AccZ * Accel_Scale;
    
    // 使用加速度计计算 Pitch和Roll
    float Accel_Pitch = atan2f(Accel_X, sqrtf(Accel_Y * Accel_Y + Accel_Z * Accel_Z)) * 57.295779f; // 弧度转角度
    float Accel_Roll = atan2f(Accel_Y, Accel_Z) * 57.295779f; // 弧度转角度
    
    // 2.处理陀螺仪数据
    float Gyro_X = GyroX * Gyro_Scale;  // 转换为 °/s
    float Gyro_Y = GyroY * Gyro_Scale;
    float Gyro_Z = GyroZ * Gyro_Scale;
    
    // 3.首次运行时的初始化
    if (Is_First)
    {
        *Pitch = Accel_Pitch;
        *Roll = Accel_Roll;
        *Yaw = 0;  // 初始偏航角设为0
        
        Last_Pitch = *Pitch;
        Last_Roll = *Roll;
        Last_Yaw = *Yaw;
        
        Is_First = 0;
        return;
    }
    
    // 4.互补滤波计算Pitch和Roll
    *Pitch = Alpha * (Last_Pitch + Gyro_X * Dt) + (1 - Alpha) * Accel_Pitch;
    *Roll = Alpha * (Last_Roll + Gyro_Y * Dt) + (1 - Alpha) * Accel_Roll;
    
    // 5.根据姿态角，补偿陀螺仪数据的坐标系转换（将陀螺仪数据从传感器坐标系转到地理坐标系）
    float Gyro_Z_Corrected = Gyro_Z + sinf(Last_Roll * 0.0174533f) * Gyro_X + 
                            cosf(Last_Roll * 0.0174533f) * sinf(Last_Pitch * 0.0174533f) * Gyro_Y;
    
    // 6.计算陀螺仪积分的偏航角
    float Gyro_Yaw = Last_Yaw + Gyro_Z_Corrected * Dt;
    
    // 7.处理磁力计数据，进行倾角补偿后计算偏航角
    float pitch_rad = (*Pitch) * 0.0174533f; // 角度转弧度
    float roll_rad = (*Roll) * 0.0174533f;   // 角度转弧度
    
    // 磁力计数据归一化（实际应用中可能需要先进行磁力计校准）
    float norm = sqrtf(MagX*MagX + MagY*MagY + MagZ*MagZ);
    if (norm == 0) {
        // 如果磁力计数据无效，只使用陀螺仪数据
        *Yaw = Gyro_Yaw;
    } else {
        float mx = MagX / norm;
        float my = MagY / norm;
        float mz = MagZ / norm;
        
        // 应用倾角补偿，将磁力计数据从传感器坐标系转换到水平坐标系
        float cos_roll = cosf(roll_rad);
        float sin_roll = sinf(roll_rad);
        float cos_pitch = cosf(pitch_rad);
        float sin_pitch = sinf(pitch_rad);
        
        // 补偿计算 - 转换磁力计数据到水平面
        float Xh = mx * cos_pitch + mz * sin_pitch;
        float Yh = mx * sin_roll * sin_pitch + my * cos_roll - mz * sin_roll * cos_pitch;
        
        // 计算偏航角（磁北方向）
        float mag_yaw = atan2f(Yh, Xh) * 57.295779f;  // 弧度转角度
        
        // 确保结果在0-359.99度范围内
        while(mag_yaw < 0)
            mag_yaw += 360.0f;
        while(mag_yaw >= 360.0f)
            mag_yaw -= 360.0f;
        
        // 8.融合陀螺仪和磁力计的偏航角
        // 由于磁力计受外界干扰大，给予陀螺仪积分更高权重
        *Yaw = Mag_Alpha * Gyro_Yaw + (1.0f - Mag_Alpha) * mag_yaw;
    }
    
    // 保持偏航角在0-359.99度范围内
    while(*Yaw < 0)
        *Yaw += 360.0f;
    while(*Yaw >= 360.0f)
        *Yaw -= 360.0f;
    
    // 保存当前角度作为下次计算的历史值
    Last_Pitch = *Pitch;
    Last_Roll = *Roll;
    Last_Yaw = *Yaw;
}
/**
 * 函    数：根据欧拉角和加速度计数据计算水平加速度
 * 参    数：AccX, AccY, AccZ - 加速度计原始数据 (-32768~32767)
 * 参    数：Pitch, Roll, Yaw - 欧拉角 (单位：度)
 * 参    数：HorizontalAcc_mps2 - 计算得到的水平加速度指针 (单位：m/s^2)
 * 返 回 值：无
 * 说    明：此函数将加速度计读数从传感器坐标系转换到世界坐标系，
 *           然后移除重力分量，最后计算水平面上的加速度大小。
 *           假定世界坐标系Z轴指向上方，重力沿Z轴负方向。
 *           假定欧拉角 Pitch, Roll, Yaw 是基于 Z-Y'-X'' 旋转顺序，
 *           表示从世界坐标系到传感器本体坐标系的旋转。
 */
void MPU6050_GetHorizontalAcceleration(
	int16_t AccX, int16_t AccY, int16_t AccZ,
	float Pitch, float Roll, float Yaw,
	float *HorizontalAcc_mps2)
{
	// 转换系数和常量
	// 满量程 ±16g 对应 1g = 2048 LSB
	const float ACCEL_SCALE_G_PER_LSB = 1.0f / 2048.0f; // 每LSB对应的g值
	const float G_Value = 9.8048f;                    // 标准重力加速度 m/s^2 (使用更精确的值)
	const float DegToRad = 0.017453292519943295f;      // 度转弧度系数 (使用更精确的值)

	// 1. 将原始加速度数据转换为物理单位 (g)
	float ax_g = (float)AccX * ACCEL_SCALE_G_PER_LSB;
	float ay_g = (float)AccY * ACCEL_SCALE_G_PER_LSB;
	float az_g = (float)AccZ * ACCEL_SCALE_G_PER_LSB;

	// 2. 将欧拉角从度转换为弧度
	float pitch_rad = Pitch * DegToRad;
	float roll_rad = Roll * DegToRad;
	float yaw_rad = Yaw * DegToRad;

	// 3. 计算构建旋转矩阵所需的三角函数值
	float cos_yaw = cosf(yaw_rad);
	float sin_yaw = sinf(yaw_rad);
	float cos_pitch = cosf(pitch_rad);
	float sin_pitch = sinf(pitch_rad);
	float cos_roll = cosf(roll_rad);
	float sin_roll = sinf(roll_rad);

	// 4. 将传感器测量的总加速度转换到世界坐标系 (单位: g)
	// Acc_world = R_transpose * Acc_sensor
	// R 是从世界到本体，R_transpose 是从本体到世界
	// R_transpose 的元素 R_Tij = Rji
	// R_transpose 的第一行对应 R11, R21, R31 (R矩阵的列向量)
	// R_transpose 的第二行对应 R12, R22, R32
	// R_transpose 的第三行对应 R13, R23, R33

	// 计算世界坐标系下的总加速度X分量
	float acc_world_x_g = cos_pitch * cos_yaw * ax_g +
						  (sin_roll * sin_pitch * cos_yaw - cos_roll * sin_yaw) * ay_g +
						  (cos_roll * sin_pitch * cos_yaw + sin_roll * sin_yaw) * az_g;

	// 计算世界坐标系下的总加速度Y分量
	float acc_world_y_g = cos_pitch * sin_yaw * ax_g +
						  (sin_roll * sin_pitch * sin_yaw + cos_roll * cos_yaw) * ay_g +
						  (cos_roll * sin_pitch * sin_yaw - sin_roll * cos_yaw) * az_g;

	// 计算世界坐标系下的总加速度Z分量 (仅需要 Z 分量来移除重力)
	float acc_world_z_g = -sin_pitch * ax_g +
						  sin_roll * cos_pitch * ay_g +
						  cos_roll * cos_pitch * az_g;


	// 5. 移除世界坐标系下的重力分量 (假设重力沿世界坐标系的Z轴负方向，大小为1g)
	// 线性加速度 = 总加速度 - 重力加速度
	// 注意：世界坐标系的Z轴通常指向上方，所以重力是 [0, 0, -1]g
	// 这里只移除Z轴分量，因为水平X, Y方向没有重力分量
	float linear_acc_world_x_g = acc_world_x_g;
	float linear_acc_world_y_g = acc_world_y_g;
	// float linear_acc_world_z_g = acc_world_z_g - (-1.0f); // 如果需要Z轴运动加速度再计算

	// 6. 计算水平面上的加速度大小 (X 和 Y 分量的矢量和)
	float horizontal_acc_g = sqrtf(linear_acc_world_x_g * linear_acc_world_x_g +
								 linear_acc_world_y_g * linear_acc_world_y_g);

	// 7. 将结果转换为 m/s^2
	*HorizontalAcc_mps2 = horizontal_acc_g * G_Value;
}
