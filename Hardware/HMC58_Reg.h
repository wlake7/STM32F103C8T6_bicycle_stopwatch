#ifndef HMC5883L_REG_H
#define HMC5883L_REG_H

// HMC5883L 7位 I2C 地址
#define HMC5883L_I2C_ADDR_7BIT     0x1E

// HMC5883L 8位 I2C 写地址 (根据你的底层函数需要)
#define HMC5883L_ADDRESS          0x3C // (0x1E << 1 | 0)

// HMC5883L 寄存器地址定义
#define HMC5883L_CONFIG_REG_A      0x00  // 配置寄存器 A (平均次数, 数据速率, 测量模式)
#define HMC5883L_CONFIG_REG_B      0x01  // 配置寄存器 B (增益/量程)
#define HMC5883L_MODE_REG          0x02  // 模式寄存器 (工作模式)

#define HMC5883L_DATA_OUT_X_MSB    0x03  // 数据输出 X MSB
#define HMC5883L_DATA_OUT_X_LSB    0x04  // 数据输出 X LSB
#define HMC5883L_DATA_OUT_Z_MSB    0x05  // 数据输出 Z MSB (注意顺序!)
#define HMC5883L_DATA_OUT_Z_LSB    0x06  // 数据输出 Z LSB
#define HMC5883L_DATA_OUT_Y_MSB    0x07  // 数据输出 Y MSB
#define HMC5883L_DATA_OUT_Y_LSB    0x08  // 数据输出 Y LSB

#define HMC5883L_STATUS_REG        0x09  // 状态寄存器 (锁定, 数据就绪)

#define HMC5883L_ID_REG_A          0x0A  // 识别寄存器 A ('H')
#define HMC5883L_ID_REG_B          0x0B  // 识别寄存器 B ('4')
#define HMC5883L_ID_REG_C          0x0C  // 识别寄存器 C ('3')

/* --- 常用配置值宏定义 (可选) --- */

// 模式寄存器 (HMC5883L_MODE_REG) 值 (MD1-MD0)
#define HMC5883L_MODE_CONTINUOUS   0x00  // 连续测量模式
#define HMC5883L_MODE_SINGLE       0x01  // 单次测量模式 (默认)
#define HMC5883L_MODE_IDLE1        0x02  // 空闲模式
#define HMC5883L_MODE_IDLE2        0x03  // 空闲模式

// 配置寄存器 A (HMC5883L_CONFIG_REG_A) - 采样平均 (MA1-MA0)
#define HMC5883L_AVG_1             0x00  // 1次平均 (0b00 << 5)
#define HMC5883L_AVG_2             0x20  // 2次平均 (0b01 << 5)
#define HMC5883L_AVG_4             0x40  // 4次平均 (0b10 << 5)
#define HMC5883L_AVG_8             0x60  // 8次平均 (默认) (0b11 << 5)

// 配置寄存器 A (HMC5883L_CONFIG_REG_A) - 数据速率 (DO2-DO0)
#define HMC5883L_DATARATE_0_75_HZ  0x00  // 0.75 Hz (0b000 << 2)
#define HMC5883L_DATARATE_1_5_HZ   0x04  // 1.5 Hz (0b001 << 2)
#define HMC5883L_DATARATE_3_HZ     0x08  // 3 Hz (0b010 << 2)
#define HMC5883L_DATARATE_7_5_HZ   0x0C  // 7.5 Hz (0b011 << 2)
#define HMC5883L_DATARATE_15_HZ    0x10  // 15 Hz (默认) (0b100 << 2)
#define HMC5883L_DATARATE_30_HZ    0x14  // 30 Hz (0b101 << 2)
#define HMC5883L_DATARATE_75_HZ    0x18  // 75 Hz (0b110 << 2)
// 160Hz 需要通过单次测量和 DRDY 实现

// 配置寄存器 A (HMC5883L_CONFIG_REG_A) - 测量模式 (MS1-MS0)
#define HMC5883L_MEASURE_NORMAL    0x00  // 正常测量 (默认) (0b00 << 0)
#define HMC5883L_MEASURE_POS_BIAS  0x01  // 正偏置自测 (0b01 << 0)
#define HMC5883L_MEASURE_NEG_BIAS  0x02  // 负偏置自测 (0b10 << 0)

// 配置寄存器 B (HMC5883L_CONFIG_REG_B) - 增益 (GN2-GN0)
#define HMC5883L_GAIN_0_88GA       0x00  // +/- 0.88 Ga (1370 LSB/Ga) (0b000 << 5)
#define HMC5883L_GAIN_1_3GA        0x20  // +/- 1.3 Ga  (1090 LSB/Ga) (默认) (0b001 << 5)
#define HMC5883L_GAIN_1_9GA        0x40  // +/- 1.9 Ga  (820 LSB/Ga) (0b010 << 5)
#define HMC5883L_GAIN_2_5GA        0x60  // +/- 2.5 Ga  (660 LSB/Ga) (0b011 << 5)
#define HMC5883L_GAIN_4_0GA        0x80  // +/- 4.0 Ga  (440 LSB/Ga) (0b100 << 5)
#define HMC5883L_GAIN_4_7GA        0xA0  // +/- 4.7 Ga  (390 LSB/Ga) (0b101 << 5)
#define HMC5883L_GAIN_5_6GA        0xC0  // +/- 5.6 Ga  (330 LSB/Ga) (0b110 << 5)
#define HMC5883L_GAIN_8_1GA        0xE0  // +/- 8.1 Ga  (230 LSB/Ga) (0b111 << 5)

#endif // HMC5883L_REG_H
