/*
 * MPU6050.h
 *
 *  Created on: Aug 25, 2024
 *      Author: 王滋行
 *
 *		注意 需要原装6050 不能是国产替代 否则无法开启DMP
 *      如何验证国产还是原装 	如果6050芯片丝印是2348且比较小(购买时对比不同6050可以清晰看出大小) 那么便是国产
 *      在此感谢王滋行up的移植和视频教学， 成功运行 ----XPQH 2025.10.7
 *      UP 视频链接 MPU6050DMP库向STM32HAL库的移植
 *      https://www.bilibili.com/video/BV1KaHvesEza/?share_source=copy_web&vd_source=64d39fa8a552803ca31a3a5c0f92bfe
*/
#ifndef INC_MPU6050_H_
#define INC_MPU6050_H_

#define ERROR_MPU_INIT      -1
#define ERROR_SET_SENSOR    -2
#define ERROR_CONFIG_FIFO   -3
#define ERROR_SET_RATE      -4
#define ERROR_LOAD_MOTION_DRIVER    -5
#define ERROR_SET_ORIENTATION       -6
#define ERROR_ENABLE_FEATURE        -7
#define ERROR_SET_FIFO_RATE         -8
#define ERROR_SELF_TEST             -9
#define ERROR_DMP_STATE             -10

//新增角速度加速度 错误码					XPQH_2025.10.9
#define ERROR_SET_ACCEL_FSR        -11
#define ERROR_SET_GYRO_FSR         -12

#define DEFAULT_MPU_HZ  100
#define Q30  1073741824.0f

int MPU6050_DMP_init(void);
int MPU6050_DMP_Get_Date(float *pitch, float *roll, float *yaw);


//新增其他数据显示函数定义 					XPQH_2025.10.9
int MPU6050_Get_Gyro(float *gyro_x, float *gyro_y, float *gyro_z);			//获取角速度数据（单位：dps）
int MPU6050_Get_Accel(float *accel_x, float *accel_y, float *accel_z);		//获取加速度数据（单位：g）
int MPU6050_Get_RawGyro(short *gyro_x, short *gyro_y, short *gyro_z);		//获取原始角速度数据（硬件单位）
int MPU6050_Get_RawAccel(short *accel_x, short *accel_y, short *accel_z);	//获取原始加速度数据（硬件单位）
int MPU6050_Get_Temperature(float *temperature);							//获取温度数据（单位：摄氏度）

// 新增传感器量程设置函数定义				XPQH_2025.10.9
int MPU6050_Get_Accel_Info(void);
int MPU6050_Set_Accel_FSR(unsigned char fsr);
int MPU6050_Set_Gyro_FSR(unsigned short fsr);
#endif /* INC_MPU6050_H_ */
