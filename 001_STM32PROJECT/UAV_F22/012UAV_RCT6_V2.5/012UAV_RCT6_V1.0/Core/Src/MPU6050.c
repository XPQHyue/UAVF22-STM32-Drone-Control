/*
 * MPU6050.c
 *
 *  Created on: Aug 25, 2024
 *      Author: 王滋行
 *      注意 需要原装6050 不能是国产替代 否则无法开启DMP
 *      如何验证国产还是原装 	如果6050芯片丝印是2348且比较小(购买时对比不同6050可以清晰看出大小) 那么便是国产
 *      在此感谢王滋行up的移植和视频教学， 成功运行 ----XPQH 2025.10.7
 *      UP 视频链接 MPU6050DMP库向STM32HAL库的移植
 *      https://www.bilibili.com/video/BV1KaHvesEza/?share_source=copy_web&vd_source=64d39fa8a552803ca31a3a5c0f92bfe
 */

#include "MPU6050.h"
#include "inv_mpu.h"
#include "inv_mpu_dmp_motion_driver.h"
#include "math.h"

/* The sensors can be mounted onto the board in any orientation. The mounting
 * matrix seen below tells the MPL how to rotate the raw data from thei
 * driver(s).
 * TODO: The following matrices refer to the configuration on an internal test
 * board at Invensense. If needed, please modify the matrices to match the
 * chip-to-body matrix for your particular set up.
 */
static signed char gyro_orientation[9] = {-1, 0, 0,
                                           0,-1, 0,
                                           0, 0, 1};

/* These next two functions converts the orientation matrix (see
 * gyro_orientation) to a scalar representation for use by the DMP.
 * NOTE: These functions are borrowed from Invensense's MPL.
 */
static unsigned short inv_row_2_scale(const signed char *row)
{
    unsigned short b;

    if (row[0] > 0)
        b = 0;
    else if (row[0] < 0)
        b = 4;
    else if (row[1] > 0)
        b = 1;
    else if (row[1] < 0)
        b = 5;
    else if (row[2] > 0)
        b = 2;
    else if (row[2] < 0)
        b = 6;
    else
        b = 7;      // error
    return b;
}

static unsigned short inv_orientation_matrix_to_scalar(
    const signed char *mtx)
{
    unsigned short scalar;

    /*
       XYZ  010_001_000 Identity Matrix
       XZY  001_010_000
       YXZ  010_000_001
       YZX  000_010_001
       ZXY  001_000_010
       ZYX  000_001_010
     */

    scalar = inv_row_2_scale(mtx);
    scalar |= inv_row_2_scale(mtx + 3) << 3;
    scalar |= inv_row_2_scale(mtx + 6) << 6;


    return scalar;
}

static int run_self_test(void)
{
    int result;
    long gyro[3], accel[3];

    result = mpu_run_self_test(gyro, accel);
    if (result == 0x3) {
        /* Test passed. We can trust the gyro data here, so let's push it down
         * to the DMP.
         */
        float sens;
        unsigned short accel_sens;
        mpu_get_gyro_sens(&sens);
        gyro[0] = (long)(gyro[0] * sens);
        gyro[1] = (long)(gyro[1] * sens);
        gyro[2] = (long)(gyro[2] * sens);
        dmp_set_gyro_bias(gyro);
        mpu_get_accel_sens(&accel_sens);
        accel[0] *= accel_sens;
        accel[1] *= accel_sens;
        accel[2] *= accel_sens;
        dmp_set_accel_bias(accel);
    } else {
        return -1;
    }

    return 0;
}

int MPU6050_DMP_init(void)
{
    int ret;
    struct int_param_s int_param;
    //mpu_init
    ret = mpu_init(&int_param);
    if(ret != 0)
    {
        return ERROR_MPU_INIT;
    }
    //设置传感器
    ret = mpu_set_sensors(INV_XYZ_GYRO | INV_XYZ_ACCEL);
    if(ret != 0)
    {
        return ERROR_SET_SENSOR;
    }
    //设置fifo
    ret = mpu_configure_fifo(INV_XYZ_GYRO | INV_XYZ_ACCEL);
    if(ret != 0)
    {
        return ERROR_CONFIG_FIFO;
    }
    //设置采样率
    ret = mpu_set_sample_rate(DEFAULT_MPU_HZ);
    if(ret != 0)
    {
        return ERROR_SET_RATE;
    }
    //加载DMP固件
    ret = dmp_load_motion_driver_firmware();
    if(ret != 0)
    {
        return ERROR_LOAD_MOTION_DRIVER;
    }
    //设置陀螺仪方向
    ret = dmp_set_orientation(inv_orientation_matrix_to_scalar(gyro_orientation));
    if(ret != 0)
    {
        return ERROR_SET_ORIENTATION;
    }
    //设置DMP功能
    ret = dmp_enable_feature(DMP_FEATURE_6X_LP_QUAT | DMP_FEATURE_TAP |
            DMP_FEATURE_ANDROID_ORIENT | DMP_FEATURE_SEND_RAW_ACCEL |
            DMP_FEATURE_SEND_CAL_GYRO | DMP_FEATURE_GYRO_CAL);
    if(ret != 0)
    {
        return ERROR_ENABLE_FEATURE;
    }
    //设置输出速率
    ret = dmp_set_fifo_rate(DEFAULT_MPU_HZ);
    if(ret != 0)
    {
        return ERROR_SET_FIFO_RATE;
    }
    //自检
    ret = run_self_test();
    if(ret != 0)
    {
        return ERROR_SELF_TEST;
    }
    //使能DMP
    ret = mpu_set_dmp_state(1);
    if(ret != 0)
    {
        return ERROR_DMP_STATE;
    }

    //新配置 滤波设置 XPQH 2025.10.9
    ret = mpu_set_lpf(98);
    if(ret != 0){

    	return ERROR_DMP_STATE;

    }

    return 0;
}

int MPU6050_DMP_Get_Date(float *pitch, float *roll, float *yaw)
{
    float q0 = 1.0f, q1 = 0.0f, q2 = 0.0f, q3 = 0.0f;
    short gyro[3];
    short accel[3];
    long quat[4];
    unsigned long timestamp;
    short sensors;
    unsigned char more;
    if(dmp_read_fifo(gyro, accel, quat, &timestamp, &sensors, &more))
    {
        return -1;
    }

    if(sensors & INV_WXYZ_QUAT)
    {
        q0 = quat[0] / Q30;
        q1 = quat[1] / Q30;
        q2 = quat[2] / Q30;
        q3 = quat[3] / Q30;

        *pitch = asin(-2 * q1 * q3 + 2 * q0 * q2) * 57.3; // pitch
        *roll = atan2(2 * q2 * q3 + 2 * q0 * q1, -2 * q1 * q1 - 2 * q2 * q2 + 1) * 57.3; // roll
        *yaw = atan2(2 * (q0 * q3 + q1 * q2), q0 * q0 + q1 * q1 - q2 * q2 - q3 * q3) * 57.3; // yaw
    }

    return 0;
}

//新增其他数据显示函数 		XPQH_2025.10.9
//获取角速度数据 单位：度
int MPU6050_Get_Gyro( volatile float *gyro_x,  volatile float *gyro_y,  volatile float *gyro_z)
{

    short raw_gyro[3];
    unsigned long timestamp;
    float gyro_sens;

    // 读取原始陀螺仪数据
    if (mpu_get_gyro_reg(raw_gyro, &timestamp) != 0) {
        return -1;
    }

    // 获取陀螺仪灵敏度
    if (mpu_get_gyro_sens(&gyro_sens) != 0) {
        return -1;
    }

    // 转换为角速度（单位：dps）
    *gyro_x = raw_gyro[0] / gyro_sens;
    *gyro_y = raw_gyro[1] / gyro_sens;
    *gyro_z = raw_gyro[2] / gyro_sens;

	return 0;
}

//获取加速度数据（单位：g）
int MPU6050_Get_Accel(volatile float *accel_x, volatile float *accel_y, volatile float *accel_z)
{
    short raw_accel[3];
    unsigned long timestamp;
    unsigned short accel_sens;

    // 读取原始加速度数据
    if (mpu_get_accel_reg(raw_accel, &timestamp) != 0) {
        return -1;
    }

    // 获取加速度计灵敏度
    if (mpu_get_accel_sens(&accel_sens) != 0) {
        return -1;
    }

    // 转换为加速度（单位：g）
    *accel_x = (float)raw_accel[0] / accel_sens;
    *accel_y = (float)raw_accel[1] / accel_sens;
    *accel_z = (float)raw_accel[2] / accel_sens;

    return 0;
}

//获取原始角速度数据（硬件单位）
int MPU6050_Get_RawGyro(short *gyro_x, short *gyro_y, short *gyro_z)
{
    short raw_gyro[3];
    unsigned long timestamp;

    // 读取原始陀螺仪数据
    if (mpu_get_gyro_reg(raw_gyro, &timestamp) != 0) {
        return -1;
    }

    *gyro_x = raw_gyro[0];
    *gyro_y = raw_gyro[1];
    *gyro_z = raw_gyro[2];

    return 0;
}

//获取原始加速度数据（硬件单位）
int MPU6050_Get_RawAccel(short *accel_x, short *accel_y, short *accel_z)
{
    short raw_accel[3];
    unsigned long timestamp;

    // 读取原始加速度数据
    if (mpu_get_accel_reg(raw_accel, &timestamp) != 0) {
        return -1;
    }

    *accel_x = raw_accel[0];
    *accel_y = raw_accel[1];
    *accel_z = raw_accel[2];

    return 0;
}

//获取温度数据（单位：摄氏度）
int MPU6050_Get_Temperature(float *temperature)
{
    long temp_data;
    unsigned long timestamp;

    // 读取温度数据
    if (mpu_get_temperature(&temp_data, &timestamp) != 0) {
        return -1;
    }

    // 转换为摄氏度
    // 温度数据是Q16格式，需要除以65536得到实际温度值
    *temperature = (float)temp_data / 65536.0f;

    return 0;
}

//传感器量程设置函数
// 获取当前加速度计量程和灵敏度信息
int MPU6050_Get_Accel_Info(void)
{
    unsigned char accel_fsr;
    unsigned short accel_sens;

    if (mpu_get_accel_fsr(&accel_fsr) != 0) {
        return -1;
    }

    if (mpu_get_accel_sens(&accel_sens) != 0) {
        return -1;
    }

    // 这里可以根据需要显示信息
    // 例如通过OLED显示或串口打印
   /* printf("Accel FSR: %dg, Sensitivity: %d LSB/g\n", accel_fsr, accel_sens);*/

    return 0;
}

// 设置加速度计量程
int MPU6050_Set_Accel_FSR(unsigned char fsr)
{
    // 验证输入参数 要是填写的不是这些数据 数据就无用
    if (fsr != 2 && fsr != 4 && fsr != 8 && fsr != 16) {
        return -1;
    }

    if (mpu_set_accel_fsr(fsr) != 0) {
        return -1;
    }

    return 0;
}

// 设置陀螺仪量程
int MPU6050_Set_Gyro_FSR(unsigned short fsr)
{
    // 支持的陀螺仪量程：250, 500, 1000, 2000 (单位：dps)
    if (fsr != 250 && fsr != 500 && fsr != 1000 && fsr != 2000) {
        return -1;
    }

    if (mpu_set_gyro_fsr(fsr) != 0) {
        return -1;
    }

    return 0;
}






