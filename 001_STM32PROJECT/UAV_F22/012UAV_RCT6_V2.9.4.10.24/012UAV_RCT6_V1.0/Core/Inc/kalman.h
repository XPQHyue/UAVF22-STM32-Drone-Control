/*
 * kalman.h
 *
 *  Created on: Oct 3, 2025
 *      Author: Administrator
 */

#ifndef INC_KALMAN_H_
#define INC_KALMAN_H_

// 卡尔曼滤波器结构体
typedef struct {
    float q; // 过程噪声协方差
    float r; // 测量噪声协方差
    float x; // 状态估计值
    float p; // 估计误差协方差
    float k; // 卡尔曼增益
} KalmanFilter;

// 初始化卡尔曼滤波器
void Kalman_Init(KalmanFilter *kf, float q, float r, float initial_value, float initial_p);

// 卡尔曼滤波更新
float Kalman_Update(KalmanFilter *kf, float measurement);

#endif /* INC_KALMAN_H_ */
