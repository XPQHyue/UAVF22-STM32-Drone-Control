/*
 * kalman.c
 *
 *  Created on: Oct 3, 2025
 *      Author: Administrator
 */
#include "kalman.h"

// 初始化卡尔曼滤波器
void Kalman_Init(KalmanFilter *kf, float q, float r, float initial_value, float initial_p) {
    kf->q = q;
    kf->r = r;
    kf->x = initial_value;
    kf->p = initial_p;
    kf->k = 0;
}

// 卡尔曼滤波更新
float Kalman_Update(KalmanFilter *kf, float measurement) {
    // 预测步骤
    kf->p = kf->p + kf->q;

    // 更新步骤
    kf->k = kf->p / (kf->p + kf->r);
    kf->x = kf->x + kf->k * (measurement - kf->x);
    kf->p = (1 - kf->k) * kf->p;

    return kf->x;
}
