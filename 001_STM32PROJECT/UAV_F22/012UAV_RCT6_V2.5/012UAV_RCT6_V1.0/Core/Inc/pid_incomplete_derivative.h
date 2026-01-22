/*
 * pid_incomplete_derivative.h
 *
 *  Created on: Nov 9, 2025
 *      Author: Administrator
 */

#ifndef INC_PID_INCOMPLETE_DERIVATIVE_H_
#define INC_PID_INCOMPLETE_DERIVATIVE_H_

#include "pid_study.h"
// 不完全微分PID状态结构体
typedef struct {

    PID_State base;           // 基础PID状态
    double derivative_alpha;  // 微分滤波系数 (0-1)
    double filtered_derivative; // 滤波后的微分值

} PID_IncompleteDerivative_State;

// 函数声明
void PID_IncompleteDerivative_Init(PID_IncompleteDerivative_State *state, double target,
                                  double output_max, double output_min, double derivative_alpha);

PID_IncompleteDerivative_State PID_IncompleteDerivative_Iterate(PID_KpidGain KpidGain,
                                                               PID_IncompleteDerivative_State state);

#endif /* INC_PID_INCOMPLETE_DERIVATIVE_H_ */
