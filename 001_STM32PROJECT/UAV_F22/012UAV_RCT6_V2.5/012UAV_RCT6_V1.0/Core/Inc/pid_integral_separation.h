/*
 * pid_integral_separation.h
 *
 *  Created on: Nov 9, 2025
 *      Author: Administrator
 */

#ifndef INC_PID_INTEGRAL_SEPARATION_H_
#define INC_PID_INTEGRAL_SEPARATION_H_

#include "pid_study.h"
// 积分分离PID状态结构体
typedef struct {

    PID_State base;           // 基础PID状态
    double integral_threshold; // 积分分离阈值
    int integral_enabled;     // 积分使能标志

} PID_IntegralSeparation_State;


// 函数声明
void PID_IntegralSeparation_Init(PID_IntegralSeparation_State *state, double target,
                                double output_max, double output_min, double integral_threshold);

PID_IntegralSeparation_State PID_IntegralSeparation_Iterate(PID_KpidGain KpidGain,
                                                           PID_IntegralSeparation_State state);

#endif /* INC_PID_INTEGRAL_SEPARATION_H_ */
