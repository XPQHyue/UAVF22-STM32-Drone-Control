/*
 * pid_variable_integral.h
 *
 *  Created on: Nov 9, 2025
 *      Author: Administrator
 */

#ifndef INC_PID_VARIABLE_INTEGRAL_H_
#define INC_PID_VARIABLE_INTEGRAL_H_

#include "pid_study.h"

// 变速积分PID状态结构体
typedef struct {

    PID_State base;        // 基础PID状态
    double max_error;      // 最大误差参考值
    double min_ki_factor;  // 最小Ki系数
    double max_ki_factor;  // 最大Ki系数

} PID_VariableIntegral_State;

// 函数声明
void PID_VariableIntegral_Init(PID_VariableIntegral_State *state, double target,
                              double output_max, double output_min, double max_error,
                              double min_ki_factor, double max_ki_factor);

PID_VariableIntegral_State PID_VariableIntegral_Iterate(PID_KpidGain KpidGain,
                                                       PID_VariableIntegral_State state);


#endif /* INC_PID_VARIABLE_INTEGRAL_H_ */
