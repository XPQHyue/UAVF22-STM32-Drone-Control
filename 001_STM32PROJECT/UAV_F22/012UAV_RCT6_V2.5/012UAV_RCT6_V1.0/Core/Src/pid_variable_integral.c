/*
 * pid_variable_integral.c
 *
 *  Created on: Nov 9, 2025
 *      Author: Administrator
 */

#include "pid_variable_integral.h"
#include <math.h>

// 初始化变速积分PID函数
void PID_VariableIntegral_Init(PID_VariableIntegral_State *state, double target,
                              double output_max, double output_min, double max_error,
                              double min_ki_factor, double max_ki_factor) {
    // 初始化基础PID
    PID_Init(&state->base, target, output_max, output_min);

    // 设置变速积分参数
    state->max_error = max_error;
    state->min_ki_factor = min_ki_factor;
    state->max_ki_factor = max_ki_factor;
}

// 执行变速积分PID迭代运算函数
PID_VariableIntegral_State PID_VariableIntegral_Iterate(PID_KpidGain KpidGain,
                                                       PID_VariableIntegral_State state) {
    state.base.prev_error = state.base.error;
    state.base.error = state.base.target - state.base.actual;

    // 变速积分逻辑：根据误差大小调整积分系数
    double error_ratio = fabs(state.base.error) / state.max_error;
    if (error_ratio > 1.0) error_ratio = 1.0; // 限制在0-1范围内

    // 计算自适应Ki系数（误差大时Ki小，误差小时Ki大）
    double adaptive_ki = state.min_ki_factor +
                        (state.max_ki_factor - state.min_ki_factor) * (1.0 - error_ratio);

    state.base.integral += (state.base.error * state.base.time_delta);
    state.base.derivative = (state.base.error - state.base.prev_error) / state.base.time_delta;

    // PID输出计算（使用自适应Ki）
    state.base.output = (KpidGain.Kp * state.base.error) +
                       (KpidGain.Ki * adaptive_ki * state.base.integral) +
                       (KpidGain.Kd * state.base.derivative);

    // 输出限幅
    if (state.base.output > state.base.output_max) state.base.output = state.base.output_max;
    if (state.base.output < state.base.output_min) state.base.output = state.base.output_min;

    return state;
}

