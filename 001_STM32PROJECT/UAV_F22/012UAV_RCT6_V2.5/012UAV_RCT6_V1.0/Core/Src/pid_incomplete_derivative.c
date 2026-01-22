/*
 * pid_incomplete_derivative.c
 *
 *  Created on: Nov 9, 2025
 *      Author: Administrator
 */

#include "pid_incomplete_derivative.h"

// 初始化不完全微分PID函数
void PID_IncompleteDerivative_Init(PID_IncompleteDerivative_State *state, double target,
                                  double output_max, double output_min, double derivative_alpha) {
    // 初始化基础PID
    PID_Init(&state->base, target, output_max, output_min);

    // 设置不完全微分参数
    state->derivative_alpha = derivative_alpha;
    state->filtered_derivative = 0.0;
}

// 执行不完全微分PID迭代运算函数
PID_IncompleteDerivative_State PID_IncompleteDerivative_Iterate(PID_KpidGain KpidGain,
                                                               PID_IncompleteDerivative_State state) {
    state.base.prev_error = state.base.error;
    state.base.error = state.base.target - state.base.actual;

    state.base.integral += (state.base.error * state.base.time_delta);

    // 不完全微分逻辑：使用一阶低通滤波器平滑微分项
    double raw_derivative = (state.base.error - state.base.prev_error) / state.base.time_delta;
    state.filtered_derivative = state.derivative_alpha * state.filtered_derivative +
                               (1 - state.derivative_alpha) * raw_derivative;

    state.base.derivative = state.filtered_derivative;

    // PID输出计算
    state.base.output = (KpidGain.Kp * state.base.error) +
                       (KpidGain.Ki * state.base.integral) +
                       (KpidGain.Kd * state.base.derivative);

    // 输出限幅
    if (state.base.output > state.base.output_max) state.base.output = state.base.output_max;
    if (state.base.output < state.base.output_min) state.base.output = state.base.output_min;

    return state;
}

