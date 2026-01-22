/*
 * pid_integral_separation.c
 *
 *  Created on: Nov 9, 2025
 *      Author: Administrator
 */
#include "pid_integral_separation.h"
#include <math.h>

// 初始化积分分离PID函数
void PID_IntegralSeparation_Init(PID_IntegralSeparation_State *state, double target,
                                double output_max, double output_min, double integral_threshold) {
    // 初始化基础PID
    PID_Init(&state->base, target, output_max, output_min);

    // 设置积分分离参数
    state->integral_threshold = integral_threshold;
    state->integral_enabled = 1; // 默认使能积分

}

// 执行积分分离PID迭代运算函数
PID_IntegralSeparation_State PID_IntegralSeparation_Iterate(PID_KpidGain KpidGain,
                                                           PID_IntegralSeparation_State state) {
    state.base.prev_error = state.base.error;
    state.base.error = state.base.target - state.base.actual;

    // 积分分离逻辑：当误差超过阈值时禁用积分
    if (fabs(state.base.error) > state.integral_threshold) {
        state.integral_enabled = 0;
        state.base.integral = 0; // 可选：重置积分项
    } else {
        state.integral_enabled = 1;
    }
    // 积分项计算（仅在使能时累加）
    if (state.integral_enabled) {
        state.base.integral += (state.base.error * state.base.time_delta);
    }

    state.base.derivative = (state.base.error - state.base.prev_error) / state.base.time_delta;
    // PID输出计算
    state.base.output = (KpidGain.Kp * state.base.error) +
                       (KpidGain.Ki * state.base.integral) +
                       (KpidGain.Kd * state.base.derivative);
    // 输出限幅
    if (state.base.output > state.base.output_max) state.base.output = state.base.output_max;
    if (state.base.output < state.base.output_min) state.base.output = state.base.output_min;
    return state;

}
