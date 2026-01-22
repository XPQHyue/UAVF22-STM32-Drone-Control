/*
 * pid_incremental.c
 *
 *  Created on: Nov 8, 2025
 *      Author: XPQH
 * 增量	incremental
 * 说明：增量式PID，和普通PID基本一致，但是新增一个文件更加灵活,注释在pid_study很多了，此处省略部分
 * 增量式PID公式：Δu(k) = Kp*[e(k)-e(k-1)] + Ki*e(k) + Kd*[e(k)-2e(k-1)+e(k-2)]
 */
#include "pid_incremental.h"

//初始化增量PID函数
void PID_Inc_Init(PID_State_Inc *state, double target, double output_max, double output_min){

	//等价于(*state).target  在主函数预设目标值之后写入PID_State结构体
	state->target		= target;
	state->actual		= 0.0;

	state->error		= 0.0;
	state->prev_error	= 0.0;
	state->prev_prev_error = 0.0;	//新增：上上次误差
	state->integral		= 0.0;
	state->derivative	= 0.0;
	state->output		= 0.0;
	state->prev_output	= 0.0;		//新增：上次输出

	state->time_delta	= 0.01;

	//新增：设置输出限幅参数
	state->output_max	= output_max;
	state->output_min	= output_min;

}

//执行增量PID迭代(Iterate)运算函数
PID_State_Inc PID_Inc_Iterate(PID_KpidGain_Inc KpidGain, PID_State_Inc state){

	//保存历史误差值
	state.prev_prev_error = state.prev_error;	//上上次误差 = 上次误差
	state.prev_error = state.error;				//上次误差 = 当前误差

	//计算当前误差
	state.error = state.target - state.actual;

	//增量式PID计算
	//Δu(k) = Kp*[e(k)-e(k-1)] + Ki*T*e(k) + (Kd/T)*[e(k)-2e(k-1)+e(k-2)]
	double delta_output = KpidGain.Kp * (state.error - state.prev_error)
						+ KpidGain.Ki * state.time_delta * state.error
						+ (KpidGain.Kd / state.time_delta) * (state.error - 2 * state.prev_error + state.prev_prev_error);

	//计算当前输出 = 上次输出 + 增量输出
	state.prev_output = state.output;	//保存上次输出
	state.output = state.prev_output + delta_output;

	//使用在main函数中设置的限幅参数进行限幅处理
	if(state.output > state.output_max) state.output = state.output_max;
	if(state.output < state.output_min) state.output = state.output_min;

	return state;

}
