/*
 * pid_study.c
 *
 *  Created on: Oct 7, 2025
 *      Author: XPQH
 *      PID学习经过
 *		PID_V1.0函数 只有基本的PID数学运算 2025.10.8
 *		最简单最原始的PID,没有任何优化措施,PID控制精简而强大2025.11.3
 *		PID_V1.2新增直接在初始化时设定限幅大小2025.11.8
 */

#include "pid_study.h"

//初始化PID函数
void PID_Init(PID_State *state, double target, double output_max, double output_min){

	//等价于(*state).target  在主函数预设目标值之后写入PID_State结构体
	state->target		= target;
	state->actual		= 0.0;

	state->error		= 0.0;
	state->prev_error	= 0.0;
	state->integral		= 0.0;
	state->derivative	= 0.0;
	state->output		= 0.0;

	state->time_delta	= 0.01;

	//新增：设置输出限幅参数
	state->output_max	= output_max;
	state->output_min	= output_min;

}


//执行PID迭代(Iterate)运算函数
PID_State PID_Iterate(PID_KpidGain KpidGain, PID_State state){

	state.prev_error = state.error;

	//0-(-30)
	state.error = state.target - state.actual;								//P

	state.integral += (state.error * state.time_delta);						//I

	state.derivative = (state.error - state.prev_error) / state.time_delta;	//D
	//不难看出state.time_delta，也就是时间dt其实可有可无，因为是常数
	//如果PID的执行放在中断里面，中断每10ms启动一次，那么我们可以认为dt=10ms，时间间隔dt尽量保持固定长度(为什么，就留给自己想吧（￣︶￣）↗　)

	//数学表达式u(t) = Kp*e(t) + Ki*∫e(t)dt + Kd*de(t)/dt)
	state.output =  (KpidGain.Kp * state.error)
								 + (KpidGain.Ki * state.integral)
								 + (KpidGain.Kd * state.derivative);

	//使用在main函数中设置的限幅参数进行限幅处理
	if(state.output > state.output_max) state.output = state.output_max;
	if(state.output < state.output_min) state.output = state.output_min;


	return state;
}
