/*
 * pid_study.c
 *
 *  Created on: Oct 7, 2025
 *      Author: XPQH
 *      PID学习经过
 *		PID_V1.0函数 只有基本的PID数学运算 2025.10.8
 */

#include "pid_study.h"

//初始化PID函数
void PID_Init(PID_State *state, double target){

	//等价于(*state).target  在主函数预设目标值之后写入PID_State结构体
	state->target		= target;
	state->actual		= 0.0;

	state->error		= 0.0;
	state->prev_error	= 0.0;
	state->integral		= 0.0;
	state->derivative	= 0.0;
	state->output		= 0.0;
	state->time_delta	= 0.01;

	//增量PID
	state->prev_output	= 0.0;
	state->Delta_Value	= 0.0;
	state->current_output = 0.0;

}


//执行PID迭代(Iterate)运算函数
PID_State PID_Iterate(PID_KpidGain KpidGain, PID_State state){

	state.prev_error = state.error;
	//0-(-30)
	state.error = state.target - state.actual;			//P

	state.integral += (state.error * state.time_delta);	//I
	/*
	 *
	 * 积分限幅 抗饱和等优化预留区
	 *
	 *
	 * */												//D
	state.derivative = (state.error - state.prev_error) / state.time_delta;



#if 0
	//增量PID 2025.10.17
	state.prev_output = state.current_output;

	state.current_output = (KpidGain.Kp * state.error)
						 + (KpidGain.Ki * state.integral)
						 + (KpidGain.Kd * state.derivative);

	state.Delta_Value = state.current_output - state.prev_output;

	//数学表达式u(t) = Kp*e(t) + Ki*∫e(t)dt + Kd*de(t)/dt)
	state.output = state.current_output + state.Delta_Value;

#endif

#if 1

	state.output = (KpidGain.Kp * state.error)
								 + (KpidGain.Ki * state.integral)
								 + (KpidGain.Kd * state.derivative);

	if(state.output > 200) state.output = 200;  //输出限幅
	if(state.output < -200) state.output = -200;
#endif
	return state;
}
