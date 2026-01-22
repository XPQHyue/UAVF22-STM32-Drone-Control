/*
 * pid_study.h
 *
 *  Created on: Oct 7, 2025
 *      Author: XPQH
 *      PID_V1.0函数 只有基本的PID数学运算 2025.10.8
 */

#ifndef INC_PID_STUDY_H_
#define INC_PID_STUDY_H_

//PID系数
typedef struct{

	double Kp;	//比例增益
	double Ki;	//积分
	double Kd;	//微分

}	PID_KpidGain;


//PID状态结构体
typedef struct{

	double target;			//设定目标
	double actual;			//实际目标
	double error;			//当前误差
	double prev_error;		//上次_误差
	double integral;		//积分项
	double derivative;		//微分项
	double output;			//PID最终输出
	double time_delta;		//时间dt

	//增量PID
	double prev_output;		//上次_输出
	double Delta_Value;		//增量值
	double current_output;	//当前输出

}	PID_State;

//初始化PID函数定义 初始给定目标值 dt长度
void PID_Init(PID_State *state, double target);


//执行PID迭代(Iterate)函数定义
PID_State PID_Iterate(PID_KpidGain KpidGain, PID_State state);

#endif /* INC_PID_STUDY_H_ */
