/*
 * pid_study.h
 *
 *  Created on: Oct 7, 2025
 *      Author: XPQH
 *      PID_V1.0函数 只有基本的PID数学运算 2025.10.8
 *      PID_V1.2新增直接在初始化时设定限幅大小2025.11.8
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
	double output_max;		//输出上限（新增限幅参数）
	double output_min;		//输出下限（新增限幅参数）

}	PID_State;

//初始化增量PID函数定义 初始给定目标值和限幅参数
void PID_Init(PID_State *state, double target, double output_max, double output_min);


//执行PID迭代(Iterate)函数定义
PID_State PID_Iterate(PID_KpidGain KpidGain, PID_State state);

#endif /* INC_PID_STUDY_H_ */
