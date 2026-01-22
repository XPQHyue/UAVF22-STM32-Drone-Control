/*
 * pid_incremental.h
 *
 *  Created on: Nov 8, 2025
 *      Author: XPQH
 * 增量	incremental
 * 说明：增量式PID，和普通PID基本一致，但是新增一个文件更加灵活，注释在pid_study很多了，此处省略部分
 */

#ifndef INC_PID_INCREMENTAL_H_
#define INC_PID_INCREMENTAL_H_

//PID系数
typedef struct{

	double Kp;	//比例增益
	double Ki;	//积分
	double Kd;	//微分

}	PID_KpidGain_Inc;

//PID状态结构体
typedef struct{

	double target;			//设定目标
	double actual;			//实际目标
	double error;			//当前误差
	double prev_error;		//上次_误差
	double prev_prev_error;	//上上次_误差（增量式PID需要）
	double integral;		//积分项
	double derivative;		//微分项
	double output;			//PID最终输出
	double prev_output;		//上次输出（增量式PID需要）
	double time_delta;		//时间dt

	double output_max;		//输出上限（新增限幅参数）
	double output_min;		//输出下限（新增限幅参数）

}	PID_State_Inc;

//初始化增量PID函数定义 初始给定目标值和限幅参数
void PID_Inc_Init(PID_State_Inc *state, double target, double output_max, double output_min);


//执行增量PID迭代(Iterate)函数定义
PID_State_Inc PID_Inc_Iterate(PID_KpidGain_Inc KpidGain, PID_State_Inc state);

#endif /* INC_PID_INCREMENTAL_H_ */
