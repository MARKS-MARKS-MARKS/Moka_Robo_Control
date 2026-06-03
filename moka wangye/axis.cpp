/*
 *	axis.cpp
 *	function: 包含对于 Axis 类中声明的函数的定义
 *  Created on: Apr 20, 2021
 *      Author: liuchongyang
 */

#include <deque>
#include "axis.h"
//#include "EcMasterApi.h"

//std::map<int,pdo_object_type> Axis::pdo_index_type_map;

//Axis* axis = nullptr;

Axis::Axis()
{
	dh = DH_param();
	decare = Decare_Para();
	motor_param = Motor_Param();
	encoder_param = Encoder_Param();
}

void Axis::add_angle_deque(std::deque<double>& deque)
{
	for (auto i : deque) {
        angle_deque.push_back(i);
    }
}

void Axis::set_angle_deque(std::deque<double>& deque)
{
	angle_deque = deque;
}

std::deque<double>& Axis::get_angle_deque()
{
	return angle_deque;
}

int Axis::set_target_pos_to_servo(int i)
{

	double temp_pos;
	int temp_inc;

	//for(int i=0;i<6;i++)
	//{
		temp_pos = angle_deque.front();
		angle_deque.pop_front();
		angleToInc(temp_pos,temp_inc,i);
	//}
	//int * ptr =&temp_inc[0];
	return temp_inc;
}

void Axis::set_act_inc(int* inc)
{
	for (int i = 0; i < 6; i++)
	{
		ActInc[i] = inc[i];
	}
}

double Axis::getActPositionAngle(int axis)
{

	double angle = 0;
	incToAngle(ActInc[axis], angle, axis);
	return angle;
}




void Axis::incToAngle(signed int inc,double& angle,int i)
{
	angle = ( ( ((double)inc) * 360 / (1 << encoder_param.encoderResolution[i]) - encoder_param.singleTurnEncoder[i]) / encoder_param.reducRatio[i] / encoder_param.direction[i]);
}
/*  角度值转编码器inc值  */
void Axis::angleToInc(double angle, signed int& inc,int i)
{
	inc = (angle * encoder_param.direction[i] * encoder_param.reducRatio[i] + encoder_param.singleTurnEncoder[i] + encoder_param.deviation[i]) * (1 << encoder_param.encoderResolution[i]) / 360.0;
}
void Axis::set_param(Encoder_Param param, Motor_Param param_set, DH_param param_DH_set, Decare_Para param_Decare_set)
{
	for(int i = 0;i < 6;i++)
	{
		motor_param.RatedVel_rpm[i] = param_set.RatedVel_rpm[i];
		motor_param.RatedVel[i] = param_set.RatedVel[i];
		motor_param.DeRatedVel[i] = param_set.DeRatedVel[i];
		motor_param.maxAcc[i] = param_set.maxAcc[i];
		motor_param.maxDecel[i] = param_set.maxDecel[i];
		motor_param.maxRotSpeed[i] = param_set.maxRotSpeed[i];
		encoder_param.direction[i] = param.direction[i];
		encoder_param.reducRatio[i] = param.reducRatio[i];
		encoder_param.encoderResolution[i] = param.encoderResolution[i];
		encoder_param.singleTurnEncoder[i] = param.singleTurnEncoder[i];
		encoder_param.deviation[i] = param.deviation[i];
		dh.alpha[i] = param_DH_set.alpha[i];
		dh.d[i] = param_DH_set.d[i];
		dh.a[i] = param_DH_set.a[i];
		dh.theta[i] = param_DH_set.theta[i];
		decare.maxvel = param_Decare_set.maxvel;
		decare.maxacc = param_Decare_set.maxacc;
		decare.maxdec = param_Decare_set.maxdec;
		decare.maxjerk = param_Decare_set.maxjerk;
	}
}
Encoder_Param Axis::get_encoder_param()
{
	return encoder_param;
}
Motor_Param Axis::get_motor_param()
{
	return motor_param;
}



