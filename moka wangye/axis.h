/*
 *	axis.h
 *	function: 定义 Axis 类
 *  Created on: Apr 20, 2021
 *      Author: liuchongyang
 */

#ifndef PROJECT_BASE_AXIS_H_
#define PROJECT_BASE_AXIS_H_
#include <string>
#include <deque>
#include <map>
//#include "EcMasterApi.h"
//  .\SRC\  这样写表示，当前目录中的SRC文件夹；
// ..\SRC\  这样写表示，当前目录的上一层目录中SRC文件夹；
// ...\SRC\ 这样写表示，当前目录下  ..\SRC\\ 路径，当然这种写法已经和第一种重叠了。
// #include "../openRC/project_base/struct_define.h"
#include "struct_define.h"
#include "eigen/Eigen/Eigen"
using namespace Eigen;

#define rad2deg(r) ((r)*180.0/M_PI)
#define deg2rad(d) ((d)*M_PI/180.0)
/// @brief 定义轴类 
/// 主要是对轴关节相关的控制信息的处理
///	包含有对于关节角、关节速度、关节转矩等控制,以及对于这些控制所需要的
/// 1. 对于伺服电机当前所处状态（是否上电、其操作模式等）的判断
/// 2. 利用EtherCatMaster提供的 API 来设置或获取 SDO、PDO 对伺服电机进行控制
/// 
class Axis
{
	/// @brief 定义一个放置伺服对象类型的类
	/// 主要包含 type 参数（pdo_object_type类型）和 union(放置可供选取参数的类型)

public:
	Axis();
	~Axis();

	/// @brief 在轴的原有的关节角度序列上增加关节序列
	/// @param deque 关节角度序列
	void add_angle_deque(std::deque<double>& deque);

	/// @brief 设置轴的关节角度序列
	/// @param deque 关节角度序列
	void set_angle_deque(std::deque<double>& deque);

	/// @brief 获取轴的关节角度序列
	/// @return 关节角度序列
	std::deque<double>& get_angle_deque();


	/// @brief 给伺服设定目标位置
	int set_target_pos_to_servo(int i);

	void set_act_inc(int* inc);

	double getActPositionAngle(int axis);

	void incToAngle(signed int inc,double& angle,int i);

	/// @brief 将关节角度值转变成编码器的 inc 值
	/// @param inc 编码器的 inc 值
	/// @param angle 关节角度值
	/// @param i 编码器索引值
	void angleToInc(double angle,signed int& inc,int i);

	/// @brief 设置编码器参数
	/// @param param 编码器参数值
	void set_param(Encoder_Param param, Motor_Param param_set, DH_param param_DH_set, Decare_Para param_Decare_set);

	/// @brief 获得编码器参数
	/// @return 编码器参数值
	Encoder_Param get_encoder_param();

	/// @brief 设置 SDO
	/// @param slaveNum 伺服索引
	/// @param index 对象字典索引
	/// @param subindex 对象字典子索引
	/// @param value 数据类型？（）
	/// @param size 数据大小
	/// @return 是否设置成功

	Motor_Param get_motor_param();



	/// @brief 利用 map 容器设置 int 和 pdo_object_type 对应关系
	static std::map<int,pdo_object_type> pdo_index_type_map;
protected:
	Encoder_Param encoder_param;		// 编码器参数
	DH_param dh; ///DH参数对象
	Decare_Para decare;  ///笛卡尔参数对象
	Motor_Param motor_param;  ///电机参数对象
	//std::deque<std::string> slaveType;	// 伺服列表
	std::deque<double> angle_deque;		// 关节角度序列
	int ActInc[6];
    //bool poweronstatus;

	
	/// @brief 利用 map 容器设置 int 和 std::vector<Slave_object_type> 对应关系
	//std::map<int,std::vector<Slave_object_type>> pdo_index_address_map;
};

#endif /* PROJECT_BASE_AXIS_H_ */
