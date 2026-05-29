#include <unistd.h>
#include "general_6s.h"
#include <errno.h>
#include <signal.h>
#include <stdio.h>
#include <string.h>
#include <sys/resource.h>
#include <sys/time.h>
#include <sys/types.h>
#include <time.h>
#include <sys/mman.h>
#include <malloc.h>
#include <sched.h> 
#include <thread>
#include "eigen/Eigen/Eigen"
#include <deque> 
#include <vector>
#include <iostream>
#include <sstream>
#include <fstream>  
#include <mutex>

// --- 新增：网络通信库 ---
#include <sys/socket.h>
#include <netinet/in.h>
#include <arpa/inet.h>

#include "ecrt.h"

using namespace Eigen;
extern General_6S* g_general_6s;

#define FREQUENCY 1000
#define CLOCK_TO_USE CLOCK_MONOTONIC
#define TARGET_POSITION         0 
#define CYCLIC_POSITION            8   

#define NSEC_PER_SEC (1000000000L)
#define PERIOD_NS (NSEC_PER_SEC / FREQUENCY)    

#define DIFF_NS(A, B) (((B).tv_sec - (A).tv_sec) * NSEC_PER_SEC + (B).tv_nsec - (A).tv_nsec)
#define TIMESPEC2NS(T) ((uint64_t) (T).tv_sec * NSEC_PER_SEC + (T).tv_nsec)

// EtherCAT
static ec_master_t* master = NULL;
static ec_master_state_t master_state = {};
static ec_domain_t* domain1 = NULL;
static ec_domain_state_t domain1_state = {};
static ec_slave_config_t* sc[7] = {};
static ec_slave_config_state_t sc_state[7] = {};

int flag[6] = { 0 };
int flag2 = 0;
int step_ms = 0;
static uint8_t* domain1_pd = NULL;
static bool IO_ISCHANGE = false;
static bool IO_OUTPUT_VALID = false;
static int io_data = 0;

#define PANASONIC_5        0,5                        
#define PANASONIC_4        0,4                        
#define PANASONIC_3        0,3                        
#define PANASONIC_2        0,2                        
#define PANASONIC_1        0,1                        
#define PANASONIC_0        0,0                        
#define IO_ban             0,6
#define AXIS_NUM 6
#define ENABLE_GRIPPER_IO 1
#define SLAVE_NUM (ENABLE_GRIPPER_IO ? 7 : AXIS_NUM)
#define GRIPPER_AIR_IO 13
#define GRIPPER_OPEN_IO 15
#define GRIPPER_CLOSE_IO 16
uint16_t a[7] = { 0 };
uint16_t p[7] = { 0,1,2,3,4,5,6 };
#define VID_PID   0x00000922,0x00000a01   
#define VID_PID2  0x00000c6d,0x00000001

struct io_out {
	int io1 = 0;
	int io2 = 0;
	int io3 = 0;
	int io4 = 0;
	int io5 = 0;
	int io6 = 0;
	int io7 = 0;
	int io8 = 0;
	int io9 = 0;
	int io10 = 0;
	int io11 = 0;
	int io12 = 0;
	int io13 = 0;
	int io14 = 0;
	int io15 = 0;
	int io16 = 0;
};

struct PosePoint {
	double x;
	double y;
	double z;
	double rx;
	double ry;
	double rz;
};

static io_out IO_out;

bool PowerStatus = 0;
bool NeedPowerOn = 0;
bool NeedPowerOff = 0;
std::deque<double> angle_deque;
std::deque<double> angle_deque_out;
std::deque<int> tor_deque_out;
bool count_start = 0;
static unsigned long long cycle_count = 0;
static unsigned long long servo_write_count = 0;
static double last_actual_deg[6] = {0};
static double last_target_deg[6] = {0};
static std::string last_command = "none";
static std::mutex trajectory_mutex;

using namespace std;

static struct {
	unsigned int ctrl_word[6];
	unsigned int operation_mode[6];
	unsigned int target_position[6];
	unsigned int touch_probe_function[6];
	unsigned int error_code[6];
	unsigned int status_word[6];
	unsigned int modes_of_operation_display[6];
	unsigned int position_actual_value[6];
	unsigned int touch_probe_status[6];
	unsigned int touch_probe_pos1_pos_value[6];
	unsigned int following_error_actual_value[6];
	unsigned int digital_inputs[6];
	unsigned int digital[6];
	unsigned int target_velocity[6];
	unsigned int target_torque[6];
	unsigned int dummy_byte1[6];
	unsigned int position_offset[6];
	unsigned int velocity_offset[6];
	unsigned int torque_offset[6];
	unsigned int velocity_actual_value[6];
	unsigned int torque_actual_value[6];
	unsigned int dummy_byte2[6];
	unsigned int BC[6];
	unsigned int F[6];
	unsigned int io_out;
	unsigned int io_in;
}offset;

const static ec_pdo_entry_reg_t domain1_regs[] = {
	{PANASONIC_0, VID_PID, 0x6040, 0, &offset.ctrl_word[0]},
	{PANASONIC_0, VID_PID, 0x607A, 0, &offset.target_position[0]},
	{PANASONIC_0, VID_PID, 0x6060, 0, &offset.operation_mode[0] },
	{PANASONIC_0, VID_PID, 0x6041, 0, &offset.status_word[0]},
	{PANASONIC_0, VID_PID, 0x6064, 0, &offset.position_actual_value[0]},
	{PANASONIC_0, VID_PID, 0x6077, 0, &offset.torque_actual_value[0]},

	{PANASONIC_1, VID_PID, 0x6040, 0, &offset.ctrl_word[1]},
	{PANASONIC_1, VID_PID, 0x607A, 0, &offset.target_position[1]},
	{PANASONIC_1, VID_PID, 0x6060, 0, &offset.operation_mode[1] },
	{PANASONIC_1, VID_PID, 0x6041, 0, &offset.status_word[1]},
	{PANASONIC_1, VID_PID, 0x6064, 0, &offset.position_actual_value[1]},
	{PANASONIC_1, VID_PID, 0x6077, 0, &offset.torque_actual_value[1]},

	{PANASONIC_2, VID_PID, 0x6040, 0, &offset.ctrl_word[2]},
	{PANASONIC_2, VID_PID, 0x607A, 0, &offset.target_position[2]},
	{PANASONIC_2, VID_PID, 0x6060, 0, &offset.operation_mode[2] },
	{PANASONIC_2, VID_PID, 0x6041, 0, &offset.status_word[2]},
	{PANASONIC_2, VID_PID, 0x6064, 0, &offset.position_actual_value[2]},
	{PANASONIC_2, VID_PID, 0x6077, 0, &offset.torque_actual_value[2]},

	{PANASONIC_3, VID_PID, 0x6040, 0, &offset.ctrl_word[3]},
	{PANASONIC_3, VID_PID, 0x607A, 0, &offset.target_position[3]},
	{PANASONIC_3, VID_PID, 0x6060, 0, &offset.operation_mode[3] },
	{PANASONIC_3, VID_PID, 0x6041, 0, &offset.status_word[3]},
	{PANASONIC_3, VID_PID, 0x6064, 0, &offset.position_actual_value[3]},
	{PANASONIC_3, VID_PID, 0x6077, 0, &offset.torque_actual_value[3]},

	{PANASONIC_4, VID_PID, 0x6040, 0, &offset.ctrl_word[4]},
	{PANASONIC_4, VID_PID, 0x607A, 0, &offset.target_position[4]},
	{PANASONIC_4, VID_PID, 0x6060, 0, &offset.operation_mode[4] },
	{PANASONIC_4, VID_PID, 0x6041, 0, &offset.status_word[4]},
	{PANASONIC_4, VID_PID, 0x6064, 0, &offset.position_actual_value[4]},
	{PANASONIC_4, VID_PID, 0x6077, 0, &offset.torque_actual_value[4]},

	{PANASONIC_5, VID_PID, 0x6040, 0, &offset.ctrl_word[5]},
	{PANASONIC_5, VID_PID, 0x607A, 0, &offset.target_position[5]},
	{PANASONIC_5, VID_PID, 0x6060, 0, &offset.operation_mode[5] },
	{PANASONIC_5, VID_PID, 0x6041, 0, &offset.status_word[5]},
	{PANASONIC_5, VID_PID, 0x6064, 0, &offset.position_actual_value[5]},
	{PANASONIC_5, VID_PID, 0x6077, 0, &offset.torque_actual_value[5]},
#if ENABLE_GRIPPER_IO
	{IO_ban, VID_PID2, 0x7000, 0, &offset.io_out},
	{IO_ban, VID_PID2, 0x6000, 0, &offset.io_in},
#endif
	{}
};

static ec_pdo_entry_info_t device_pdo_entries[] = {
	{0x6040, 0x00, 16}, {0x607a, 0x00, 32}, {0x60b8, 0x00, 16}, {0x6060, 0x00, 8},
	{0x6041, 0x00, 16}, {0x6064, 0x00, 32}, {0x60b9, 0x00, 16}, {0x60ba, 0x00, 32},
	{0x60bc, 0x00, 32}, {0x603f, 0x00, 16}, {0x60fd, 0x00, 32}, {0x6077, 0x00, 16},
};
static ec_pdo_info_t device_pdos[] = {
	{0x1600, 4, device_pdo_entries + 0 }, {0x1A00, 8, device_pdo_entries + 4 }
};
static ec_pdo_entry_info_t device2_pdo_entries[] = {
	{0x7000, 0x00, 16}, {0x6000, 0x00, 16},
};
static ec_pdo_info_t device2_pdos[] = {
	{0x1600, 1, device2_pdo_entries + 0}, {0x1A00, 1, device2_pdo_entries + 1}
};
static ec_sync_info_t device_syncs[] = {
	{ 0, EC_DIR_OUTPUT, 0, NULL, EC_WD_DISABLE },
	{ 1, EC_DIR_INPUT, 0, NULL, EC_WD_DISABLE },
	{ 2, EC_DIR_OUTPUT, 1, device_pdos + 0, EC_WD_ENABLE },
	{ 3, EC_DIR_INPUT, 1, device_pdos + 1, EC_WD_DISABLE },
	{ 0xFF}
};
static ec_sync_info_t device2_syncs[] = {
	{ 0, EC_DIR_OUTPUT, 0, NULL, EC_WD_DISABLE },
	{ 1, EC_DIR_INPUT, 0, NULL, EC_WD_DISABLE },
	{ 2, EC_DIR_OUTPUT, 1, device2_pdos + 0, EC_WD_ENABLE },
	{ 3, EC_DIR_INPUT, 1, device2_pdos + 1, EC_WD_DISABLE },
	{ 0xFF}
};

static unsigned int counter = 0;
static unsigned int blink = 0;
static unsigned int sync_ref_counter = 0;
const struct timespec cycletime = { 0, PERIOD_NS };

struct timespec timespec_add(struct timespec time1, struct timespec time2){
	struct timespec result;
	if ((time1.tv_nsec + time2.tv_nsec) >= NSEC_PER_SEC) {
		result.tv_sec = time1.tv_sec + time2.tv_sec + 1;
		result.tv_nsec = time1.tv_nsec + time2.tv_nsec - NSEC_PER_SEC;
	} else {
		result.tv_sec = time1.tv_sec + time2.tv_sec;
		result.tv_nsec = time1.tv_nsec + time2.tv_nsec;
	}
	return result;
}

void check_domain1_state(void){ /*...*/ }
void check_master_state(void){ /*...*/ }
void check_slave_config_states(ec_slave_config_t* sc, int i) {
	ec_slave_config_state_t s;
	ecrt_slave_config_state(sc, &s);
	if (i < AXIS_NUM && s.operational == 1) flag[i] = 1;
	sc_state[i] = s;
}

int pack_io_data(const io_out& out) {
	return out.io1 + out.io2 * 2 + out.io3 * 4 + out.io4 * 8 +
		out.io5 * 16 + out.io6 * 32 + out.io7 * 64 + out.io8 * 128 +
		out.io9 * 256 + out.io10 * 512 + out.io11 * 1024 + out.io12 * 2048 +
		out.io13 * 4096 + out.io14 * 4096 * 2 +
		out.io15 * 4096 * 4 + out.io16 * 4096 * 8;
}

void set_io_bit(io_out& out, int channel, int value) {
	switch (channel) {
		case 1: out.io1 = value; break;
		case 2: out.io2 = value; break;
		case 3: out.io3 = value; break;
		case 4: out.io4 = value; break;
		case 5: out.io5 = value; break;
		case 6: out.io6 = value; break;
		case 7: out.io7 = value; break;
		case 8: out.io8 = value; break;
		case 9: out.io9 = value; break;
		case 10: out.io10 = value; break;
		case 11: out.io11 = value; break;
		case 12: out.io12 = value; break;
		case 13: out.io13 = value; break;
		case 14: out.io14 = value; break;
		case 15: out.io15 = value; break;
		case 16: out.io16 = value; break;
		default: break;
	}
}

int get_io_bit(const io_out& out, int channel) {
	switch (channel) {
		case 1: return out.io1;
		case 2: return out.io2;
		case 3: return out.io3;
		case 4: return out.io4;
		case 5: return out.io5;
		case 6: return out.io6;
		case 7: return out.io7;
		case 8: return out.io8;
		case 9: return out.io9;
		case 10: return out.io10;
		case 11: return out.io11;
		case 12: return out.io12;
		case 13: return out.io13;
		case 14: return out.io14;
		case 15: return out.io15;
		case 16: return out.io16;
		default: return 0;
	}
}

void set_gripper(bool closed) {
#if ENABLE_GRIPPER_IO
	set_io_bit(IO_out, GRIPPER_AIR_IO, 1);
	set_io_bit(IO_out, GRIPPER_OPEN_IO, closed ? 0 : 1);
	set_io_bit(IO_out, GRIPPER_CLOSE_IO, closed ? 1 : 0);
	io_data = pack_io_data(IO_out);
	IO_ISCHANGE = true;
	IO_OUTPUT_VALID = true;
	printf("[底层] 夹爪%s指令已写入 IO：air io%d=%d open io%d=%d close io%d=%d data=%d。\n",
		closed ? "夹紧" : "松开",
		GRIPPER_AIR_IO, get_io_bit(IO_out, GRIPPER_AIR_IO),
		GRIPPER_OPEN_IO, get_io_bit(IO_out, GRIPPER_OPEN_IO),
		GRIPPER_CLOSE_IO, get_io_bit(IO_out, GRIPPER_CLOSE_IO),
		io_data);
#else
	printf("[底层] 已收到夹爪%s指令，但当前构建未启用 IO 从站，未写 EtherCAT IO。\n", closed ? "夹紧" : "松开");
#endif
}

void set_single_io(int channel, int value) {
#if ENABLE_GRIPPER_IO
	if (channel < 1 || channel > 16) {
		printf("[底层-拒绝] IO 通道非法: %d，应为 1-16。\n", channel);
		return;
	}
	set_io_bit(IO_out, channel, value ? 1 : 0);
	io_data = pack_io_data(IO_out);
	IO_ISCHANGE = true;
	IO_OUTPUT_VALID = true;
	printf("[底层] IOSET io%d=%d data=%d。\n", channel, value ? 1 : 0, io_data);
#else
	printf("[底层] 已收到 IOSET io%d=%d，但当前构建未启用 IO 从站。\n", channel, value ? 1 : 0);
#endif
}

void clear_all_io() {
#if ENABLE_GRIPPER_IO
	IO_out = io_out();
	io_data = pack_io_data(IO_out);
	IO_ISCHANGE = true;
	IO_OUTPUT_VALID = true;
	printf("[底层] 已清空全部 IO 输出，data=%d。\n", io_data);
#else
	printf("[底层] 已收到 IOCLEAR，但当前构建未启用 IO 从站。\n");
#endif
}

void write_debug_status() {
	static unsigned long long last_dump_cycle = 0;
	if (cycle_count - last_dump_cycle < FREQUENCY) return;
	last_dump_cycle = cycle_count;

	std::ofstream out("/tmp/moka_status.txt");
	if (!out) return;
	out << "cycle_count=" << cycle_count << "\n";
	out << "PowerStatus=" << PowerStatus << "\n";
	out << "NeedPowerOn=" << NeedPowerOn << "\n";
	out << "flag2=" << flag2 << "\n";
	out << "flags=";
	for (int i = 0; i < 6; i++) out << flag[i] << (i == 5 ? "\n" : " ");
	size_t queue_size = 0;
	{
		std::lock_guard<std::mutex> lock(trajectory_mutex);
		queue_size = g_general_6s->get_angle_deque().size();
	}
	out << "queue_size=" << queue_size << "\n";
	out << "servo_write_count=" << servo_write_count << "\n";
	out << "last_command=" << last_command << "\n";
#if ENABLE_GRIPPER_IO
	out << "io_enabled=1\n";
	out << "io_output_valid=" << IO_OUTPUT_VALID << "\n";
	out << "io_data=" << io_data << "\n";
	out << "gripper_air_io=" << GRIPPER_AIR_IO << "\n";
	out << "gripper_close_io=" << GRIPPER_CLOSE_IO << "\n";
	out << "gripper_open_io=" << GRIPPER_OPEN_IO << "\n";
	out << "gripper_air_value=" << get_io_bit(IO_out, GRIPPER_AIR_IO) << "\n";
	out << "gripper_close_value=" << get_io_bit(IO_out, GRIPPER_CLOSE_IO) << "\n";
	out << "gripper_open_value=" << get_io_bit(IO_out, GRIPPER_OPEN_IO) << "\n";
	out << "io13=" << IO_out.io13 << "\n";
	out << "io5=" << IO_out.io5 << "\n";
	out << "io6=" << IO_out.io6 << "\n";
	out << "io15=" << IO_out.io15 << "\n";
	out << "io16=" << IO_out.io16 << "\n";
	out << "io_bits=";
	for (int i = 1; i <= 16; i++) out << get_io_bit(IO_out, i) << (i == 16 ? "\n" : " ");
#else
	out << "io_enabled=0\n";
#endif
	out << "actual_deg=";
	for (int i = 0; i < 6; i++) out << last_actual_deg[i] << (i == 5 ? "\n" : " ");
	out << "last_target_deg=";
	for (int i = 0; i < 6; i++) out << last_target_deg[i] << (i == 5 ? "\n" : " ");
}

void append_line_move(const VectorXd& target, std::deque<double>& trajectory) {
	VectorXd originACS(6);
	for(int i=0; i<6; i++) {
		originACS(i) = g_general_6s->getActPositionAngle(i);
	}

	MatrixXd originTrans;
	g_general_6s->calc_forward_kin(originACS, originTrans);
	VectorXd originMCS = g_general_6s->tr_2_MCS(originTrans);

	if (!trajectory.empty() && trajectory.size() >= 6) {
		for (int i = 0; i < 6; i++) {
			originACS(i) = trajectory[trajectory.size() - 6 + i];
		}
		g_general_6s->calc_forward_kin(originACS, originTrans);
		originMCS = g_general_6s->tr_2_MCS(originTrans);
	}

	VectorXd safeTarget(target);
	// 书写时只改变 TCP 位置，姿态保持当前末端姿态，避免逆解切到手腕翻转构型。
	safeTarget(3) = originMCS(3);
	safeTarget(4) = originMCS(4);
	safeTarget(5) = originMCS(5);

	VectorXd targetACS(6);
	targetACS = originACS;
	g_general_6s->calc_inverse_kin(g_general_6s->rpy_2_tr(safeTarget), originACS, targetACS);

	for (int i = 0; i < 6; i++) {
		if (fabs(targetACS(i) - originACS(i)) > 35.0) {
			printf("[底层-拒绝] WRITE 单段关节变化过大: J%d %.3f -> %.3f。\n", i + 1, originACS(i), targetACS(i));
			return;
		}
	}

	VectorXd vel_current = VectorXd::Zero(6);
	VectorXd acc_current = VectorXd::Zero(6);
	g_general_6s->move_joint_interp(
		targetACS, originACS, vel_current, acc_current,
		0.001, 3.0, 5.0, 5.0, 5.0,
		trajectory
	);
}

bool append_pose_move(const VectorXd& target, std::deque<double>& trajectory) {
	VectorXd originACS(6);
	for(int i=0; i<6; i++) {
		originACS(i) = g_general_6s->getActPositionAngle(i);
	}

	if (!trajectory.empty() && trajectory.size() >= 6) {
		for (int i = 0; i < 6; i++) {
			originACS(i) = trajectory[trajectory.size() - 6 + i];
		}
	}

	VectorXd targetACS(6);
	targetACS = originACS;
	g_general_6s->calc_inverse_kin(g_general_6s->rpy_2_tr(target), originACS, targetACS);

	double max_delta = 0.0;
	for (int i = 0; i < 6; i++) {
		if (fabs(targetACS(i) - originACS(i)) > max_delta) {
			max_delta = fabs(targetACS(i) - originACS(i));
		}
	}
	if (max_delta < 0.001) {
		printf("[底层-拒绝] CARTPATH 逆解未产生有效关节目标，目标TCP: %.3f %.3f %.3f %.3f %.3f %.3f。\n",
			target(0), target(1), target(2), target(3), target(4), target(5));
		return false;
	}

	for (int i = 0; i < 6; i++) {
		if (fabs(targetACS(i) - originACS(i)) > 45.0) {
			printf("[底层-拒绝] CARTPATH 单段关节变化过大: J%d %.3f -> %.3f。\n", i + 1, originACS(i), targetACS(i));
			return false;
		}
	}

	VectorXd vel_current = VectorXd::Zero(6);
	VectorXd acc_current = VectorXd::Zero(6);
	g_general_6s->move_joint_interp(
		targetACS, originACS, vel_current, acc_current,
		0.001, 10.0, 10.0, 10.0, 10.0,
		trajectory
	);
	return true;
}

bool append_transform_move(const MatrixXd& targetTrans, std::deque<double>& trajectory, const char* tag) {
	VectorXd originACS(6);
	for(int i=0; i<6; i++) {
		originACS(i) = g_general_6s->getActPositionAngle(i);
	}

	if (!trajectory.empty() && trajectory.size() >= 6) {
		for (int i = 0; i < 6; i++) {
			originACS(i) = trajectory[trajectory.size() - 6 + i];
		}
	}

	VectorXd targetACS(6);
	targetACS = originACS;
	g_general_6s->calc_inverse_kin(targetTrans, originACS, targetACS);

	double max_delta = 0.0;
	for (int i = 0; i < 6; i++) {
		if (fabs(targetACS(i) - originACS(i)) > max_delta) {
			max_delta = fabs(targetACS(i) - originACS(i));
		}
	}
	if (max_delta < 0.001) {
		printf("[底层-拒绝] %s 逆解未产生有效关节目标，目标XYZ: %.3f %.3f %.3f。\n",
			tag, targetTrans(0, 3), targetTrans(1, 3), targetTrans(2, 3));
		return false;
	}

	for (int i = 0; i < 6; i++) {
		if (fabs(targetACS(i) - originACS(i)) > 45.0) {
			printf("[底层-拒绝] %s 单段关节变化过大: J%d %.3f -> %.3f。\n",
				tag, i + 1, originACS(i), targetACS(i));
			return false;
		}
	}

	printf("[底层] %s IK目标关节: %.3f %.3f %.3f %.3f %.3f %.3f。\n",
		tag, targetACS(0), targetACS(1), targetACS(2), targetACS(3), targetACS(4), targetACS(5));

	VectorXd vel_current = VectorXd::Zero(6);
	VectorXd acc_current = VectorXd::Zero(6);
	g_general_6s->move_joint_interp(
		targetACS, originACS, vel_current, acc_current,
		0.001, 10.0, 10.0, 10.0, 10.0,
		trajectory
	);
	return true;
}

VectorXd current_or_queued_tcp(const std::deque<double>& trajectory) {
	VectorXd originACS(6);
	for(int i=0; i<6; i++) {
		originACS(i) = g_general_6s->getActPositionAngle(i);
	}

	if (!trajectory.empty() && trajectory.size() >= 6) {
		for (int i = 0; i < 6; i++) {
			originACS(i) = trajectory[trajectory.size() - 6 + i];
		}
	}

	MatrixXd originTrans;
	g_general_6s->calc_forward_kin(originACS, originTrans);
	return g_general_6s->tr_2_MCS(originTrans);
}

VectorXd current_or_queued_acs(const std::deque<double>& trajectory) {
	VectorXd originACS(6);
	for(int i=0; i<6; i++) {
		originACS(i) = g_general_6s->getActPositionAngle(i);
	}

	if (!trajectory.empty() && trajectory.size() >= 6) {
		for (int i = 0; i < 6; i++) {
			originACS(i) = trajectory[trajectory.size() - 6 + i];
		}
	}
	return originACS;
}

VectorXd dh_zero_tcp_pose() {
	VectorXd zeroACS = VectorXd::Zero(6);
	MatrixXd zeroTrans;
	g_general_6s->calc_forward_kin(zeroACS, zeroTrans);
	return g_general_6s->tr_2_MCS(zeroTrans);
}

void execute_cartesian_path(const std::vector<PosePoint>& points) {
	if (points.empty()) return;

	std::deque<double> new_trajectory;
	size_t max_points = points.size() > 80 ? 80 : points.size();
	for (size_t i = 0; i < max_points; i++) {
		if (fabs(points[i].x) > 1200 || fabs(points[i].y) > 1200 || points[i].z < -100 || points[i].z > 1200) {
			printf("[底层-拒绝] CARTPATH 航点超出安全范围: %.3f %.3f %.3f。\n", points[i].x, points[i].y, points[i].z);
			return;
		}
		VectorXd target(6);
		target << points[i].x, points[i].y, points[i].z, points[i].rx, points[i].ry, points[i].rz;
		size_t before_size = new_trajectory.size();
		if (!append_pose_move(target, new_trajectory)) {
			return;
		}
		if (new_trajectory.size() == before_size) {
			printf("[底层-拒绝] CARTPATH 未生成关节插补轨迹，目标TCP: %.3f %.3f %.3f %.3f %.3f %.3f。\n",
				target(0), target(1), target(2), target(3), target(4), target(5));
			return;
		}
	}

	if (new_trajectory.empty()) {
		printf("[底层-拒绝] CARTPATH 未生成轨迹，请检查目标点是否可达。\n");
		return;
	}

	{
		std::lock_guard<std::mutex> lock(trajectory_mutex);
		g_general_6s->set_angle_deque(new_trajectory);
	}
	printf("[底层] CARTPATH 已生成轨迹：%zu/%zu 个位姿点，队列长度=%zu。\n",
		max_points, points.size(), new_trajectory.size());
}

void execute_teach_xyz_path(const std::vector<PosePoint>& points) {
	if (points.empty()) return;

	VectorXd zeroACS = VectorXd::Zero(6);
	MatrixXd zeroTrans;
	g_general_6s->calc_forward_kin(zeroACS, zeroTrans);
	VectorXd zeroPose = g_general_6s->tr_2_MCS(zeroTrans);
	std::deque<double> new_trajectory;
	size_t max_points = points.size() > 80 ? 80 : points.size();
	for (size_t i = 0; i < max_points; i++) {
		if (fabs(points[i].x) > 1200 || fabs(points[i].y) > 1200 || points[i].z < -100 || points[i].z > 1200) {
			printf("[底层-拒绝] TEACHXYZ 目标超出安全范围: %.3f %.3f %.3f。\n", points[i].x, points[i].y, points[i].z);
			return;
		}

		MatrixXd targetTrans = zeroTrans;
		targetTrans(0, 3) = points[i].x;
		targetTrans(1, 3) = points[i].y;
		targetTrans(2, 3) = points[i].z;
		size_t before_size = new_trajectory.size();
		if (!append_transform_move(targetTrans, new_trajectory, "TEACHXYZ")) {
			return;
		}
		if (new_trajectory.size() == before_size) {
			printf("[底层-拒绝] TEACHXYZ 未生成轨迹，目标XYZ: %.3f %.3f %.3f。\n",
				points[i].x, points[i].y, points[i].z);
			return;
		}
	}

	{
		std::lock_guard<std::mutex> lock(trajectory_mutex);
		g_general_6s->set_angle_deque(new_trajectory);
	}
	printf("[底层] TEACHXYZ 已用DH零位姿态生成轨迹：%zu/%zu 点，姿态=%.3f %.3f %.3f，队列长度=%zu。\n",
		max_points, points.size(), zeroPose(3), zeroPose(4), zeroPose(5), new_trajectory.size());
}

void execute_write_path(const std::vector<PosePoint>& points, double lift_mm) {
	if (points.empty()) return;

	std::deque<double> new_trajectory;
	size_t max_points = points.size() > 80 ? 80 : points.size();
	for (size_t i = 0; i < max_points; i++) {
		if (fabs(points[i].x) > 1200 || fabs(points[i].y) > 1200 || points[i].z < -100 || points[i].z > 1200) {
			printf("[底层-拒绝] WRITE 航点超出安全范围: %.3f %.3f %.3f。\n", points[i].x, points[i].y, points[i].z);
			return;
		}
		VectorXd target(6);
		target << points[i].x, points[i].y, points[i].z, points[i].rx, points[i].ry, points[i].rz;
		append_line_move(target, new_trajectory);
	}

	{
		std::lock_guard<std::mutex> lock(trajectory_mutex);
		g_general_6s->set_angle_deque(new_trajectory);
	}
	printf("[底层] 已生成书写轨迹：%zu/%zu 个笛卡尔航点，抬笔高度 %.2f mm，队列长度=%zu。\n",
		max_points, points.size(), lift_mm, new_trajectory.size());
}

// -------------------------------------------------------------
// 原有的 EtherCAT 实时循环
void cyclic_task() {
	struct timespec wakeupTime, time;
	clock_gettime(CLOCK_TO_USE, &wakeupTime);

	while (1) {
		cycle_count++;
		wakeupTime = timespec_add(wakeupTime, cycletime);
		clock_nanosleep(CLOCK_TO_USE, TIMER_ABSTIME, &wakeupTime, NULL);
		ecrt_master_application_time(master, TIMESPEC2NS(wakeupTime));

		ecrt_master_receive(master);
		ecrt_domain_process(domain1);

		signed int actualInc[6];
		for (unsigned int i = 0; i < 6; i++) {
			actualInc[i] = EC_READ_S32(domain1_pd + offset.position_actual_value[i]);
		}
		
		g_general_6s->set_act_inc(actualInc);   
		if (NeedPowerOn || PowerStatus) {
			for (unsigned int i = 0; i < 6; i++) {
				last_actual_deg[i] = g_general_6s->getActPositionAngle(i);
			}
		}

		if (counter) {
			counter--;
		} else { 
			counter = FREQUENCY * 2;
			for (int i = 0; i < SLAVE_NUM; i++) {
				check_slave_config_states(sc[i], i);
			}

			if (!PowerStatus && NeedPowerOn) {
				if (flag[0] == 1 && flag[1] == 1 && flag[2] == 1 && flag[3] == 1 && flag[4] == 1 && flag[5] == 1 && flag2 == 0) {
					for (int i = 0; i < 6; i++) EC_WRITE_U16(domain1_pd + offset.ctrl_word[i], 0x0080);
					flag2 = 2;
				} else if (flag2 == 2) {
					for (int i = 0; i < 6; i++) EC_WRITE_U16(domain1_pd + offset.ctrl_word[i], 0x0006);
					flag2 = 3;
				} else if (flag2 == 3) {
					for (int i = 0; i < 6; i++) {
						EC_WRITE_U16(domain1_pd + offset.ctrl_word[i], 0x0007);
						EC_WRITE_S8(domain1_pd + offset.operation_mode[i], CYCLIC_POSITION);
						EC_WRITE_S32(domain1_pd + offset.target_position[i], actualInc[i]);
					}
					flag2 = 4;
				} else if (flag2 == 4) {
					for (int i = 0; i < 6; i++) {
						EC_WRITE_U16(domain1_pd + offset.ctrl_word[i], 0x000f);
						EC_WRITE_S32(domain1_pd + offset.target_position[i], actualInc[i]);
					}
					flag2 = 5;
					PowerStatus = 1;
					NeedPowerOn = 0;
				}
			}
			blink = !blink;
		}

		/* 消耗轨迹队列发给伺服 */
		{
			std::lock_guard<std::mutex> trajectory_lock(trajectory_mutex);
			if (PowerStatus && !g_general_6s->get_angle_deque().empty()) {
				for (int i = 0; i < 6; i++) {
					double target_deg = g_general_6s->get_angle_deque().front();
					last_target_deg[i] = target_deg;
					EC_WRITE_S32(domain1_pd + offset.target_position[i], g_general_6s->set_target_pos_to_servo(i));
				}
				servo_write_count++;
			}
		}

		if (ENABLE_GRIPPER_IO && IO_OUTPUT_VALID) {
			EC_WRITE_U16(domain1_pd + offset.io_out, (uint16_t)io_data);
			IO_ISCHANGE = false;
		}

		if (sync_ref_counter) {
			sync_ref_counter--;
		} else {
			sync_ref_counter = 1; 
			clock_gettime(CLOCK_TO_USE, &time);
			ecrt_master_sync_reference_clock(master);
		}
		ecrt_master_sync_slave_clocks(master);

		ecrt_domain_queue(domain1);
		ecrt_master_send(master);
		write_debug_status();
	}
}

int StartEC() {
	if (mlockall(MCL_CURRENT | MCL_FUTURE) == -1) {
		perror("mlockall failed");
		return -1;
	}

	master = ecrt_request_master(0);
	if (!master) return -1;
	domain1 = ecrt_master_create_domain(master);
	if (!domain1) return -1;

	for (int i = 0; i < SLAVE_NUM; i++) {
		if (i < AXIS_NUM) {
			if (!(sc[i] = ecrt_master_slave_config(master, a[i], p[i], VID_PID))) return -1;
			if (ecrt_slave_config_pdos(sc[i], EC_END, device_syncs)) return -1;
		} else {
			if (!(sc[i] = ecrt_master_slave_config(master, a[i], p[i], VID_PID2))) return -1;
			if (ecrt_slave_config_pdos(sc[i], EC_END, device2_syncs)) return -1;
		}
	}

	if (ecrt_domain_reg_pdo_entry_list(domain1, domain1_regs)) exit(EXIT_FAILURE);
	for (int i = 0; i < AXIS_NUM; i++) ecrt_slave_config_dc(sc[i], 0x0300, PERIOD_NS, PERIOD_NS / 2, 0, 0);

	if (ecrt_master_activate(master)) return -1;
	if (!(domain1_pd = ecrt_domain_data(domain1))) return -1;

	struct sched_param param = {};
	param.sched_priority = sched_get_priority_max(SCHED_FIFO);
	sched_setscheduler(0, SCHED_FIFO, &param);

	cyclic_task();
}

// ======================= 新增：机器人参数初始化 =======================
void init_robot_params() {
	DH_param dh_example;
	dh_example.a[0] = 0.0408;  dh_example.a[1] = 450.342; dh_example.a[2] = 99.107; 
	dh_example.a[3] = 0.0;     dh_example.a[4] = 0.0;     dh_example.a[5] = 0.0;
	dh_example.alpha[0] = M_PI * 90 / 180;  dh_example.alpha[1] = M_PI * 0 / 180;
	dh_example.alpha[2] = M_PI * 90 / 180;  dh_example.alpha[3] = M_PI * 90 / 180;
	dh_example.alpha[4] = M_PI * (-90) / 180; dh_example.alpha[5] = M_PI * 0 / 180;
	dh_example.d[0] = 390;     dh_example.d[1] = 0.4997;  dh_example.d[2] = 0.0;
	dh_example.d[3] = 470.557; dh_example.d[4] = 0.0;     dh_example.d[5] = 123;
	dh_example.theta[0] = M_PI * 0 / 180;  dh_example.theta[1] = M_PI * 90 / 180;
	dh_example.theta[2] = M_PI * 0 / 180;  dh_example.theta[3] = M_PI * 0 / 180;
	dh_example.theta[4] = M_PI * 90 / 180; dh_example.theta[5] = M_PI * 0 / 180;

	Decare_Para decare;
	decare.maxacc = 5; decare.maxdec = -5; decare.maxjerk = 10000; decare.maxvel = 2000;

	Motor_Param motor_pa;
	motor_pa.encoder.reducRatio[0] = 80.007; motor_pa.encoder.reducRatio[1] = 109.837;
	motor_pa.encoder.reducRatio[2] = 100.024; motor_pa.encoder.reducRatio[3] = 118.996;
	motor_pa.encoder.reducRatio[4] = 80.007; motor_pa.encoder.reducRatio[5] = 79.977;

	motor_pa.encoder.singleTurnEncoder[0] = 240.235291; motor_pa.encoder.singleTurnEncoder[1] = 282.285461;
	motor_pa.encoder.singleTurnEncoder[2] = 66.478271; motor_pa.encoder.singleTurnEncoder[3] = 48.540344;
	motor_pa.encoder.singleTurnEncoder[4] = 225.788269; motor_pa.encoder.singleTurnEncoder[5] = 143.937378;

	motor_pa.encoder.direction[0] = -1; motor_pa.encoder.direction[1] = 1; motor_pa.encoder.direction[2] = -1;
	motor_pa.encoder.direction[3] = 1; motor_pa.encoder.direction[4] = 1; motor_pa.encoder.direction[5] = 1;

	motor_pa.RatedVel_rpm[0] = 450; motor_pa.RatedVel_rpm[1] = 350; motor_pa.RatedVel_rpm[2] = 450;
	motor_pa.RatedVel_rpm[3] = 350; motor_pa.RatedVel_rpm[4] = 450; motor_pa.RatedVel_rpm[5] = 450;

	for (int i = 0; i < 6; i++) {
		motor_pa.encoder.deviation[i] = 0;
		motor_pa.encoder.encoderResolution[i] = 23;
		motor_pa.maxAcc[i] = 5.0; motor_pa.maxDecel[i] = -5.0; motor_pa.maxRotSpeed[i] = 5000;
		motor_pa.RatedVel[i] = motor_pa.RatedVel_rpm[i] * 6 / motor_pa.encoder.reducRatio[i];
		motor_pa.DeRatedVel[i] = -motor_pa.RatedVel[i];
	}

	g_general_6s->set_param(motor_pa.encoder, motor_pa, dh_example, decare);
	
	printf("[底座] 机器人参数初始化完成！准备上电...\n");
	NeedPowerOn = 1;
}

// ======================= 新增：指令接收线程 (端口9000) =======================
void command_server_thread() {
    int server_fd = socket(AF_INET, SOCK_STREAM, 0);
    struct sockaddr_in address;
    int opt = 1;
    setsockopt(server_fd, SOL_SOCKET, SO_REUSEADDR | SO_REUSEPORT, &opt, sizeof(opt));
    
    address.sin_family = AF_INET;
    address.sin_addr.s_addr = INADDR_ANY;
    address.sin_port = htons(9000);
    bind(server_fd, (struct sockaddr *)&address, sizeof(address));
    listen(server_fd, 3);
    
    printf("[网络] 指令接收端口 9000 启动监听...\n");
    
    while(true) {
        int new_socket = accept(server_fd, NULL, NULL);
        char buffer[4096] = {0};
        std::string msg;
        ssize_t nread = 0;
        while ((nread = read(new_socket, buffer, sizeof(buffer))) > 0) {
            msg.append(buffer, nread);
            if (nread < (ssize_t)sizeof(buffer)) break;
        }
        
        std::stringstream ss(msg);
        std::string cmd_type;
        ss >> cmd_type;
        last_command = msg.substr(0, 120);
        printf("[网络] 收到指令: %.120s\n", msg.c_str());
        
        if (cmd_type == "MOVEJ") {
            VectorXd target(6);
            for(int i=0; i<6; i++) {
                double val; ss >> val; target(i) = val;
            }
            
            // 获取当前实际角度
            VectorXd origin(6);
            for(int i=0; i<6; i++) {
                origin(i) = g_general_6s->getActPositionAngle(i);
            }
            
            VectorXd vel_current = VectorXd::Zero(6);
            VectorXd acc_current = VectorXd::Zero(6);
            std::deque<double> new_trajectory;
            
            double Ts = 0.001;	       
            double velPerc = 10;	 
            double accPerc = 10;	 
            double decPerc = 10;	 
            double jerkPerc = 10;
            
            // 计算插补轨迹
            g_general_6s->move_joint_interp(
                target, origin, vel_current, acc_current, 
                Ts, velPerc, accPerc, decPerc, jerkPerc, new_trajectory
            );
            
            // 写入底层运动队列，EtherCAT 循环会自动消费它
            {
                std::lock_guard<std::mutex> lock(trajectory_mutex);
                g_general_6s->set_angle_deque(new_trajectory);
            }
            printf("[底层] MOVEJ 插补完成，轨迹队列长度=%zu，PowerStatus=%d。\n", new_trajectory.size(), PowerStatus);
        }
        else if (cmd_type == "MOVEL") {
            VectorXd target(6);
            for(int i=0; i<6; i++) {
                double val; ss >> val; target(i) = val;
            }

            VectorXd originACS(6);
            for(int i=0; i<6; i++) {
                originACS(i) = g_general_6s->getActPositionAngle(i);
            }

            MatrixXd originTrans;
            g_general_6s->calc_forward_kin(originACS, originTrans);
            VectorXd originMCS = g_general_6s->tr_2_MCS(originTrans);

            VectorXd targetACS(6);
            targetACS = originACS;
            g_general_6s->calc_inverse_kin(g_general_6s->rpy_2_tr(target), originACS, targetACS);
            printf("[底层] MOVEL 目标TCP: %.3f %.3f %.3f %.3f %.3f %.3f -> IK关节: %.3f %.3f %.3f %.3f %.3f %.3f。\n",
                target(0), target(1), target(2), target(3), target(4), target(5),
                targetACS(0), targetACS(1), targetACS(2), targetACS(3), targetACS(4), targetACS(5));

            VectorXd vel_current = VectorXd::Zero(6);
            VectorXd acc_current = VectorXd::Zero(6);
            std::deque<double> new_trajectory;
            g_general_6s->move_joint_interp(
                targetACS, originACS, vel_current, acc_current,
                0.001, 10.0, 10.0, 10.0, 10.0,
                new_trajectory
            );

            {
                std::lock_guard<std::mutex> lock(trajectory_mutex);
                g_general_6s->set_angle_deque(new_trajectory);
            }
            printf("[底层] MOVEL 插补完成，轨迹队列长度=%zu，PowerStatus=%d。\n", new_trajectory.size(), PowerStatus);
        }
        else if (cmd_type == "WRITE") {
            int count = 0;
            double lift_mm = 10.0;
            ss >> count >> lift_mm;

            std::vector<PosePoint> points;
            points.reserve(count > 0 ? count : 0);
            for (int i = 0; i < count; i++) {
                PosePoint point;
                ss >> point.x >> point.y >> point.z >> point.rx >> point.ry >> point.rz;
                if (!ss.fail()) points.push_back(point);
            }

            execute_write_path(points, lift_mm);
        }
        else if (cmd_type == "CARTPATH") {
            int count = 0;
            ss >> count;

            std::vector<PosePoint> points;
            points.reserve(count > 0 ? count : 0);
            for (int i = 0; i < count; i++) {
                PosePoint point;
                ss >> point.x >> point.y >> point.z >> point.rx >> point.ry >> point.rz;
                if (!ss.fail()) points.push_back(point);
            }

            execute_cartesian_path(points);
        }
        else if (cmd_type == "TEACHXYZ") {
            PosePoint point;
            ss >> point.x >> point.y >> point.z;

            std::vector<PosePoint> points;
            if (!ss.fail()) points.push_back(point);

            execute_teach_xyz_path(points);
        }
        else if (cmd_type == "TEACHPATH") {
            int count = 0;
            ss >> count;

            std::vector<PosePoint> points;
            points.reserve(count > 0 ? count : 0);
            for (int i = 0; i < count; i++) {
                PosePoint point;
                ss >> point.x >> point.y >> point.z;
                if (!ss.fail()) points.push_back(point);
            }

            execute_teach_xyz_path(points);
        }
        else if (cmd_type == "GRIPPER") {
            std::string action;
            ss >> action;
            set_gripper(action == "CLOSE" || action == "close" || action == "1");
        }
        else if (cmd_type == "IOSET") {
            int channel = 0;
            int value = 0;
            ss >> channel >> value;
            set_single_io(channel, value);
        }
        else if (cmd_type == "IOCLEAR") {
            clear_all_io();
        }
        // ===== 新增：处理网页发来的急停指令 =====
        else if (cmd_type == "ESTOP") {
            // 构造一个空的队列传进去，直接清空正在执行的轨迹
            std::deque<double> empty_trajectory;
            {
                std::lock_guard<std::mutex> lock(trajectory_mutex);
                g_general_6s->set_angle_deque(empty_trajectory);
            }
            printf("\n[底层-警告] 收到 ESTOP 急停指令！已清空运动队列，机器人紧急刹车！\n\n");
        }
        
        close(new_socket);
    }
}
// ======================= 新增：状态发送线程 (端口9001) =======================
void state_server_thread() {
    int server_fd = socket(AF_INET, SOCK_STREAM, 0);
    struct sockaddr_in address;
    int opt = 1;
    setsockopt(server_fd, SOL_SOCKET, SO_REUSEADDR | SO_REUSEPORT, &opt, sizeof(opt));
    
    address.sin_family = AF_INET;
    address.sin_addr.s_addr = INADDR_ANY;
    address.sin_port = htons(9001);
    bind(server_fd, (struct sockaddr *)&address, sizeof(address));
    listen(server_fd, 3);
    
    printf("[网络] 状态推送端口 9001 启动监听...\n");
    
    while(true) {
        int client_socket = accept(server_fd, NULL, NULL);
        printf("[网络] Python 桥接程序已连接至状态端口。\n");
        
        while(true) {
            VectorXd current_joints(6);
            std::stringstream status_ss;
            
            for(int i=0; i<6; i++) {
                double j_val = g_general_6s->getActPositionAngle(i);
                current_joints(i) = j_val;
                status_ss << j_val << " ";
            }
            
            MatrixXd trans_matrix;
            g_general_6s->calc_forward_kin(current_joints, trans_matrix);
            
            // D-H 参数已经以 mm 为单位配置，这里不能再乘 1000。
            VectorXd tcp_pose = g_general_6s->tr_2_MCS(trans_matrix);
            double x = tcp_pose(0);
            double y = tcp_pose(1);
            double z = tcp_pose(2);
            double rx = tcp_pose(3);
            double ry = tcp_pose(4);
            double rz = tcp_pose(5);
            
            status_ss << x << " " << y << " " << z << " " << rx << " " << ry << " " << rz << "\n";
            std::string msg = status_ss.str();
            
            if (send(client_socket, msg.c_str(), msg.length(), MSG_NOSIGNAL) < 0) break;
            usleep(50000); // 20Hz 发送频率
        }
        close(client_socket);
    }
}

// ======================= 主程序入口 =======================
int start_controller() {
	g_general_6s = new General_6S();
	
	// 1. 启动 EtherCAT 实时底层
	std::thread ecat_thread(StartEC);
	ecat_thread.detach();
	sleep(3); // 等待网络初始化
	
	// 2. 初始化单臂机器人参数并上电
	init_robot_params();
	sleep(5); // 等待上电完成
	
	// 3. 启动网络命令接收线程
	std::thread cmd_thread(command_server_thread);
	cmd_thread.detach();
	
	// 4. 启动状态推送线程
	std::thread state_thread(state_server_thread);
	state_thread.join(); 
	
	return 0;
}

int main(int argc, char* argv[]) {
	start_controller();
	return 0;
}
