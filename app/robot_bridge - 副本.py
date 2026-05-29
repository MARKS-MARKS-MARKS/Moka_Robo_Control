import threading
import time
import copy

# 纯模拟模式，不连接 C++ 底层
robot_state = {
    "joints": [0.0] * 6,
    "tcp": [0.0] * 6
}

# 用于记录网页发来的目标角度
target_joints = [0.0] * 6
_state_lock = threading.Lock()

def send_command(cmd: str):
    """ 模拟接收指令，解析目标角度 """
    global target_joints
    print(f"[BRIDGE-模拟] -> 收到网页指令: {cmd}")
    try:
        parts = cmd.split()
        if parts[0] == "MOVEJ":
            with _state_lock:
                # 提取指令里的 6 个角度作为目标位置
                target_joints = [float(x) for x in parts[1:7]]
        elif parts[0] == "ESTOP":
            print("[BRIDGE-模拟] 执行急停！")
            with _state_lock:
                # 急停时让目标位置瞬间等于当前位置，停止运动
                target_joints = list(robot_state["joints"])
    except Exception as e:
        print(f"[BRIDGE-模拟] 指令解析错误: {e}")

def _state_loop():
    """ 模拟机器人运动状态的循环 """
    print("[BRIDGE-模拟] 启动纯软件模拟状态线程 (20Hz)...")
    step = 0.5  # 每次循环移动的步长(度)，控制模拟运动的速度
    
    while True:
        with _state_lock:
            # 简单的模拟插补：让当前角度慢慢逼近目标角度
            for i in range(6):
                diff = target_joints[i] - robot_state["joints"][i]
                if abs(diff) > step:
                    robot_state["joints"][i] += step if diff > 0 else -step
                else:
                    robot_state["joints"][i] = target_joints[i]

            # 模拟 TCP 数据 (引入一些简单的联动计算)
            robot_state["tcp"] = [
                450.0 + robot_state["joints"][0] * 2.0,
                0.0 + robot_state["joints"][1] * 2.0,
                300.0 + robot_state["joints"][2] * 2.0,
                3.14 + (robot_state["joints"][3] * 0.01), # RX 随 J4 变化
                0.0 + (robot_state["joints"][4] * 0.01),  # RY 随 J5 变化
                0.0 + (robot_state["joints"][5] * 0.01)   # RZ 随 J6 变化
            ]

        time.sleep(0.05) # 50ms 刷新率 (20Hz)

def start_state_thread():
    t = threading.Thread(target=_state_loop, daemon=True)
    t.start()

def get_robot_state():
    with _state_lock:
        return copy.deepcopy(robot_state)