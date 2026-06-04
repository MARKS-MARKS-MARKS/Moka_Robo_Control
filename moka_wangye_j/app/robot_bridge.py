import socket
import threading
import time
import copy

ROBOT_IP = "127.0.0.1"
CMD_PORT = 9000
STATE_PORT = 9001

# 仅保留纯粹的 6 轴数据结构
robot_state = {
    "joints": [0.0] * 6,
    "tcp": [0.0] * 6
}
_state_lock = threading.Lock()

def send_command(cmd: str):
    """ 将指令通过 TCP 发送给 C++ 控制台 """
    last_error = None
    for attempt in range(3):
        try:
            with socket.socket(socket.AF_INET, socket.SOCK_STREAM) as s:
                s.settimeout(1.0)
                s.connect((ROBOT_IP, CMD_PORT))
                s.sendall(cmd.encode('utf-8'))
            print(f"[BRIDGE] -> 已发送底层指令: {cmd}")
            return True
        except Exception as e:
            last_error = e
            time.sleep(0.05)
    print(f"[BRIDGE] 命令发送失败: {last_error}")
    return False

def _state_loop():
    """ 持续监听 C++ 发来的状态数据 """
    while True:
        try:
            with socket.socket(socket.AF_INET, socket.SOCK_STREAM) as s:
                s.settimeout(2.0)
                s.connect((ROBOT_IP, STATE_PORT))
                print("[BRIDGE] 成功连接到底层状态端口 9001")
                
                while True:
                    data = s.recv(1024).decode('utf-8').strip()
                    if not data: break 
                    
                    vals = list(map(float, data.split()))
                    # C++ 那边回传的是 6个关节角 + 6个位姿 = 12个数据
                    if len(vals) == 12:
                        with _state_lock:
                            robot_state["joints"] = vals[0:6]
                            robot_state["tcp"] = vals[6:12]
                    time.sleep(0.02)
        except Exception as e:
            time.sleep(1.0)

def start_state_thread():
    t = threading.Thread(target=_state_loop, daemon=True)
    t.start()

def get_robot_state():
    with _state_lock:
        return copy.deepcopy(robot_state)
