import copy
import os
import socket
import threading
import time
from typing import List

ROBOT_IP = os.getenv("ROBOT_IP", "127.0.0.1")
CMD_PORT = int(os.getenv("ROBOT_CMD_PORT", "9000"))
STATE_PORT = int(os.getenv("ROBOT_STATE_PORT", "9001"))

robot_state = {
    "joints": [0.0] * 6,
    "tcp": [0.0] * 6,
    "bridge_connected": False,
    "updated_at": 0.0,
}
_state_lock = threading.Lock()
_thread_started = False


def _set_bridge_connected(connected: bool):
    with _state_lock:
        robot_state["bridge_connected"] = connected
        if connected:
            robot_state["updated_at"] = time.time()


def send_command(cmd: str):
    try:
        with socket.create_connection((ROBOT_IP, CMD_PORT), timeout=1.0) as sock:
            sock.sendall(cmd.encode("utf-8"))
        print(f"[BRIDGE] sent command: {cmd}")
    except Exception as exc:
        print(f"[BRIDGE] failed to send command: {exc}")


def _apply_state_values(values: List[float]):
    if len(values) != 12:
        return

    with _state_lock:
        robot_state["joints"] = values[:6]
        robot_state["tcp"] = values[6:12]
        robot_state["bridge_connected"] = True
        robot_state["updated_at"] = time.time()


def _apply_state_line(line: str):
    try:
        values = [float(item) for item in line.split()]
    except ValueError:
        return

    _apply_state_values(values)


def _state_loop():
    while True:
        try:
            with socket.create_connection((ROBOT_IP, STATE_PORT), timeout=2.0) as sock:
                sock.settimeout(2.0)
                _set_bridge_connected(True)
                print(f"[BRIDGE] connected to state port {STATE_PORT}")

                recv_buffer = ""
                while True:
                    chunk = sock.recv(1024)
                    if not chunk:
                        raise ConnectionError("state connection closed")

                    recv_buffer += chunk.decode("utf-8", errors="ignore")
                    while "\n" in recv_buffer:
                        line, recv_buffer = recv_buffer.split("\n", 1)
                        line = line.strip()
                        if line:
                            _apply_state_line(line)
        except Exception as exc:
            _set_bridge_connected(False)
            print(f"[BRIDGE] state loop error, retry in 1s: {exc}")
            time.sleep(1.0)


def start_state_thread():
    global _thread_started

    if _thread_started:
        return

    thread = threading.Thread(target=_state_loop, daemon=True)
    thread.start()
    _thread_started = True


def get_robot_state():
    with _state_lock:
        return copy.deepcopy(robot_state)
