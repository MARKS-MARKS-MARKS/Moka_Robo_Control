import uvicorn
from fastapi import FastAPI, WebSocket, WebSocketDisconnect
from fastapi.middleware.cors import CORSMiddleware
from fastapi.responses import JSONResponse, StreamingResponse
from fastapi.staticfiles import StaticFiles
from contextlib import asynccontextmanager
from pathlib import Path
import json
import asyncio
import os
import threading
import time
from robot_bridge import send_command, start_state_thread, get_robot_state

try:
    import cv2
    import depthai as dai
except Exception as exc:
    cv2 = None
    dai = None
    CAMERA_IMPORT_ERROR = str(exc)
else:
    CAMERA_IMPORT_ERROR = ""

camera_lock = threading.Lock()
APP_DIR = Path(__file__).resolve().parent
PROJECT_ROOT = APP_DIR.parent
DEFAULT_HANZI_DATA_DIR = PROJECT_ROOT / "data"
LOCAL_HANZI_DATA_DIR = APP_DIR / "static" / "hanzi-data"
ENV_HANZI_DATA_DIR = os.environ.get("HANZI_DATA_DIR")
HANZI_DATA_DIRS = [
    Path(ENV_HANZI_DATA_DIR).expanduser() if ENV_HANZI_DATA_DIR else None,
    DEFAULT_HANZI_DATA_DIR,
    LOCAL_HANZI_DATA_DIR,
]

@asynccontextmanager
async def lifespan(app: FastAPI):
    if not os.path.exists("static"): os.makedirs("static")
    print("="*50)
    print("[系统] 正在启动后台状态监控线程...")
    start_state_thread()
    yield
    print("[系统] 服务器已关闭。")

app = FastAPI(lifespan=lifespan)
app.add_middleware(
    CORSMiddleware,
    allow_origins=[
        "http://127.0.0.1:9002",
        "http://localhost:9002",
        "http://127.0.0.1:9010",
        "http://localhost:9010",
    ],
    allow_methods=["GET"],
    allow_headers=["*"],
)

def _camera_available():
    if dai is None:
        return False, f"DepthAI/CV2 未加载: {CAMERA_IMPORT_ERROR}"
    try:
        devices = dai.Device.getAllAvailableDevices()
    except Exception as exc:
        return False, str(exc)
    if not devices:
        return False, "未检测到 DepthAI 相机，或当前用户没有 USB 权限"
    return True, str(devices[0])

@app.get("/camera/status")
def camera_status():
    ok, detail = _camera_available()
    return JSONResponse({"ok": ok, "detail": detail})

def _mjpeg_camera_stream():
    if dai is None or cv2 is None:
        return
    if not camera_lock.acquire(blocking=False):
        return

    try:
        with dai.Pipeline(dai.Device()) as pipeline:
            cam = pipeline.create(dai.node.Camera).build(dai.CameraBoardSocket.CAM_A)
            capability = dai.ImgFrameCapability()
            capability.size.fixed((640, 360))
            capability.fps.fixed(20)
            output = cam.requestOutput(capability, True)
            queue = output.createOutputQueue(maxSize=1, blocking=False)

            pipeline.start()
            while True:
                packet = queue.tryGet()
                if packet is None:
                    time.sleep(0.02)
                    continue
                frame = packet.getCvFrame()
                ok, encoded = cv2.imencode(".jpg", frame, [int(cv2.IMWRITE_JPEG_QUALITY), 80])
                if not ok:
                    time.sleep(0.02)
                    continue
                yield (
                    b"--frame\r\n"
                    b"Content-Type: image/jpeg\r\n\r\n" +
                    encoded.tobytes() +
                    b"\r\n"
                )
    finally:
        camera_lock.release()

@app.get("/camera/stream")
def camera_stream():
    ok, detail = _camera_available()
    if not ok:
        return JSONResponse({"ok": False, "detail": detail}, status_code=503)
    return StreamingResponse(
        _mjpeg_camera_stream(),
        media_type="multipart/x-mixed-replace; boundary=frame"
    )

@app.get("/hanzi/{char}")
def hanzi_data(char: str):
    if len(char) != 1:
        return JSONResponse(
            {"ok": False, "detail": "只支持查询单个汉字", "char": char},
            status_code=400
        )

    checked_dirs = []
    for data_dir in HANZI_DATA_DIRS:
        if data_dir is None:
            continue
        checked_dirs.append(str(data_dir))
        file_path = data_dir / f"{char}.json"
        if not file_path.is_file():
            continue

        try:
            with file_path.open("r", encoding="utf-8") as fp:
                data = json.load(fp)
        except Exception as exc:
            return JSONResponse(
                {"ok": False, "detail": f"字库文件读取失败: {exc}", "char": char},
                status_code=500
            )

        if not isinstance(data.get("medians"), list) or not data["medians"]:
            return JSONResponse(
                {"ok": False, "detail": "字库文件缺少有效 medians", "char": char},
                status_code=422
            )

        source = "local-static" if data_dir == LOCAL_HANZI_DATA_DIR else "hanzi-writer-data"
        if ENV_HANZI_DATA_DIR and data_dir == Path(ENV_HANZI_DATA_DIR).expanduser():
            source = "env"
        return JSONResponse({"ok": True, "char": char, "source": source, "data": data})

    return JSONResponse(
        {
            "ok": False,
            "detail": f"未找到汉字字库数据: {char}",
            "char": char,
            "checkedDirs": checked_dirs
        },
        status_code=404
    )

@app.websocket("/ws")
async def websocket_endpoint(ws: WebSocket):
    await ws.accept()
    print(f"[WS] 网页客户端已连接")
    
    async def status_loop():
        try:
            while True:
                state = get_robot_state()
                # 必须和网页前端期待的数据格式严格对应
                if state and state.get("joints"):
                    await ws.send_text(json.dumps({
                        "type": "status",
                        "joints": state["joints"],
                        "tcp": state["tcp"]
                    }))
                await asyncio.sleep(0.05)   
        except Exception as e:
            pass

    status_task = asyncio.create_task(status_loop())

    try:
        while True:
            raw_data = await ws.receive_text()
            payload = json.loads(raw_data)
            msg_type = payload.get("type")
            
            # 这里处理网页发来的指令
            if msg_type == "movej_joints":
                target = payload.get("target", [0.0]*6)
                cmd = f"MOVEJ {' '.join(map(str, target))}"
                send_command(cmd)

            elif msg_type == "movel_cart":
                target = payload.get("target", [0.0]*6)
                cmd = f"MOVEL {' '.join(map(str, target))}"
                send_command(cmd)

            elif msg_type == "write_path":
                points = payload.get("points", [])
                lift = float(payload.get("lift", 10.0))
                flat_points = []
                for point in points:
                    if isinstance(point, list) and len(point) == 6:
                        flat_points.extend(map(str, point))
                cmd = f"WRITE {len(points)} {lift} {' '.join(flat_points)}"
                send_command(cmd)

            elif msg_type == "cart_path":
                points = payload.get("points", [])
                flat_points = []
                for point in points:
                    if isinstance(point, list) and len(point) == 6:
                        flat_points.extend(map(str, point))
                cmd = f"CARTPATH {len(points)} {' '.join(flat_points)}"
                send_command(cmd)

            elif msg_type == "gripper":
                action = "CLOSE" if payload.get("closed", True) else "OPEN"
                send_command(f"GRIPPER {action}")

            elif msg_type == "ioset":
                channel = int(payload.get("channel", 0))
                value = 1 if payload.get("value", False) else 0
                send_command(f"IOSET {channel} {value}")

            elif msg_type == "ioclear":
                send_command("IOCLEAR")

            elif msg_type == "estop":
                send_command("ESTOP")

    except WebSocketDisconnect:
        print(f"[WS] 网页客户端断开")
    finally:
        status_task.cancel()

# 确保把你的 index.html 放在 static 文件夹内
app.mount("/", StaticFiles(directory="static", html=True), name="static")

if __name__ == "__main__":
    uvicorn.run(app, host="0.0.0.0", port=9002)
