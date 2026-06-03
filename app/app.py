import asyncio
import json
import math
import os
import threading
import time
from contextlib import asynccontextmanager, suppress
from pathlib import Path

import uvicorn
from fastapi import FastAPI, WebSocket, WebSocketDisconnect
from fastapi.responses import JSONResponse, StreamingResponse
from fastapi.staticfiles import StaticFiles

from robot_bridge import get_robot_state, send_command, start_state_thread

BASE_DIR = os.path.dirname(os.path.abspath(__file__))
STATIC_DIR = os.path.join(BASE_DIR, "static")
APP_DIR = Path(__file__).resolve().parent
PROJECT_ROOT = APP_DIR.parent
ENV_HANZI_DATA_DIR = os.environ.get("HANZI_DATA_DIR")
HANZI_DATA_DIRS = [
    Path(ENV_HANZI_DATA_DIR).expanduser() if ENV_HANZI_DATA_DIR else None,
    PROJECT_ROOT / "data",
    PROJECT_ROOT / "moka wangye" / "data",
    APP_DIR / "static" / "hanzi-data",
]

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


def build_robot_command(payload: dict) -> str:
    msg_type = payload.get("type")

    if msg_type == "estop":
        return "ESTOP"

    target = payload.get("target", [0.0] * 6)
    if not isinstance(target, list) or len(target) != 6:
        raise ValueError("target must be a list of 6 numbers")

    values = []
    for value in target:
        number = float(value)
        if not math.isfinite(number):
            raise ValueError("target contains a non-finite number")
        values.append(number)

    if msg_type == "movej_joints":
        return f"MOVEJ {' '.join(map(str, values))}"
    if msg_type == "movel_cart":
        return f"MOVEL {' '.join(map(str, values))}"
    if msg_type == "write_path":
        points = payload.get("points", [])
        if not isinstance(points, list):
            raise ValueError("points must be a list")
        lift = float(payload.get("lift", 10.0))
        if not math.isfinite(lift):
            raise ValueError("lift must be finite")

        flat_points = []
        valid_count = 0
        for point in points:
            if not isinstance(point, list) or len(point) != 6:
                raise ValueError("each write point must be a list of 6 numbers")
            values = []
            for value in point:
                number = float(value)
                if not math.isfinite(number):
                    raise ValueError("write point contains a non-finite number")
                values.append(number)
            flat_points.extend(map(str, values))
            valid_count += 1
        return f"WRITE {valid_count} {lift} {' '.join(flat_points)}"
    if msg_type == "gripper":
        action = "CLOSE" if payload.get("closed", True) else "OPEN"
        return f"GRIPPER {action}"
    if msg_type == "ioset":
        channel = int(payload.get("channel", 0))
        value = 1 if payload.get("value", False) else 0
        return f"IOSET {channel} {value}"
    if msg_type == "ioclear":
        return "IOCLEAR"

    raise ValueError(f"unsupported command type: {msg_type}")


def _camera_available():
    if dai is None or cv2 is None:
        return False, f"DepthAI/CV2 未加载: {CAMERA_IMPORT_ERROR}"
    try:
        devices = dai.Device.getAllAvailableDevices()
    except Exception as exc:
        return False, str(exc)
    if not devices:
        return False, "未检测到 DepthAI 相机，或当前用户没有 USB 权限"
    return True, str(devices[0])


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
                    b"Content-Type: image/jpeg\r\n\r\n"
                    + encoded.tobytes()
                    + b"\r\n"
                )
    finally:
        camera_lock.release()


@asynccontextmanager
async def lifespan(_: FastAPI):
    os.makedirs(STATIC_DIR, exist_ok=True)
    print("=" * 50)
    print("[SYSTEM] starting robot state bridge thread")
    start_state_thread()
    yield
    print("[SYSTEM] FastAPI service stopped")


app = FastAPI(lifespan=lifespan)


@app.get("/camera/status")
def camera_status():
    ok, detail = _camera_available()
    return JSONResponse({"ok": ok, "detail": detail})


@app.get("/camera/stream")
def camera_stream():
    ok, detail = _camera_available()
    if not ok:
        return JSONResponse({"ok": False, "detail": detail}, status_code=503)
    return StreamingResponse(
        _mjpeg_camera_stream(),
        media_type="multipart/x-mixed-replace; boundary=frame",
    )


@app.get("/hanzi/{char}")
def hanzi_data(char: str):
    if len(char) != 1:
        return JSONResponse(
            {"ok": False, "detail": "只支持查询单个汉字", "char": char},
            status_code=400,
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
                status_code=500,
            )
        if not isinstance(data.get("medians"), list) or not data["medians"]:
            return JSONResponse(
                {"ok": False, "detail": "字库文件缺少有效 medians", "char": char},
                status_code=422,
            )
        return JSONResponse({"ok": True, "char": char, "data": data})

    return JSONResponse(
        {
            "ok": False,
            "detail": f"未找到汉字字库数据: {char}",
            "char": char,
            "checkedDirs": checked_dirs,
        },
        status_code=404,
    )


@app.websocket("/ws")
async def websocket_endpoint(ws: WebSocket):
    await ws.accept()
    print("[WS] web client connected")

    async def status_loop():
        try:
            while True:
                state = get_robot_state()
                await ws.send_text(
                    json.dumps(
                        {
                            "type": "status",
                            "joints": state["joints"],
                            "tcp": state["tcp"],
                            "bridge_connected": state["bridge_connected"],
                            "updated_at": state["updated_at"],
                        },
                        ensure_ascii=False,
                    )
                )
                await asyncio.sleep(0.05)
        except asyncio.CancelledError:
            raise
        except Exception as exc:
            print(f"[WS] status task stopped unexpectedly: {exc}")

    status_task = asyncio.create_task(status_loop())

    try:
        while True:
            raw_data = await ws.receive_text()
            payload = json.loads(raw_data)
            command = build_robot_command(payload)
            send_command(command)
    except WebSocketDisconnect:
        print("[WS] web client disconnected")
    except ValueError as exc:
        await ws.send_text(json.dumps({"type": "error", "message": str(exc)}, ensure_ascii=False))
    except Exception as exc:
        print(f"[WS] message handler error: {exc}")
    finally:
        status_task.cancel()
        with suppress(asyncio.CancelledError):
            await status_task


app.mount("/", StaticFiles(directory=STATIC_DIR, html=True), name="static")


if __name__ == "__main__":
    uvicorn.run(app, host="0.0.0.0", port=9002)
