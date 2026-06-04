import asyncio
import base64
import json
import math
import os
import re
import threading
import time
import urllib.error
import urllib.request
from contextlib import asynccontextmanager, suppress
from pathlib import Path

import uvicorn
from fastapi import Body, FastAPI, WebSocket, WebSocketDisconnect
from fastapi.responses import JSONResponse, StreamingResponse
from fastapi.staticfiles import StaticFiles

from robot_bridge import get_robot_state, send_command, start_state_thread

BASE_DIR = os.path.dirname(os.path.abspath(__file__))
STATIC_DIR = os.path.join(BASE_DIR, "static")
APP_DIR = Path(__file__).resolve().parent
PROJECT_ROOT = APP_DIR.parent
TEACH_PATH_FILE = PROJECT_ROOT / "teach_path.txt"
QA_PROVIDER = (os.environ.get("QA_PROVIDER") or "openai").strip().lower()
QA_VISION_MODEL = os.environ.get("QA_VISION_MODEL") or os.environ.get("OPENAI_MODEL") or "gpt-4.1-mini"
ZHIPU_BASE_URL = (os.environ.get("ZHIPU_BASE_URL") or "https://api.z.ai/api/paas/v4").rstrip("/")
ZHIPU_MODEL = os.environ.get("ZHIPU_MODEL") or "glm-5v-turbo"
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


def clean_answer_text(raw_answer: str, max_chars: int = 3) -> str:
    text = str(raw_answer or "").strip()
    math_result = re.search(r"(?:等于|答案是|答案为|结果是|结果为)\s*([0-9A-Za-z\u4e00-\u9fff+\-*/×÷=]{1,12})", text)
    if math_result:
        text = math_result.group(1)
    text = re.sub(r"\s+", "", text)
    text = re.sub(r"^(答案是|答案为|结果是|结果为|所以|答[:：]?|答案[:：]?)", "", text)
    text = re.sub(r"[，。！？、；：,.!?;:\"'“”‘’（）()【】\[\]{}<>《》\n\r\t]", "", text)
    text = re.sub(r"[^0-9A-Za-z\u4e00-\u9fff+\-*/×÷=]", "", text)
    if not text:
        text = "看不清"
    return text[: max(1, int(max_chars or 3))]


def _extract_response_text(payload: dict) -> str:
    output_text = payload.get("output_text")
    if isinstance(output_text, str) and output_text.strip():
        return output_text.strip()

    chunks = []
    for item in payload.get("output", []) or []:
        for content in item.get("content", []) or []:
            text = content.get("text")
            if isinstance(text, str):
                chunks.append(text)
    return "\n".join(chunks).strip()


def _call_openai_vision_answer(image_base64: str, max_answer_chars: int) -> tuple[str, str]:
    api_key = os.environ.get("OPENAI_API_KEY")
    if not api_key:
        raise RuntimeError("未配置 OPENAI_API_KEY，无法调用视觉模型")

    prompt = (
        "请读取图片中的手写或印刷题目，并给出最短答案。"
        f"只输出答案本身，不要解释。答案限制在 {max_answer_chars} 个中文字符以内。"
        "如果无法识别，请输出“看不清”。"
    )
    request_payload = {
        "model": QA_VISION_MODEL,
        "input": [
            {
                "role": "user",
                "content": [
                    {"type": "input_text", "text": prompt},
                    {
                        "type": "input_image",
                        "image_url": f"data:image/jpeg;base64,{image_base64}",
                        "detail": "high",
                    },
                ],
            }
        ],
        "max_output_tokens": 80,
    }
    req = urllib.request.Request(
        "https://api.openai.com/v1/responses",
        data=json.dumps(request_payload).encode("utf-8"),
        headers={
            "Authorization": f"Bearer {api_key}",
            "Content-Type": "application/json",
        },
        method="POST",
    )
    try:
        with urllib.request.urlopen(req, timeout=45) as resp:
            payload = json.loads(resp.read().decode("utf-8"))
    except urllib.error.HTTPError as exc:
        detail = exc.read().decode("utf-8", errors="replace")
        raise RuntimeError(f"视觉模型调用失败: HTTP {exc.code} {detail[:300]}")
    except Exception as exc:
        raise RuntimeError(f"视觉模型调用失败: {exc}")

    raw_answer = _extract_response_text(payload)
    return raw_answer, clean_answer_text(raw_answer, max_answer_chars)


def _call_zhipu_answer(
    question_text: str,
    image_base64: str,
    max_answer_chars: int,
) -> tuple[str, str]:
    api_key = os.environ.get("ZHIPU_API_KEY")
    if not api_key:
        raise RuntimeError("未配置 ZHIPU_API_KEY，无法调用智谱 GLM-5V-Turbo")

    question_text = str(question_text or "").strip()
    image_base64 = str(image_base64 or "").strip()
    if not question_text and not image_base64:
        raise RuntimeError("GLM-5V-Turbo 需要 imageBase64 或 questionText")

    prompt = (
        "请读取图片中的手写或印刷题目，并给出最短答案。"
        f"只输出答案本身，不要解释。答案限制在 {max_answer_chars} 个中文字符以内。"
        "如果无法识别，请输出“看不清”。"
    )
    content = []
    if image_base64:
        content.append(
            {
                "type": "image_url",
                "image_url": {"url": f"data:image/jpeg;base64,{image_base64}"},
            }
        )
    if question_text:
        content.append({"type": "text", "text": f"{prompt}\n题目：{question_text}"})
    else:
        content.append({"type": "text", "text": prompt})

    request_payload = {
        "model": ZHIPU_MODEL,
        "messages": [
            {
                "role": "user",
                "content": content,
            },
        ],
        "thinking": {"type": "disabled"},
        "do_sample": False,
        "temperature": 0,
        "max_tokens": 80,
        "stream": False,
    }
    req = urllib.request.Request(
        f"{ZHIPU_BASE_URL}/chat/completions",
        data=json.dumps(request_payload).encode("utf-8"),
        headers={
            "Authorization": f"Bearer {api_key}",
            "Content-Type": "application/json",
        },
        method="POST",
    )
    try:
        with urllib.request.urlopen(req, timeout=45) as resp:
            payload = json.loads(resp.read().decode("utf-8"))
    except urllib.error.HTTPError as exc:
        detail = exc.read().decode("utf-8", errors="replace")
        raise RuntimeError(f"智谱 GLM 调用失败: HTTP {exc.code} {detail[:300]}")
    except Exception as exc:
        raise RuntimeError(f"智谱 GLM 调用失败: {exc}")

    choices = payload.get("choices") or []
    if not choices:
        raise RuntimeError("智谱 GLM 返回结果为空")
    raw_answer = choices[0].get("message", {}).get("content", "")
    return raw_answer, clean_answer_text(raw_answer, max_answer_chars)


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


def _capture_camera_frame():
    if dai is None or cv2 is None:
        raise RuntimeError(f"DepthAI/CV2 未加载: {CAMERA_IMPORT_ERROR}")
    if not camera_lock.acquire(blocking=False):
        raise RuntimeError("相机正在预览或被占用，请先停止相机预览")

    try:
        with dai.Pipeline(dai.Device()) as pipeline:
            cam = pipeline.create(dai.node.Camera).build(dai.CameraBoardSocket.CAM_A)
            capability = dai.ImgFrameCapability()
            capability.size.fixed((640, 360))
            capability.fps.fixed(20)
            output = cam.requestOutput(capability, True)
            queue = output.createOutputQueue(maxSize=1, blocking=False)

            pipeline.start()
            deadline = time.time() + 5.0
            while time.time() < deadline:
                packet = queue.tryGet()
                if packet is None:
                    time.sleep(0.02)
                    continue
                frame = packet.getCvFrame()
                ok, encoded = cv2.imencode(".jpg", frame, [int(cv2.IMWRITE_JPEG_QUALITY), 88])
                if not ok:
                    raise RuntimeError("相机图像编码失败")
                height, width = frame.shape[:2]
                return {
                    "imageBase64": base64.b64encode(encoded.tobytes()).decode("ascii"),
                    "width": int(width),
                    "height": int(height),
                    "capturedAt": int(time.time() * 1000),
                }
            raise RuntimeError("相机采集超时")
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


def _validate_teach_path_payload(payload: dict) -> dict:
    if not isinstance(payload, dict):
        raise ValueError("payload must be an object")

    raw_points = payload.get("points", [])
    if not isinstance(raw_points, list):
        raise ValueError("points must be a list")

    points = []
    for index, point in enumerate(raw_points, start=1):
        if not isinstance(point, dict):
            raise ValueError(f"P{index} must be an object")
        pose = point.get("pose")
        if not isinstance(pose, list) or len(pose) != 6:
            raise ValueError(f"P{index} pose must be a list of 6 numbers")
        clean_pose = []
        for value in pose:
            number = float(value)
            if not math.isfinite(number):
                raise ValueError(f"P{index} pose contains a non-finite number")
            clean_pose.append(number)
        delay_ms = float(point.get("delayMs", 2500))
        if not math.isfinite(delay_ms):
            raise ValueError(f"P{index} delayMs must be finite")
        points.append(
            {
                "pose": clean_pose,
                "gripperClosed": bool(point.get("gripperClosed", False)),
                "delayMs": min(60000, max(300, int(round(delay_ms)))),
                "recordedAt": int(point.get("recordedAt", int(time.time() * 1000))),
            }
        )

    default_delay_ms = float(payload.get("defaultDelayMs", 2500))
    if not math.isfinite(default_delay_ms):
        default_delay_ms = 2500

    return {
        "version": 1,
        "defaultDelayMs": min(60000, max(300, int(round(default_delay_ms)))),
        "points": points,
        "savedAt": int(time.time() * 1000),
    }


@app.get("/teach-path")
def get_teach_path():
    if not TEACH_PATH_FILE.is_file():
        return JSONResponse({"ok": False, "detail": "尚未保存示教链", "points": []}, status_code=404)
    try:
        with TEACH_PATH_FILE.open("r", encoding="utf-8") as fp:
            payload = json.load(fp)
        clean_payload = _validate_teach_path_payload(payload)
    except Exception as exc:
        return JSONResponse({"ok": False, "detail": f"示教链文件读取失败: {exc}"}, status_code=500)
    return JSONResponse({"ok": True, "path": str(TEACH_PATH_FILE), "data": clean_payload})


@app.post("/teach-path")
def save_teach_path(payload: dict = Body(...)):
    try:
        clean_payload = _validate_teach_path_payload(payload)
        with TEACH_PATH_FILE.open("w", encoding="utf-8") as fp:
            json.dump(clean_payload, fp, ensure_ascii=False, indent=2)
            fp.write("\n")
    except ValueError as exc:
        return JSONResponse({"ok": False, "detail": str(exc)}, status_code=400)
    except Exception as exc:
        return JSONResponse({"ok": False, "detail": f"示教链文件保存失败: {exc}"}, status_code=500)
    return JSONResponse({"ok": True, "path": str(TEACH_PATH_FILE), "data": clean_payload})


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


@app.get("/camera/capture")
def camera_capture():
    ok, detail = _camera_available()
    if not ok:
        return JSONResponse({"ok": False, "detail": detail}, status_code=503)
    try:
        payload = _capture_camera_frame()
    except Exception as exc:
        return JSONResponse({"ok": False, "detail": str(exc)}, status_code=503)
    return JSONResponse({"ok": True, **payload})


@app.post("/qa/answer")
def qa_answer(payload: dict = Body(...)):
    provider = str(payload.get("provider") or QA_PROVIDER or "openai").strip().lower()
    question_text = str(payload.get("questionText", "")).strip()
    image_base64 = str(payload.get("imageBase64", "")).strip()
    if image_base64.startswith("data:image"):
        image_base64 = image_base64.split(",", 1)[-1]

    max_answer_chars = int(payload.get("maxAnswerChars", 3) or 3)
    max_answer_chars = min(3, max(1, max_answer_chars))

    if provider in {"zhipu", "glm", "glm-5-turbo", "glm-5v-turbo"}:
        if not question_text and not image_base64:
            return JSONResponse({"ok": False, "detail": "缺少 imageBase64 或 questionText"}, status_code=400)
        try:
            raw_answer, clean_answer = _call_zhipu_answer(question_text, image_base64, max_answer_chars)
        except Exception as exc:
            return JSONResponse({"ok": False, "detail": str(exc)}, status_code=503)
        return JSONResponse(
            {
                "ok": True,
                "provider": "zhipu",
                "model": ZHIPU_MODEL,
                "question": question_text,
                "rawAnswer": raw_answer,
                "cleanAnswer": clean_answer,
                "maxAnswerChars": max_answer_chars,
            }
        )

    if provider != "openai":
        return JSONResponse({"ok": False, "detail": f"不支持的 QA provider: {provider}"}, status_code=400)

    if not image_base64:
        return JSONResponse({"ok": False, "detail": "缺少 imageBase64"}, status_code=400)

    try:
        raw_answer, clean_answer = _call_openai_vision_answer(image_base64, max_answer_chars)
    except Exception as exc:
        return JSONResponse({"ok": False, "detail": str(exc)}, status_code=503)

    return JSONResponse(
        {
            "ok": True,
            "provider": "openai",
            "model": QA_VISION_MODEL,
            "question": question_text,
            "rawAnswer": raw_answer,
            "cleanAnswer": clean_answer,
            "maxAnswerChars": max_answer_chars,
        }
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
