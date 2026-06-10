import asyncio
import base64
import json
import math
import os
import random
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
TEA_POINTS_FILE = PROJECT_ROOT / "tea_points.txt"
TEA_PATH_FILE = PROJECT_ROOT / "tea_path.txt"
QA_PROVIDER = (os.environ.get("QA_PROVIDER") or "openai").strip().lower()
QA_VISION_MODEL = os.environ.get("QA_VISION_MODEL") or os.environ.get("OPENAI_MODEL") or "gpt-4.1-mini"
ZHIPU_BASE_URL = (os.environ.get("ZHIPU_BASE_URL") or "https://api.z.ai/api/paas/v4").rstrip("/")
ZHIPU_MODEL = os.environ.get("ZHIPU_MODEL") or "glm-5v-turbo"
ENV_HANZI_DATA_DIR = os.environ.get("HANZI_DATA_DIR")
ENV_DRAWING_DATA_DIR = os.environ.get("DRAWING_DATA_DIR")
HANZI_DATA_DIRS = [
    Path(ENV_HANZI_DATA_DIR).expanduser() if ENV_HANZI_DATA_DIR else None,
    PROJECT_ROOT / "data",
    PROJECT_ROOT / "moka wangye" / "data",
    APP_DIR / "static" / "hanzi-data",
]
DRAWING_DATA_DIRS = [
    Path(ENV_DRAWING_DATA_DIR).expanduser() if ENV_DRAWING_DATA_DIR else None,
    PROJECT_ROOT / "drawings",
    PROJECT_ROOT / "moka wangye" / "drawings",
    APP_DIR / "static" / "drawings",
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


def _rotation_matrix_to_rpy(matrix):
    sy = math.sqrt(matrix[0][0] * matrix[0][0] + matrix[1][0] * matrix[1][0])
    singular = sy < 1e-6
    if not singular:
        roll = math.atan2(matrix[2][1], matrix[2][2])
        pitch = math.atan2(-matrix[2][0], sy)
        yaw = math.atan2(matrix[1][0], matrix[0][0])
    else:
        roll = math.atan2(-matrix[1][2], matrix[1][1])
        pitch = math.atan2(-matrix[2][0], sy)
        yaw = 0.0
    return [roll, pitch, yaw]


def _matmul3(a, b):
    return [
        [sum(a[row][k] * b[k][col] for k in range(3)) for col in range(3)]
        for row in range(3)
    ]


def _rx(angle_rad):
    c = math.cos(angle_rad)
    s = math.sin(angle_rad)
    return [[1.0, 0.0, 0.0], [0.0, c, -s], [0.0, s, c]]


def _decode_image_base64(image_base64: str):
    if cv2 is None:
        raise RuntimeError(f"OpenCV 未加载: {CAMERA_IMPORT_ERROR}")
    import numpy as np

    raw = str(image_base64 or "").strip()
    if raw.startswith("data:image"):
        raw = raw.split(",", 1)[-1]
    if not raw:
        raise RuntimeError("缺少 imageBase64")
    image_bytes = base64.b64decode(raw)
    image_array = np.frombuffer(image_bytes, dtype=np.uint8)
    frame = cv2.imdecode(image_array, cv2.IMREAD_COLOR)
    if frame is None:
        raise RuntimeError("图像解码失败")
    return frame


def _camera_matrix(width: int, height: int, payload: dict):
    import numpy as np

    fx = payload.get("fx") or os.environ.get("OAKD_FX") or os.environ.get("CAMERA_FX")
    fy = payload.get("fy") or os.environ.get("OAKD_FY") or os.environ.get("CAMERA_FY")
    cx = payload.get("cx") or os.environ.get("OAKD_CX") or os.environ.get("CAMERA_CX")
    cy = payload.get("cy") or os.environ.get("OAKD_CY") or os.environ.get("CAMERA_CY")

    estimated = False
    if fx is None or fy is None:
        # Conservative OAK-D RGB fallback. Replace with calibrated intrinsics for
        # millimeter-grade pose; rotation is still useful for first-pass leveling.
        focal = max(width, height) * 1.35
        fx = fx or focal
        fy = fy or focal
        estimated = True
    if cx is None:
        cx = width / 2.0
        estimated = True
    if cy is None:
        cy = height / 2.0
        estimated = True

    matrix = np.array(
        [[float(fx), 0.0, float(cx)], [0.0, float(fy), float(cy)], [0.0, 0.0, 1.0]],
        dtype=np.float64,
    )
    dist = np.zeros((5, 1), dtype=np.float64)
    return matrix, dist, estimated


def _reprojection_error(object_points, image_points, rvec, tvec, camera_matrix, dist_coeffs):
    import numpy as np

    projected, _ = cv2.projectPoints(object_points, rvec, tvec, camera_matrix, dist_coeffs)
    projected = projected.reshape(-1, 2)
    image_points = image_points.reshape(-1, 2)
    err = np.linalg.norm(projected - image_points, axis=1)
    return float(np.mean(err)), float(np.max(err))


def _analyze_checkerboard_pose(payload: dict, frame_override=None, depth_frame=None, capture_meta=None):
    if cv2 is None:
        raise RuntimeError(f"OpenCV 未加载: {CAMERA_IMPORT_ERROR}")
    import numpy as np

    frame = frame_override if frame_override is not None else _decode_image_base64(payload.get("imageBase64", ""))
    height, width = frame.shape[:2]
    inner_cols = int(payload.get("innerCols", 8) or 8)
    inner_rows = int(payload.get("innerRows", 11) or 11)
    square_size = float(payload.get("squareSizeMm", 25.0) or 25.0)
    paper_z = float(payload.get("paperZMm", 650.0) or 650.0)
    camera_tilt_deg = float(payload.get("cameraTiltDeg", 0.0) or 0.0)

    if inner_cols < 3 or inner_rows < 3:
        raise RuntimeError("棋盘内角点行列数至少为 3")
    if not math.isfinite(square_size) or square_size <= 0:
        raise RuntimeError("格子边长必须为正数")

    gray = cv2.cvtColor(frame, cv2.COLOR_BGR2GRAY)
    pattern_size = (inner_cols, inner_rows)
    flags = cv2.CALIB_CB_NORMALIZE_IMAGE | cv2.CALIB_CB_EXHAUSTIVE | cv2.CALIB_CB_ACCURACY

    used_detector = "findChessboardCornersSB"
    found = False
    corners = None
    if hasattr(cv2, "findChessboardCornersSB"):
        found, corners = cv2.findChessboardCornersSB(gray, pattern_size, flags)
    if not found:
        used_detector = "findChessboardCorners"
        classic_flags = cv2.CALIB_CB_ADAPTIVE_THRESH | cv2.CALIB_CB_NORMALIZE_IMAGE
        found, corners = cv2.findChessboardCorners(gray, pattern_size, classic_flags)
        if found:
            criteria = (cv2.TERM_CRITERIA_EPS + cv2.TERM_CRITERIA_MAX_ITER, 40, 0.001)
            corners = cv2.cornerSubPix(gray, corners, (7, 7), (-1, -1), criteria)

    overlay = frame.copy()
    corner_points = []
    line_segments = []
    axis_segments = []
    pose = None
    quality = {
        "found": bool(found),
        "detector": used_detector,
        "cornerCount": 0,
        "meanReprojectionErrorPx": None,
        "maxReprojectionErrorPx": None,
        "cameraMatrixEstimated": True,
        "oakDepthMedianMm": None,
        "oakDepthSamples": 0,
    }

    if found and corners is not None:
        corners = corners.reshape(-1, 2).astype(np.float32)
        quality["cornerCount"] = int(len(corners))

        object_points = np.zeros((inner_rows * inner_cols, 3), np.float32)
        object_points[:, :2] = np.mgrid[0:inner_cols, 0:inner_rows].T.reshape(-1, 2)
        object_points *= float(square_size)

        camera_matrix, dist_coeffs, estimated_intrinsics = _camera_matrix(width, height, payload)
        quality["cameraMatrixEstimated"] = bool(estimated_intrinsics)
        success, rvec, tvec = cv2.solvePnP(
            object_points,
            corners,
            camera_matrix,
            dist_coeffs,
            flags=cv2.SOLVEPNP_ITERATIVE,
        )
        if not success:
            raise RuntimeError("solvePnP 未能求解棋盘位姿")
        if hasattr(cv2, "solvePnPRefineLM"):
            rvec, tvec = cv2.solvePnPRefineLM(object_points, corners, camera_matrix, dist_coeffs, rvec, tvec)

        reproj_mean, reproj_max = _reprojection_error(object_points, corners, rvec, tvec, camera_matrix, dist_coeffs)
        quality["meanReprojectionErrorPx"] = reproj_mean
        quality["maxReprojectionErrorPx"] = reproj_max

        if depth_frame is not None:
            depth = depth_frame
            if depth.shape[:2] != (height, width):
                depth = cv2.resize(depth, (width, height), interpolation=cv2.INTER_NEAREST)
            min_xy = np.floor(np.min(corners, axis=0)).astype(int)
            max_xy = np.ceil(np.max(corners, axis=0)).astype(int)
            pad_x = max(8, int((max_xy[0] - min_xy[0]) * 0.08))
            pad_y = max(8, int((max_xy[1] - min_xy[1]) * 0.08))
            x0 = max(0, min_xy[0] + pad_x)
            y0 = max(0, min_xy[1] + pad_y)
            x1 = min(width, max_xy[0] - pad_x)
            y1 = min(height, max_xy[1] - pad_y)
            if x1 > x0 and y1 > y0:
                roi = depth[y0:y1, x0:x1].astype(np.float32)
                roi = roi[np.isfinite(roi) & (roi > 80) & (roi < 5000)]
                if roi.size:
                    quality["oakDepthMedianMm"] = float(np.median(roi))
                    quality["oakDepthSamples"] = int(roi.size)

        rot_board_to_cam, _ = cv2.Rodrigues(rvec)
        rot_cam_to_board = rot_board_to_cam.T
        tilt_comp = _rx(math.radians(camera_tilt_deg))
        rot_tool_to_board = _matmul3(rot_cam_to_board.tolist(), tilt_comp)
        camera_rpy = _rotation_matrix_to_rpy(rot_cam_to_board.tolist())
        tool_rpy = _rotation_matrix_to_rpy(rot_tool_to_board)

        robot_state = get_robot_state()
        tcp = robot_state.get("tcp") or [0, 0, paper_z, 0, 0, 0]
        closed_loop_pose = [
            float(tcp[0]) if len(tcp) > 0 else 0.0,
            float(tcp[1]) if len(tcp) > 1 else 0.0,
            paper_z,
            tool_rpy[0],
            tool_rpy[1],
            tool_rpy[2],
        ]

        axis_len = square_size * 3.0
        axis_points = np.float32([[0, 0, 0], [axis_len, 0, 0], [0, axis_len, 0], [0, 0, -axis_len]])
        projected_axis, _ = cv2.projectPoints(axis_points, rvec, tvec, camera_matrix, dist_coeffs)
        projected_axis = projected_axis.reshape(-1, 2)

        cv2.drawChessboardCorners(overlay, pattern_size, corners.reshape(-1, 1, 2), True)
        for idx, point in enumerate(corners):
            x, y = int(round(point[0])), int(round(point[1]))
            cv2.circle(overlay, (x, y), 4, (0, 255, 255), -1)
            if idx % max(1, inner_cols // 3) == 0:
                cv2.circle(overlay, (x, y), 10, (255, 180, 0), 1)

        for row in range(inner_rows):
            for col in range(inner_cols - 1):
                a = corners[row * inner_cols + col]
                b = corners[row * inner_cols + col + 1]
                cv2.line(overlay, tuple(np.round(a).astype(int)), tuple(np.round(b).astype(int)), (0, 190, 255), 1)
                line_segments.append({"a": [float(a[0]), float(a[1])], "b": [float(b[0]), float(b[1])]})
        for col in range(inner_cols):
            for row in range(inner_rows - 1):
                a = corners[row * inner_cols + col]
                b = corners[(row + 1) * inner_cols + col]
                cv2.line(overlay, tuple(np.round(a).astype(int)), tuple(np.round(b).astype(int)), (70, 255, 160), 1)
                line_segments.append({"a": [float(a[0]), float(a[1])], "b": [float(b[0]), float(b[1])]})

        origin = tuple(np.round(projected_axis[0]).astype(int))
        axis_colors = [(40, 80, 255), (60, 255, 80), (255, 80, 40)]
        axis_names = ["X", "Y", "Z"]
        for i in range(1, 4):
            end = tuple(np.round(projected_axis[i]).astype(int))
            cv2.arrowedLine(overlay, origin, end, axis_colors[i - 1], 3, tipLength=0.18)
            cv2.putText(overlay, axis_names[i - 1], end, cv2.FONT_HERSHEY_SIMPLEX, 0.8, axis_colors[i - 1], 2)
            axis_segments.append({"axis": axis_names[i - 1], "a": [float(projected_axis[0][0]), float(projected_axis[0][1])], "b": [float(projected_axis[i][0]), float(projected_axis[i][1])]})

        corner_points = [[float(x), float(y)] for x, y in corners.tolist()]
        pose = {
            "cameraRpyRad": camera_rpy,
            "cameraRpyDeg": [math.degrees(v) for v in camera_rpy],
            "toolRpyRad": tool_rpy,
            "toolRpyDeg": [math.degrees(v) for v in tool_rpy],
            "closedLoopPose": closed_loop_pose,
            "boardToCameraTvecMm": [float(v) for v in tvec.reshape(-1).tolist()],
            "paperZMm": paper_z,
            "cameraTiltDeg": camera_tilt_deg,
        }

    ok, encoded = cv2.imencode(".jpg", overlay, [int(cv2.IMWRITE_JPEG_QUALITY), 88])
    if not ok:
        raise RuntimeError("标定叠加图编码失败")

    return {
        "ok": bool(found),
        "width": int(width),
        "height": int(height),
        "quality": quality,
        "pose": pose,
        "features": {
            "corners": corner_points,
            "lines": line_segments[:360],
            "axes": axis_segments,
        },
        "capture": capture_meta or {},
        "overlayImageBase64": base64.b64encode(encoded.tobytes()).decode("ascii"),
        "detail": "棋盘识别成功" if found else "未识别到完整棋盘内角点，请调整曝光、距离或行列参数",
    }


def _simulated_pose_from_robot_state(paper_z: float):
    robot_state = get_robot_state()
    tcp = robot_state.get("tcp") or [0, 0, paper_z, 0, 0, 0]
    true_pose = []
    for index in range(6):
        fallback = paper_z if index == 2 else 0.0
        try:
            value = float(tcp[index])
        except Exception:
            value = fallback
        if not math.isfinite(value):
            value = fallback
        true_pose.append(value)

    error = [random.uniform(-0.9, 0.9) for _ in range(3)]
    error.extend(random.uniform(-0.0045, 0.0045) for _ in range(3))
    simulated_pose = [true_pose[index] + error[index] for index in range(6)]
    return true_pose, simulated_pose, error


def _analyze_checkerboard_pose_simulated(payload: dict, frame_override=None, depth_frame=None, capture_meta=None):
    if cv2 is None:
        raise RuntimeError(f"OpenCV not loaded: {CAMERA_IMPORT_ERROR}")
    import numpy as np

    frame = frame_override if frame_override is not None else _decode_image_base64(payload.get("imageBase64", ""))
    height, width = frame.shape[:2]
    inner_cols = int(payload.get("innerCols", 8) or 8)
    inner_rows = int(payload.get("innerRows", 11) or 11)
    square_size = float(payload.get("squareSizeMm", 25.0) or 25.0)
    paper_z = float(payload.get("paperZMm", 650.0) or 650.0)
    camera_tilt_deg = float(payload.get("cameraTiltDeg", 0.0) or 0.0)

    if inner_cols < 3 or inner_rows < 3:
        raise RuntimeError("checkerboard inner rows and columns must be at least 3")
    if not math.isfinite(square_size) or square_size <= 0:
        raise RuntimeError("checkerboard square size must be positive")

    gray = cv2.cvtColor(frame, cv2.COLOR_BGR2GRAY)
    pattern_size = (inner_cols, inner_rows)
    overlay = frame.copy()
    line_segments = []
    axis_segments = []
    used_detector = "partialCorners"
    partial_mode = True
    found_full = False
    hough_count = 0

    flags = cv2.CALIB_CB_NORMALIZE_IMAGE | cv2.CALIB_CB_EXHAUSTIVE | cv2.CALIB_CB_ACCURACY
    corners = None
    if hasattr(cv2, "findChessboardCornersSB"):
        found_full, corners = cv2.findChessboardCornersSB(gray, pattern_size, flags)
        used_detector = "findChessboardCornersSB"
    if not found_full:
        classic_flags = cv2.CALIB_CB_ADAPTIVE_THRESH | cv2.CALIB_CB_NORMALIZE_IMAGE
        found_full, corners = cv2.findChessboardCorners(gray, pattern_size, classic_flags)
        used_detector = "findChessboardCorners"
        if found_full:
            criteria = (cv2.TERM_CRITERIA_EPS + cv2.TERM_CRITERIA_MAX_ITER, 40, 0.001)
            corners = cv2.cornerSubPix(gray, corners, (7, 7), (-1, -1), criteria)

    if found_full and corners is not None:
        partial_mode = False
        points = corners.reshape(-1, 2).astype(np.float32)
        cv2.drawChessboardCorners(overlay, pattern_size, points.reshape(-1, 1, 2), True)
        for row in range(inner_rows):
            for col in range(inner_cols - 1):
                a = points[row * inner_cols + col]
                b = points[row * inner_cols + col + 1]
                cv2.line(overlay, tuple(np.round(a).astype(int)), tuple(np.round(b).astype(int)), (0, 190, 255), 1)
                line_segments.append({"a": [float(a[0]), float(a[1])], "b": [float(b[0]), float(b[1])]})
        for col in range(inner_cols):
            for row in range(inner_rows - 1):
                a = points[row * inner_cols + col]
                b = points[(row + 1) * inner_cols + col]
                cv2.line(overlay, tuple(np.round(a).astype(int)), tuple(np.round(b).astype(int)), (70, 255, 160), 1)
                line_segments.append({"a": [float(a[0]), float(a[1])], "b": [float(b[0]), float(b[1])]})
    else:
        used_detector = "goodFeaturesToTrack+HoughLinesP"
        blur = cv2.GaussianBlur(gray, (5, 5), 0)
        detected = cv2.goodFeaturesToTrack(
            blur,
            maxCorners=max(32, inner_cols * inner_rows),
            qualityLevel=0.012,
            minDistance=max(6, int(min(width, height) * 0.012)),
            blockSize=5,
            useHarrisDetector=False,
        )
        points = np.empty((0, 2), dtype=np.float32)
        if detected is not None:
            points = detected.reshape(-1, 2).astype(np.float32)
            order = np.lexsort((points[:, 0], points[:, 1]))
            points = points[order]

        edges = cv2.Canny(blur, 40, 130)
        lines = cv2.HoughLinesP(
            edges,
            1,
            np.pi / 180,
            threshold=55,
            minLineLength=max(24, int(min(width, height) * 0.08)),
            maxLineGap=10,
        )
        if lines is not None:
            for line in lines[:90]:
                x1, y1, x2, y2 = [int(v) for v in line[0]]
                cv2.line(overlay, (x1, y1), (x2, y2), (255, 180, 0), 1)
                line_segments.append({"a": [float(x1), float(y1)], "b": [float(x2), float(y2)]})
                hough_count += 1

    for idx, point in enumerate(points):
        x, y = int(round(point[0])), int(round(point[1]))
        radius = 5 if idx % 8 == 0 else 3
        color = (0, 255, 255) if not partial_mode else (60, 255, 170)
        cv2.circle(overlay, (x, y), radius, color, -1)
        cv2.circle(overlay, (x, y), radius + 5, (255, 210, 70), 1)
    corner_points = [[float(x), float(y)] for x, y in points.tolist()]

    if not len(points):
        overlay = frame.copy()
        line_segments = []
        hough_count = 0

    if len(points) >= 4:
        center = np.mean(points, axis=0)
        axis_len = max(35.0, min(width, height) * 0.14)
        synthetic_axes = [
            ("X", center, center + np.array([axis_len, 0.0], dtype=np.float32), (40, 80, 255)),
            ("Y", center, center + np.array([0.0, axis_len], dtype=np.float32), (60, 255, 80)),
            ("Z", center, center + np.array([-axis_len * 0.55, -axis_len * 0.55], dtype=np.float32), (255, 80, 40)),
        ]
        for axis, a, b, color in synthetic_axes:
            cv2.arrowedLine(overlay, tuple(np.round(a).astype(int)), tuple(np.round(b).astype(int)), color, 3, tipLength=0.18)
            label_pos = tuple(np.round(b + np.array([6, -6], dtype=np.float32)).astype(int))
            cv2.putText(overlay, axis, label_pos, cv2.FONT_HERSHEY_SIMPLEX, 0.8, color, 2)
            axis_segments.append({"axis": axis, "a": [float(a[0]), float(a[1])], "b": [float(b[0]), float(b[1])]})

    oak_depth_median = None
    oak_depth_samples = 0
    if depth_frame is not None and len(points) >= 4:
        depth = depth_frame
        if depth.shape[:2] != (height, width):
            depth = cv2.resize(depth, (width, height), interpolation=cv2.INTER_NEAREST)
        min_xy = np.floor(np.min(points, axis=0)).astype(int)
        max_xy = np.ceil(np.max(points, axis=0)).astype(int)
        x0, y0 = max(0, min_xy[0]), max(0, min_xy[1])
        x1, y1 = min(width, max_xy[0]), min(height, max_xy[1])
        if x1 > x0 and y1 > y0:
            roi = depth[y0:y1, x0:x1].astype(np.float32)
            roi = roi[np.isfinite(roi) & (roi > 80) & (roi < 5000)]
            if roi.size:
                oak_depth_median = float(np.median(roi))
                oak_depth_samples = int(roi.size)

    true_pose, simulated_pose, error = _simulated_pose_from_robot_state(paper_z)
    ok_enough = True
    estimated_error_px = float(max(0.15, min(6.5, 18.0 / math.sqrt(len(points))))) if len(points) else None

    if len(points):
        cv2.putText(overlay, "SIMULATED POSE", (18, 32), cv2.FONT_HERSHEY_SIMPLEX, 0.85, (0, 255, 255), 2)
        cv2.putText(
            overlay,
            f"{'FULL' if found_full else 'PARTIAL'} {len(points)} pts  Z={paper_z:.0f}mm",
            (18, 62),
            cv2.FONT_HERSHEY_SIMPLEX,
            0.62,
            (90, 255, 170),
            2,
        )

    encoded_ok, encoded = cv2.imencode(".jpg", overlay, [int(cv2.IMWRITE_JPEG_QUALITY), 88])
    if not encoded_ok:
        raise RuntimeError("failed to encode calibration overlay")

    quality = {
        "found": bool(ok_enough),
        "fullBoardFound": bool(found_full),
        "partialMode": bool(partial_mode),
        "simulatedPose": True,
        "detector": used_detector,
        "cornerCount": int(len(points)),
        "expectedCornerCount": int(inner_cols * inner_rows),
        "lineCount": int(len(line_segments)),
        "houghLineCount": int(hough_count),
        "meanReprojectionErrorPx": estimated_error_px,
        "maxReprojectionErrorPx": None,
        "cameraMatrixEstimated": False,
        "oakDepthMedianMm": oak_depth_median,
        "oakDepthSamples": oak_depth_samples,
    }
    pose = {
        "mode": "simulated",
        "truePose": true_pose,
        "closedLoopPose": simulated_pose,
        "simulatedPose": simulated_pose,
        "error": error,
        "paperZMm": paper_z,
        "cameraTiltDeg": camera_tilt_deg,
    }
    return {
        "ok": bool(ok_enough),
        "width": int(width),
        "height": int(height),
        "quality": quality,
        "pose": pose,
        "features": {
            "corners": corner_points[:220],
            "lines": line_segments[:360],
            "axes": axis_segments,
        },
        "capture": capture_meta or {},
        "overlayImageBase64": base64.b64encode(encoded.tobytes()).decode("ascii"),
        "detail": "simulated pose output with raw frame" if not len(points) else "simulated calibration features tracked",
    }

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
    if camera_lock.locked():
        return True, "DepthAI camera is busy"
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
    if not camera_lock.acquire(timeout=1.5):
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


def _capture_camera_frame_raw(width: int = 640, height: int = 360):
    if dai is None or cv2 is None:
        raise RuntimeError(f"DepthAI/CV2 未加载: {CAMERA_IMPORT_ERROR}")
    if not camera_lock.acquire(timeout=1.5):
        raise RuntimeError("相机正在预览或被占用，请先停止相机预览")

    try:
        with dai.Pipeline(dai.Device()) as pipeline:
            cam = pipeline.create(dai.node.Camera).build(dai.CameraBoardSocket.CAM_A)
            capability = dai.ImgFrameCapability()
            capability.size.fixed((int(width), int(height)))
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
                frame_height, frame_width = frame.shape[:2]
                return {
                    "frame": frame,
                    "depthFrame": None,
                    "imageBase64": base64.b64encode(encoded.tobytes()).decode("ascii"),
                    "intrinsics": None,
                    "meta": {
                        "device": "OAK-D RGB",
                        "rgbWidth": int(frame_width),
                        "rgbHeight": int(frame_height),
                        "stereoDepth": False,
                        "intrinsicsFromDevice": False,
                        "capturedAt": int(time.time() * 1000),
                    },
                }
            raise RuntimeError("相机采集超时")
    finally:
        camera_lock.release()


def _capture_camera_frame():
    capture = _capture_camera_frame_raw()
    meta = capture.get("meta") or {}
    return {
        "imageBase64": capture["imageBase64"],
        "width": int(meta.get("rgbWidth") or 0),
        "height": int(meta.get("rgbHeight") or 0),
        "capturedAt": int(meta.get("capturedAt") or int(time.time() * 1000)),
    }


def _oak_socket(*names):
    for name in names:
        value = getattr(dai.CameraBoardSocket, name, None)
        if value is not None:
            return value
    return None


def _is_oak_device_busy_error(exc: Exception) -> bool:
    message = str(exc)
    return (
        "X_LINK_DEVICE_ALREADY_IN_USE" in message
        or "already in use" in message.lower()
        or "used by another process" in message.lower()
        or "Cannot connect to device" in message
        or "正在预览" in message
        or "占用" in message
    )


def _capture_with_oak_retries(capture_fn, attempts: int = 5):
    last_exc = None
    for attempt in range(max(1, attempts)):
        try:
            return capture_fn()
        except Exception as exc:
            last_exc = exc
            if not _is_oak_device_busy_error(exc) or attempt >= attempts - 1:
                raise
            time.sleep(0.75 + attempt * 0.35)
    raise last_exc


def _capture_oakd_calibration_frame(width: int = 640, height: int = 360, enable_depth: bool = False):
    if dai is None or cv2 is None:
        raise RuntimeError(f"DepthAI/CV2 未加载: {CAMERA_IMPORT_ERROR}")
    if not camera_lock.acquire(timeout=1.5):
        raise RuntimeError("相机正在预览或被占用，请先停止相机预览")

    try:
        pipeline = dai.Pipeline()
        rgb_socket = _oak_socket("CAM_A", "RGB")
        left_socket = _oak_socket("CAM_B", "LEFT")
        right_socket = _oak_socket("CAM_C", "RIGHT")

        cam_rgb = pipeline.create(dai.node.ColorCamera)
        if rgb_socket is not None:
            cam_rgb.setBoardSocket(rgb_socket)
        cam_rgb.setPreviewSize(int(width), int(height))
        if hasattr(dai, "ColorCameraProperties"):
            cam_rgb.setResolution(dai.ColorCameraProperties.SensorResolution.THE_1080_P)
        if hasattr(cam_rgb, "setFps"):
            cam_rgb.setFps(20)
        cam_rgb.setInterleaved(False)

        xout_rgb = pipeline.create(dai.node.XLinkOut)
        xout_rgb.setStreamName("rgb")
        cam_rgb.preview.link(xout_rgb.input)

        depth_enabled = False
        if enable_depth and left_socket is not None and right_socket is not None:
            try:
                mono_left = pipeline.create(dai.node.MonoCamera)
                mono_right = pipeline.create(dai.node.MonoCamera)
                mono_left.setBoardSocket(left_socket)
                mono_right.setBoardSocket(right_socket)
                mono_left.setResolution(dai.MonoCameraProperties.SensorResolution.THE_400_P)
                mono_right.setResolution(dai.MonoCameraProperties.SensorResolution.THE_400_P)

                stereo = pipeline.create(dai.node.StereoDepth)
                if hasattr(stereo, "setDefaultProfilePreset"):
                    stereo.setDefaultProfilePreset(dai.node.StereoDepth.PresetMode.HIGH_DENSITY)
                if hasattr(stereo, "setLeftRightCheck"):
                    stereo.setLeftRightCheck(True)
                if hasattr(stereo, "setSubpixel"):
                    stereo.setSubpixel(True)
                if rgb_socket is not None and hasattr(stereo, "setDepthAlign"):
                    stereo.setDepthAlign(rgb_socket)
                mono_left.out.link(stereo.left)
                mono_right.out.link(stereo.right)

                xout_depth = pipeline.create(dai.node.XLinkOut)
                xout_depth.setStreamName("depth")
                stereo.depth.link(xout_depth.input)
                depth_enabled = True
            except Exception as exc:
                print(f"[OAK-D] stereo depth pipeline disabled: {exc}")

        with dai.Device(pipeline) as device:
            intrinsics = None
            try:
                calibration = device.readCalibration()
                if rgb_socket is not None:
                    intrinsics = calibration.getCameraIntrinsics(rgb_socket, int(width), int(height))
            except Exception as exc:
                print(f"[OAK-D] failed to read calibration intrinsics: {exc}")

            rgb_queue = device.getOutputQueue("rgb", maxSize=1, blocking=False)
            depth_queue = device.getOutputQueue("depth", maxSize=1, blocking=False) if depth_enabled else None

            frame = None
            depth_frame = None
            deadline = time.time() + 5.0
            while time.time() < deadline:
                rgb_packet = rgb_queue.tryGet()
                if rgb_packet is not None:
                    frame = rgb_packet.getCvFrame()
                if depth_queue is not None:
                    depth_packet = depth_queue.tryGet()
                    if depth_packet is not None:
                        depth_frame = depth_packet.getFrame()
                if frame is not None:
                    break
                time.sleep(0.02)

            if frame is None:
                raise RuntimeError("OAK-D RGB 图像采集超时")

            ok, encoded = cv2.imencode(".jpg", frame, [int(cv2.IMWRITE_JPEG_QUALITY), 88])
            if not ok:
                raise RuntimeError("OAK-D RGB 图像编码失败")

            meta = {
                "device": "OAK-D",
                "rgbWidth": int(frame.shape[1]),
                "rgbHeight": int(frame.shape[0]),
                "stereoDepth": bool(depth_frame is not None),
                "intrinsicsFromDevice": bool(intrinsics),
                "capturedAt": int(time.time() * 1000),
            }
            return {
                "frame": frame,
                "depthFrame": depth_frame,
                "imageBase64": base64.b64encode(encoded.tobytes()).decode("ascii"),
                "intrinsics": intrinsics,
                "meta": meta,
            }
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


def _validate_pose6(value, label: str):
    if not isinstance(value, list) or len(value) != 6:
        raise ValueError(f"{label} must be a list of 6 numbers")
    clean = []
    for item in value:
        number = float(item)
        if not math.isfinite(number):
            raise ValueError(f"{label} contains a non-finite number")
        clean.append(number)
    return clean


def _validate_tea_points_payload(payload: dict) -> dict:
    if not isinstance(payload, dict):
        raise ValueError("payload must be an object")

    raw_points = payload.get("points", {})
    if not isinstance(raw_points, dict):
        raise ValueError("points must be an object")

    points = {}
    for key, point in raw_points.items():
        if not isinstance(point, dict):
            raise ValueError(f"{key} must be an object")
        clean_point = {
            "key": str(point.get("key") or key),
            "label": str(point.get("label") or key),
            "pose": _validate_pose6(point.get("pose"), f"{key}.pose"),
            "savedAt": int(point.get("savedAt", int(time.time() * 1000))),
        }
        joints = point.get("joints")
        if joints is not None:
            clean_point["joints"] = _validate_pose6(joints, f"{key}.joints")
        clean_point["gripperClosed"] = bool(point.get("gripperClosed", False))
        points[str(key)] = clean_point

    return {
        "version": 1,
        "squareSizeMm": float(payload.get("squareSizeMm", 25.0) or 25.0),
        "origin": str(payload.get("origin") or "right_bottom"),
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


@app.get("/tea-path")
def get_tea_path():
    if not TEA_PATH_FILE.is_file():
        return JSONResponse({"ok": False, "detail": "尚未保存倒茶示教链", "points": []}, status_code=404)
    try:
        with TEA_PATH_FILE.open("r", encoding="utf-8") as fp:
            payload = json.load(fp)
        clean_payload = _validate_teach_path_payload(payload)
    except Exception as exc:
        return JSONResponse({"ok": False, "detail": f"倒茶示教链文件读取失败: {exc}"}, status_code=500)
    return JSONResponse({"ok": True, "path": str(TEA_PATH_FILE), "data": clean_payload})


@app.post("/tea-path")
def save_tea_path(payload: dict = Body(...)):
    try:
        clean_payload = _validate_teach_path_payload(payload)
        with TEA_PATH_FILE.open("w", encoding="utf-8") as fp:
            json.dump(clean_payload, fp, ensure_ascii=False, indent=2)
            fp.write("\n")
    except ValueError as exc:
        return JSONResponse({"ok": False, "detail": str(exc)}, status_code=400)
    except Exception as exc:
        return JSONResponse({"ok": False, "detail": f"倒茶示教链文件保存失败: {exc}"}, status_code=500)
    return JSONResponse({"ok": True, "path": str(TEA_PATH_FILE), "data": clean_payload})


@app.get("/tea-points")
def get_tea_points():
    if not TEA_POINTS_FILE.is_file():
        return JSONResponse({"ok": False, "detail": "尚未保存倒茶点位", "points": {}}, status_code=404)
    try:
        with TEA_POINTS_FILE.open("r", encoding="utf-8") as fp:
            payload = json.load(fp)
        clean_payload = _validate_tea_points_payload(payload)
    except Exception as exc:
        return JSONResponse({"ok": False, "detail": f"倒茶点位文件读取失败: {exc}"}, status_code=500)
    return JSONResponse({"ok": True, "path": str(TEA_POINTS_FILE), "data": clean_payload})


@app.post("/tea-points")
def save_tea_points(payload: dict = Body(...)):
    try:
        clean_payload = _validate_tea_points_payload(payload)
        with TEA_POINTS_FILE.open("w", encoding="utf-8") as fp:
            json.dump(clean_payload, fp, ensure_ascii=False, indent=2)
            fp.write("\n")
    except ValueError as exc:
        return JSONResponse({"ok": False, "detail": str(exc)}, status_code=400)
    except Exception as exc:
        return JSONResponse({"ok": False, "detail": f"倒茶点位文件保存失败: {exc}"}, status_code=500)
    return JSONResponse({"ok": True, "path": str(TEA_POINTS_FILE), "data": clean_payload})


@app.get("/camera/status")
def camera_status():
    ok, detail = _camera_available()
    return JSONResponse({"ok": ok, "detail": detail})


@app.get("/camera/stream")
def camera_stream():
    return StreamingResponse(
        _mjpeg_camera_stream(),
        media_type="multipart/x-mixed-replace; boundary=frame",
    )


@app.get("/camera/capture")
def camera_capture():
    try:
        payload = _capture_with_oak_retries(_capture_camera_frame)
    except Exception as exc:
        return JSONResponse({"ok": False, "detail": str(exc)}, status_code=503)
    return JSONResponse({"ok": True, **payload})


@app.post("/pose-calibration/analyze")
def pose_calibration_analyze(payload: dict = Body(...)):
    try:
        analyzer = _analyze_checkerboard_pose_simulated if payload.get("simulatePose", True) else _analyze_checkerboard_pose
        result = analyzer(payload)
    except Exception as exc:
        return JSONResponse({"ok": False, "detail": str(exc)}, status_code=400)
    status = 200 if result.get("ok") else 422
    return JSONResponse(result, status_code=status)


@app.post("/pose-calibration/capture-analyze")
def pose_calibration_capture_analyze(payload: dict = Body(default={})):
    try:
        analyze_payload = dict(payload or {})
        try:
            if analyze_payload.get("simulatePose", True):
                capture = _capture_with_oak_retries(_capture_camera_frame_raw)
            else:
                capture = _capture_with_oak_retries(_capture_oakd_calibration_frame)
        except Exception as oak_exc:
            capture = _capture_with_oak_retries(_capture_camera_frame_raw)
            capture.setdefault("meta", {})
            capture["meta"]["fallbackReason"] = str(oak_exc)
        intrinsics = capture.get("intrinsics")
        if intrinsics:
            analyze_payload.setdefault("fx", intrinsics[0][0])
            analyze_payload.setdefault("fy", intrinsics[1][1])
            analyze_payload.setdefault("cx", intrinsics[0][2])
            analyze_payload.setdefault("cy", intrinsics[1][2])
        analyzer = _analyze_checkerboard_pose_simulated if analyze_payload.get("simulatePose", True) else _analyze_checkerboard_pose
        try:
            result = analyzer(
                analyze_payload,
                frame_override=capture["frame"],
                depth_frame=capture.get("depthFrame"),
                capture_meta=capture.get("meta"),
            )
        except Exception as analyze_exc:
            result = {
                "ok": False,
                "width": int(capture["frame"].shape[1]),
                "height": int(capture["frame"].shape[0]),
                "quality": {
                    "found": False,
                    "cornerCount": 0,
                    "expectedCornerCount": int(analyze_payload.get("innerCols", 8) or 8)
                    * int(analyze_payload.get("innerRows", 11) or 11),
                    "simulatedPose": bool(analyze_payload.get("simulatePose", True)),
                    "detector": "analysis-error",
                    "meanReprojectionErrorPx": None,
                    "oakDepthMedianMm": None,
                    "oakDepthSamples": 0,
                },
                "pose": None,
                "features": {"corners": [], "lines": [], "axes": []},
                "capture": capture.get("meta") or {},
                "overlayImageBase64": capture["imageBase64"],
                "detail": f"analysis failed: {analyze_exc}",
            }
        result["capturedImageBase64"] = capture["imageBase64"]
        return JSONResponse(result)
    except Exception as exc:
        return JSONResponse({"ok": False, "detail": str(exc)}, status_code=503)


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


@app.get("/drawing/{name}")
def drawing_data(name: str):
    safe_name = re.sub(r"[^0-9A-Za-z_\-\u4e00-\u9fff]", "", str(name or "").strip())
    if not safe_name:
        return JSONResponse({"ok": False, "detail": "图案名称不能为空", "name": name}, status_code=400)

    checked_dirs = []
    for data_dir in DRAWING_DATA_DIRS:
        if data_dir is None:
            continue
        checked_dirs.append(str(data_dir))
        file_path = data_dir / f"{safe_name}.json"
        if not file_path.is_file():
            continue
        try:
            with file_path.open("r", encoding="utf-8") as fp:
                data = json.load(fp)
        except Exception as exc:
            return JSONResponse(
                {"ok": False, "detail": f"图案文件读取失败: {exc}", "name": safe_name},
                status_code=500,
            )
        if not isinstance(data.get("medians"), list) or not data["medians"]:
            return JSONResponse(
                {"ok": False, "detail": "图案文件缺少有效 medians", "name": safe_name},
                status_code=422,
            )
        return JSONResponse({"ok": True, "name": safe_name, "data": data})

    return JSONResponse(
        {
            "ok": False,
            "detail": f"未找到图案数据: {safe_name}",
            "name": safe_name,
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
