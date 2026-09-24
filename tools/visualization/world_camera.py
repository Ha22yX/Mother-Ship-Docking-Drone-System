# World camera (pyqtgraph)
# 读取串口输出的 AprilTag 位姿，固定 Tag，显示相机 3D 位姿
# 依赖：pip install pyserial pyqtgraph PyQt6

import serial
import time
import math
import threading
from collections import deque

import numpy as np
import pyqtgraph as pg
import pyqtgraph.opengl as gl

PORT = "COM20"
BAUD = 115200

AXIS_LEN = 1
TAG_SIZE_MM = 100.0
POS_SCALE = 1.0     # 位置放大倍率（>1 放大）
FLIP_X = False
FLIP_Y = True
FLIP_Z = True
POS_EMA_ALPHA = 0.2  # 位置平滑系数（0~1，越小越稳）
TAG_ON_WALL = True   # 标签固定在墙面（垂直平面）
STALE_SEC = 0.3
SHOW_TRAIL = True
TRAIL_LEN = 120

lock = threading.Lock()
last_pose = None
last_seen = 0.0
last_line = ""
trail = deque(maxlen=TRAIL_LEN)
pos_ema = None


def parse_line(line):
    if not line or line == "none":
        return None
    try:
        parts = line.split(",")
        d = {}
        for p in parts:
            k, v = p.split("=")
            d[k.strip()] = float(v) if k != "id" else int(v)
        if "id" not in d:
            d["id"] = 0
        if ("rx" not in d) and ("rx_deg" in d):
            d["rx"] = math.radians(d["rx_deg"])
            d["ry"] = math.radians(d["ry_deg"])
            d["rz"] = math.radians(d["rz_deg"])
        return d
    except Exception:
        return None


def rot_matrix(rx, ry, rz):
    cx, sx = math.cos(rx), math.sin(rx)
    cy, sy = math.cos(ry), math.sin(ry)
    cz, sz = math.cos(rz), math.sin(rz)
    Rx = [[1, 0, 0], [0, cx, -sx], [0, sx, cx]]
    Ry = [[cy, 0, sy], [0, 1, 0], [-sy, 0, cy]]
    Rz = [[cz, -sz, 0], [sz, cz, 0], [0, 0, 1]]

    def mm(a, b):
        return [
            [a[0][0]*b[0][0] + a[0][1]*b[1][0] + a[0][2]*b[2][0],
             a[0][0]*b[0][1] + a[0][1]*b[1][1] + a[0][2]*b[2][1],
             a[0][0]*b[0][2] + a[0][1]*b[1][2] + a[0][2]*b[2][2]],
            [a[1][0]*b[0][0] + a[1][1]*b[1][0] + a[1][2]*b[2][0],
             a[1][0]*b[0][1] + a[1][1]*b[1][1] + a[1][2]*b[2][1],
             a[1][0]*b[0][2] + a[1][1]*b[1][2] + a[1][2]*b[2][2]],
            [a[2][0]*b[0][0] + a[2][1]*b[1][0] + a[2][2]*b[2][0],
             a[2][0]*b[0][1] + a[2][1]*b[1][1] + a[2][2]*b[2][1],
             a[2][0]*b[0][2] + a[2][1]*b[1][2] + a[2][2]*b[2][2]],
        ]
    return mm(mm(Rz, Ry), Rx)


def transpose3(R):
    return [
        [R[0][0], R[1][0], R[2][0]],
        [R[0][1], R[1][1], R[2][1]],
        [R[0][2], R[1][2], R[2][2]],
    ]


def apply_R(R, v):
    return (
        R[0][0]*v[0] + R[0][1]*v[1] + R[0][2]*v[2],
        R[1][0]*v[0] + R[1][1]*v[1] + R[1][2]*v[2],
        R[2][0]*v[0] + R[2][1]*v[1] + R[2][2]*v[2],
    )

def apply_flip_vec(v):
    fx = -1 if FLIP_X else 1
    fy = -1 if FLIP_Y else 1
    fz = -1 if FLIP_Z else 1
    return (v[0]*fx, v[1]*fy, v[2]*fz)

def apply_flip_R(R):
    fx = -1 if FLIP_X else 1
    fy = -1 if FLIP_Y else 1
    fz = -1 if FLIP_Z else 1
    F = [[fx,0,0],[0,fy,0],[0,0,fz]]
    # R' = F * R * F
    def mm(a,b):
        return [
            [a[0][0]*b[0][0] + a[0][1]*b[1][0] + a[0][2]*b[2][0],
             a[0][0]*b[0][1] + a[0][1]*b[1][1] + a[0][2]*b[2][1],
             a[0][0]*b[0][2] + a[0][1]*b[1][2] + a[0][2]*b[2][2]],
            [a[1][0]*b[0][0] + a[1][1]*b[1][0] + a[1][2]*b[2][0],
             a[1][0]*b[0][1] + a[1][1]*b[1][1] + a[1][2]*b[2][1],
             a[1][0]*b[0][2] + a[1][1]*b[1][2] + a[1][2]*b[2][2]],
            [a[2][0]*b[0][0] + a[2][1]*b[1][0] + a[2][2]*b[2][0],
             a[2][0]*b[0][1] + a[2][1]*b[1][1] + a[2][2]*b[2][1],
             a[2][0]*b[0][2] + a[2][1]*b[1][2] + a[2][2]*b[2][2]],
        ]
    return mm(mm(F, R), F)


def reader_thread():
    ser = serial.Serial(PORT, BAUD, timeout=1)
    while True:
        line = ser.readline().decode(errors="ignore").strip()
        if line:
            print(line)
            with lock:
                global last_line
                last_line = line
        d = parse_line(line)
        if not d:
            continue
        with lock:
            global last_pose, last_seen
            last_pose = d
            last_seen = time.time()


def make_lines(points, color=(1, 1, 1, 1), width=2):
    return gl.GLLinePlotItem(pos=np.array(points, dtype=float), color=color, width=width, mode="lines")


def main():
    t = threading.Thread(target=reader_thread, daemon=True)
    t.start()

    pg.mkQApp("Camera Pose (Tag Fixed)")
    view = gl.GLViewWidget()
    view.setWindowTitle("Camera Pose (Tag Fixed)")
    view.opts["distance"] = 600
    view.show()

    grid = gl.GLGridItem()
    grid.scale(10, 10, 1)
    view.addItem(grid)

    # Tag 坐标系与平面
    tag_axes = [
        make_lines([[0, 0, 0], [AXIS_LEN, 0, 0]], color=(1, 0, 0, 1)),
        make_lines([[0, 0, 0], [0, AXIS_LEN, 0]], color=(0, 1, 0, 1)),
        make_lines([[0, 0, 0], [0, 0, AXIS_LEN]], color=(0, 0, 1, 1)),
    ]
    for item in tag_axes:
        view.addItem(item)

    hs = TAG_SIZE_MM * 0.5
    if TAG_ON_WALL:
        # 墙面：标签在 YZ 平面，法线沿 +X
        tag_plane_pts = [
            [0, -hs, -hs], [0, hs, -hs],
            [0, hs, -hs], [0, hs, hs],
            [0, hs, hs], [0, -hs, hs],
            [0, -hs, hs], [0, -hs, -hs],
        ]
    else:
        # 地面：标签在 XY 平面，法线沿 +Z
        tag_plane_pts = [
            [-hs, -hs, 0], [hs, -hs, 0],
            [hs, -hs, 0], [hs, hs, 0],
            [hs, hs, 0], [-hs, hs, 0],
            [-hs, hs, 0], [-hs, -hs, 0],
        ]
    tag_plane = make_lines(tag_plane_pts, color=(0, 0, 0, 1), width=1)
    view.addItem(tag_plane)

    # 相机坐标轴与视锥
    cam_x = make_lines([[0, 0, 0], [AXIS_LEN, 0, 0]], color=(1, 0, 0, 1))
    cam_y = make_lines([[0, 0, 0], [0, AXIS_LEN, 0]], color=(0, 1, 0, 1))
    cam_z = make_lines([[0, 0, 0], [0, 0, AXIS_LEN]], color=(0, 0, 1, 1))
    cam_frustum = make_lines([[0, 0, 0], [0, 0, 0]], color=(0.6, 0.6, 0.6, 1), width=1)
    trail_line = make_lines([[0, 0, 0], [0, 0, 0]], color=(1, 0.5, 0, 1), width=2)
    view.addItem(cam_x)
    view.addItem(cam_y)
    view.addItem(cam_z)
    view.addItem(cam_frustum)
    view.addItem(trail_line)

    def update():
        now = time.time()
        with lock:
            pose = last_pose
            seen = last_seen
        if not pose or (now - seen) > STALE_SEC:
            return

        tx, ty, tz = pose["tx"], pose["ty"], pose["tz"]
        R_tag_in_cam = rot_matrix(pose["rx"], pose["ry"], pose["rz"])
        R_cam_in_tag = transpose3(R_tag_in_cam)
        cam_pos = apply_R(R_cam_in_tag, (-tx, -ty, -tz))
        cam_pos = apply_flip_vec(cam_pos)
        cam_pos = (cam_pos[0]*POS_SCALE, cam_pos[1]*POS_SCALE, cam_pos[2]*POS_SCALE)
        # 平滑位置，减少旋转导致的抖动
        global pos_ema
        if pos_ema is None:
            pos_ema = cam_pos
        else:
            pos_ema = (
                pos_ema[0] * (1 - POS_EMA_ALPHA) + cam_pos[0] * POS_EMA_ALPHA,
                pos_ema[1] * (1 - POS_EMA_ALPHA) + cam_pos[1] * POS_EMA_ALPHA,
                pos_ema[2] * (1 - POS_EMA_ALPHA) + cam_pos[2] * POS_EMA_ALPHA,
            )
        cam_pos = pos_ema
        R_cam_in_tag = apply_flip_R(R_cam_in_tag)

        ex = apply_R(R_cam_in_tag, (AXIS_LEN, 0, 0))
        ey = apply_R(R_cam_in_tag, (0, AXIS_LEN, 0))
        ez = apply_R(R_cam_in_tag, (0, 0, AXIS_LEN))

        cam_x.setData(pos=np.array([cam_pos, (cam_pos[0]+ex[0], cam_pos[1]+ex[1], cam_pos[2]+ex[2])]))
        cam_y.setData(pos=np.array([cam_pos, (cam_pos[0]+ey[0], cam_pos[1]+ey[1], cam_pos[2]+ey[2])]))
        cam_z.setData(pos=np.array([cam_pos, (cam_pos[0]+ez[0], cam_pos[1]+ez[1], cam_pos[2]+ez[2])]))

        # 视锥
        L = AXIS_LEN * 1.2
        H = AXIS_LEN * 0.4
        corners_cam = [
            (-H, -H, L), (H, -H, L), (H, H, L), (-H, H, L)
        ]
        corners = [apply_R(R_cam_in_tag, c) for c in corners_cam]
        corners = [(cam_pos[0]+c[0], cam_pos[1]+c[1], cam_pos[2]+c[2]) for c in corners]
        frustum_pts = [
            cam_pos, corners[0], cam_pos, corners[1], cam_pos, corners[2], cam_pos, corners[3],
            corners[0], corners[1], corners[1], corners[2], corners[2], corners[3], corners[3], corners[0]
        ]
        cam_frustum.setData(pos=np.array(frustum_pts))

        if SHOW_TRAIL:
            trail.append(cam_pos)
            if len(trail) >= 2:
                trail_line.setData(pos=np.array(list(trail)))

    timer = pg.QtCore.QTimer()
    timer.timeout.connect(update)
    timer.start(16)

    pg.exec()


if __name__ == "__main__":
    main()

