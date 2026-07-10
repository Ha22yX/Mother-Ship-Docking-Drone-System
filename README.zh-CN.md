<div align="center">
  <h1>Mother-Ship Docking Drone System</h1>
  <p>面向双无人机空中对接的研究型系统，核心是获取子机相对于运动母机的稳定相对位置。</p>

  <p>
    <a href="README.md">English</a> &middot;
    <a href="https://isef.rosebeg.com">项目网站</a> &middot;
    <a href="https://github.com/Ha22yX/UWB-Project">UWB 附属仓库</a>
  </p>
</div>

<p align="center">
  <img src=".github/assets/readme-hero.svg" alt="母机对接相对定位系统概览图" width="100%">
</p>

本仓库是项目级工作区，保存 PX4/MAVLink、GPS 漂移、UDP 转发、MAVSDK 通信等早期实验脚本。项目真正关注的不是“飞到一个 GPS 坐标”，而是在母机同样处于运动状态时，让子机持续估计自己在母机坐标系中的 `x, y, z`，再把这个相对状态转换成 PX4 可执行的控制目标。

> 状态：研究原型。本仓库不是可直接实飞的完整自动驾驶软件包。

## 核心重点：相对位置获取

空中对接需要的是相对定位，而不是单纯的经纬度。系统按距离分层使用传感器：

| 阶段 | 主要传感器 | 作用 | 控制输出 |
| --- | --- | --- | --- |
| 远距离接近 | GPS / RTK-GPS | 让子机进入母机附近区域 | 全局位置目标 |
| 中距离对准 | UWB | 在母机平台坐标系中求解子机 `x, y, z` | 相对位置或速度目标 |
| 末端对准 | AprilTag 视觉 | 修正最后的横向、垂直和姿态误差 | 视觉伺服误差 |
| 最终对接 | UWB + 视觉 + PX4 遥测 | 低速接近并触发吸附 | position / velocity setpoint |

其中 UWB 是中距离相对定位的核心方案：母机平台布置 4 个 anchor，子机携带 1 个 tag，通过 tag 到各 anchor 的距离求解子机在母机平台坐标系下的位置。

## 技术栈

| 层级 | 技术 | 作用 |
| --- | --- | --- |
| 飞控平台 | PX4 / Pixhawk | 提供无人机遥测，并接收后续控制目标。 |
| 通信 | MAVLink、MAVSDK、pymavlink | 连接飞控、读取遥测、测试心跳、进行串口到 UDP 转发。 |
| 相对定位 | UWB anchors/tag | 获取母机坐标系下的中距离相对 `x, y, z`。 |
| 末端视觉 | AprilTag、OpenMV / camera 方案 | 对接前的高精度末端对准。 |
| 伴随硬件 | ESP32-S3、ESP-NOW 方案 | 嵌入式通信与定位辅助，主要在附属仓库中维护。 |
| 数据分析 | Python、matplotlib | GPS 漂移记录和实验数据可视化。 |

## UWB 相对定位方案

UWB 固件、三边定位、ESP32-S3 测试、可视化和 Pixhawk 集成实验位于：

[Ha22yX/UWB-Project](https://github.com/Ha22yX/UWB-Project)

当前测试几何：

- 4 个 anchor 位于正方形对接平台四条边的中点。
- 平台边长 `35.5 in`，约 `0.9017 m`。
- 坐标原点位于平台中心。
- `an0` 为右侧中点，`an1` 为下侧中点，`an2` 为左侧中点，`an3` 为上侧中点。
- `z` 轴以 anchor 平面上方为正。

求解流程：

1. 读取类似 `an0:0.57m` 的测距输出。
2. 对每个 anchor 距离做中值滤波。
3. 以 anchor 0 为参考线性化测距方程。
4. 使用最小二乘求解 `x, y`。
5. 根据距离残差估计正向 `z`。
6. 输出类似 `[POS] x=0.031 m, y=-0.042 m, z=0.615 m` 的相对位置。

## 仓库内容

```text
.
├── Drone Control.py
├── Router_Comm_Test.py
├── UDP_MAVLink_Comm_Test.py
├── serial_px4_udp_router.py
├── gps_drift_test/
│   └── GPS_Drift_Logger.py
├── requirement.txt
├── README.md
└── README.zh-CN.md
```

| 文件 | 作用 |
| --- | --- |
| `Drone Control.py` | 早期 MAVSDK 连接脚本，用于 PX4/QGroundControl 转发端口连接和 GPS/Home 状态检查。 |
| `serial_px4_udp_router.py` | 自动检测 PX4 MAVLink 串口，并在串口与 UDP 之间转发 MAVLink 数据。 |
| `UDP_MAVLink_Comm_Test.py` | 使用 pymavlink 测试 UDP MAVLink 双向通信、心跳和版本请求。 |
| `Router_Comm_Test.py` | 通过 UDP 转发器读取电池、GPS、健康状态等遥测信息。 |
| `gps_drift_test/GPS_Drift_Logger.py` | 记录 PX4 GPS 样本，保存 CSV，并生成漂移、高度、卫星数量图。 |
| `requirement.txt` | 本仓库 Python 脚本依赖。 |

## 快速开始

安装依赖：

```bash
pip install -r requirement.txt
```

记录 GPS 漂移：

```bash
python gps_drift_test/GPS_Drift_Logger.py --duration 120
```

运行 PX4 串口到 UDP 转发：

```bash
python serial_px4_udp_router.py --port 14540
```

测试 UDP MAVLink 通信：

```bash
python UDP_MAVLink_Comm_Test.py --host 127.0.0.1 --port 14540
```

连接真实硬件前，请确认 PX4 串口、TELEM 波特率、UDP 端口、QGroundControl/MAVSDK 配置、解锁设置、failsafe、RC override 和 kill switch。

## 建议实验顺序

1. 拆除桨叶，在架台上验证 MAVLink 遥测。
2. 记录 GPS 漂移，量化 GPS-only 的误差。
3. 在 [UWB-Project](https://github.com/Ha22yX/UWB-Project) 中验证 anchor/tag 测距和三边定位。
4. 校准 anchor ID、平台尺寸和坐标轴方向。
5. 先可视化 UWB `x, y, z`，再接入飞控控制。
6. 验证 AprilTag 末端位姿。
7. 将相对定位接入融合控制流程。
8. 最后再做低高度、低速度、可人工接管的飞行测试。

## 安全说明

- 架台测试必须拆除桨叶。
- 保留 RC kill switch 和 PX4 failsafe。
- 坐标映射未校准前，不要启用自动跟随或自动对接。
- 先在仿真或低风险环境验证，再实飞。
- 早期测试保持低高度、低速度，并始终有人监督和接管。

## 相关仓库

- [Ha22yX/UWB-Project](https://github.com/Ha22yX/UWB-Project)：UWB 测距、三边定位、ESP32-S3 固件、ESP-NOW 母子机通信、Pixhawk 集成和 OpenMV/AprilTag 可视化。

## License

当前仓库暂未包含 license 文件。若需要作为开源项目复用或分发，建议补充明确 license。
