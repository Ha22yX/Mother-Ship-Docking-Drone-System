# 母机与子无人机空中对接项目

[English](README.md) · [项目网站](https://isef.rosebeg.com)

我想做一个系统，让一架较小的子无人机在空中接近并对接到较大的“母机”上。这个仓库记录了我为此做的硬件、程序和实验。

**当前进度：** 理论工作和相关原理验证已经完成。我已经组装了实体原型，也做过初步飞行测试，包括把两架无人机提前连接后一起起飞的测试。完整的自主接近和空中对接流程仍未完成。

## 我想实现什么

我想研究的是：当两架无人机都在运动时，其中一架怎样找到另一架，并对准它的对接位置。子无人机需要知道自己相对母机框架的位置，以及距离对接点还有多远。

我的方案是在不同阶段使用不同的传感器：

| 阶段 | 方法 | 用途 |
| --- | --- | --- |
| 初步接近 | GPS / RTK-GPS | 让两架无人机进入同一区域。 |
| 相对定位 | 母机框架上的 UWB 锚点和子机上的标签 | 计算子机相对对接框架的位置。 |
| 近距离对准 | OpenMV 摄像头和 AprilTag 标记 | 估计剩余的位置与角度偏差。 |
| 连接 | PX4 / MAVLink 和电磁对接机构 | 控制最后的接近过程并连接两架无人机。 |

UWB 部分把到多个锚点的距离转换成局部 `x, y, z` 坐标，接近对接点后再通过摄像头进一步对准。完整流程还需要通过实机飞行验证。

## 硬件设计和组装

我用 SolidWorks 设计零件，并组装实体原型进行测试。[CAD 文件](hardware/solidworks/)包括无人机平台、对接框架、电池仓、GPS 支架、管夹和电磁铁连接件。

下面是母机框架与子无人机的整体测试装置。

<p align="center">
  <img src=".github/assets/project-introduction.jpg" alt="母机对接框架与子无人机组成的整体测试装置" width="680" />
</p>

这两张照片记录了我的组装过程：

<table>
  <tr>
    <td align="center" width="50%">
      <img src=".github/assets/drone-assembly-side.jpg" alt="无人机组装侧视图，可以看到中心机架、蓝色支架、接线和下方安装的部件" width="360" />
    </td>
    <td align="center" width="50%">
      <img src=".github/assets/drone-assembly-top.jpg" alt="无人机组装俯视图，可以看到机臂、电机、中心板和电子元件" width="360" />
    </td>
  </tr>
  <tr>
    <td align="center">组装过程中的侧视图。</td>
    <td align="center">组装过程中的机架与电子元件俯视图。</td>
  </tr>
</table>

## 先把两架无人机连接起来测试

在尝试让两架独立飞行的无人机对接之前，我先在地面把它们连接起来，再一起进行飞行测试。我想先检查它们在连接状态下能否安全起飞和飞行。

<p align="center">
  <img src=".github/assets/connected-flight-test.jpg" alt="两架无人机在起飞前已连接，正在进行初步共同飞行测试" width="800" />
</p>

*两架无人机在起飞前已连接，照片记录的是连接状态下的飞行测试。*

这张照片记录了共同飞行的初步测试。自主接近、对准以及在空中完成连接，仍然需要各自完成验证，并最终打通整个流程。

## 目前做到哪一步

| 项目内容 | 进度 |
| --- | --- |
| 理论设计和相关原理验证 | 已完成。 |
| 硬件设计与组装 | 已搭建实体原型，仓库包含 CAD 文件和组装照片。 |
| 两机提前连接后的共同飞行 | 已开展初步测试，用于检查连接状态下能否安全共同飞行。 |
| 完整飞行与对接验证 | 已多次尝试，仍未完成。 |
| 自主空中对接 | 尚未实现。 |

我已经进行了多次飞行测试尝试，但还没有完成一次完整的自主空中对接验证。当前卡住我的问题，是所用的既有开源 RTK-GPS 软件环境中的高度数据不稳定。

在静止测试中，**无人机没有移动，GPS 报告的高度仍然会出现约 ±5 米的跳动**。这使我无法在当前状态下安全地继续接近与对接实验。理论部分已经完成，但高度数据问题和完整实机验证仍待解决。

仓库里有一个 [GPS 漂移记录与绘图工具](gps_drift_test/GPS_Drift_Logger.py)，可以记录位置数据并观察高度变化。

接下来我需要：

1. 解决高度数据问题，检查高度读数是否稳定。
2. 在受控飞行中验证读数，再继续接近与对接测试。
3. 完成并记录整个自主对接流程的验证。

## 程序和工具

我用 PX4/Pixhawk 做飞行控制，用 Python 做通信和数据分析，UWB 模块中用到了 ESP32-S3。定位相关的程序分在另外两个仓库里：

| 仓库 | 内容 |
| --- | --- |
| 当前仓库 | 飞控通信实验、GPS 漂移工具、CAD 文件和项目照片。 |
| [UWB-Project](https://github.com/Ha22yX/UWB-Project) | UWB 测距、滤波、位置解算、ESP32-S3 固件和可视化。 |
| [OpenMV-AprilTag](https://github.com/Ha22yX/OpenMV-AprilTag) | AprilTag 检测、位置与姿态输出，以及电脑端可视化工具。 |

主仓库中的一些文件：

```text
Drone Control.py                 早期无人机控制实验
Router_Comm_Test.py              路由通信测试
UDP_MAVLink_Comm_Test.py         MAVLink 双向通信测试
serial_px4_udp_router.py         飞控串口与 UDP 之间的桥接
gps_drift_test/                  GPS 数据记录与绘图工具
hardware/solidworks/             我的 SolidWorks 源文件
.github/assets/                 原型、组装和飞行测试照片
requirement.txt                 Python 依赖
```

## 运行实验程序

安装 Python 和 Git 后，可以这样准备环境：

```bash
git clone https://github.com/Ha22yX/Mother-Ship-Docking-Drone-System.git
cd Mother-Ship-Docking-Drone-System
pip install -r requirement.txt
```

这些脚本分别用于不同的实验。例如，`serial_px4_udp_router.py` 把飞控串口连接到 UDP，`UDP_MAVLink_Comm_Test.py` 用来检查经过该路由器的通信。可以先查看它们支持的参数：

```bash
python serial_px4_udp_router.py --help
python UDP_MAVLink_Comm_Test.py --help
```

运行测试前，需要根据实际硬件确认端口、波特率、网络地址和 PX4 设置。这些程序仍处于研究阶段，用于飞行前需要完成台架检查、拆桨测试和失控保护设置。使用 CAD 文件加工零件前，也需要检查尺寸、材料和紧固件。

## 许可证

[MIT](LICENSE)。
