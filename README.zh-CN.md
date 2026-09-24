<div align="center">
  <h1>Mother-Ship Docking Drone System</h1>
  <p>我的双无人机对接项目：探索如何结合 GPS、UWB 和视觉，实现相对定位与空中对接。</p>

  <p>
    <a href="README.md">English</a>
    &middot;
    <a href="https://isef.rosebeg.com">项目网站</a>
    &middot;
    <a href="https://github.com/Ha22yX/UWB-Project">UWB 模块</a>
    &middot;
    <a href="https://github.com/Ha22yX/OpenMV-AprilTag">视觉模块</a>
  </p>

  <p>
    <img alt="Python: experiments" src="https://img.shields.io/badge/Python-experiments-3776AB?style=for-the-badge&logo=python&logoColor=white" />
    <img alt="PX4 / MAVLink: routing" src="https://img.shields.io/badge/PX4%20/%20MAVLink-routing-2f6f67?style=for-the-badge" />
    <img alt="UWB: relative position" src="https://img.shields.io/badge/UWB-relative%20position-287866?style=for-the-badge" />
    <img alt="AprilTag: terminal vision" src="https://img.shields.io/badge/AprilTag-terminal%20vision-7d73b7?style=for-the-badge" />
    <img alt="状态：尚未完成的原型" src="https://img.shields.io/badge/Status-incomplete%20prototype-b7791f?style=for-the-badge" />
  </p>
</div>

<p align="center">
  <img src=".github/assets/readme-hero.svg" alt="项目概览：结合 GPS、UWB 和视觉，获取子无人机相对母机的位置" width="100%" />
</p>

我想做一个系统，让一架较小的子无人机在空中接近并对接到较大的“母机”上。这个仓库记录了我为此做的硬件、程序和实验。

**当前进度：** 理论工作和相关原理验证已经完成。我已经组装了实体原型，也做过初步飞行测试，包括把两架无人机提前连接后一起起飞的测试。完整的自主接近和空中对接流程仍未完成。

## 为什么做这个项目

我从小就觉得无人机很酷，也一直很喜欢无人机。我想把这个兴趣变成一个能自己设计、组装和测试的项目，于是选择了让两架无人机在空中相互接近并完成对接。

吸引我的一个问题是：能不能在较大的距离范围内，持续获得足够准确的相对位置？对于我想做的对接系统，单靠一种定位方式，很难同时兼顾覆盖范围、精度和可靠性。

GPS 的室外覆盖范围很广，但普通 GPS 本身的精度还不够满足近距离对接的需要。[RTK 可以提高 GNSS 定位精度](https://www.u-blox.com/en/technologies/rtk-real-time-kinematic)，但也需要修正数据和合适的信号条件。UWB 和 AprilTag 视觉可以为这个任务提供更精细的局部测量，不过 UWB 需要在锚点的有效测距范围内工作，摄像头也需要在合适的距离内清楚地看到标记。

所以我想把这几种方法结合起来：远距离接近时使用卫星定位，靠近后用 UWB 获取相对位置，最后用视觉进一步对准。我的目标是探索多传感器融合，让相对位置估计在这些阶段都能派上用场，再把它应用到无人机对接上。这个项目既有我想解决的定位问题，也和我从小的兴趣有关，所以我很喜欢做它。

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

我用 PX4/Pixhawk 做飞行控制，用 ESP32-S3 做通信和传感器接入，用 Python 做调试和数据分析。之前有一些整机程序放在 UWB 仓库里，现在把它们整理到了主项目中：

| 仓库 | 内容 |
| --- | --- |
| 当前仓库 | 母机/子机 ESP-NOW 固件、Pixhawk 通信测试、UWB 与飞控及 OpenMV 接入实验、调试工具、GPS 漂移工具、CAD 文件和项目照片。 |
| [UWB-Project](https://github.com/Ha22yX/UWB-Project) | UWB 模块本身：测距、滤波、位置解算、锚点/标签固件、接线说明和 UWB 可视化。 |
| [OpenMV-AprilTag](https://github.com/Ha22yX/OpenMV-AprilTag) | AprilTag 检测、位置与姿态输出，以及电脑端可视化工具。 |

[固件说明](firmware/README.md)介绍了各个草图的用途，[迁移对照表](docs/repository-layout.md)记录了文件的原位置和新位置，也包括早期版本。

主仓库中的一些文件：

```text
firmware/docking/               母机和子机的 ESP-NOW 控制草图
firmware/pixhawk/               Pixhawk TELEM / MAVLink 实验
firmware/integration/           UWB 位置接入 Pixhawk 的跟随控制实验
firmware/openmv/                ESP32 端的 OpenMV 串口与网页显示实验
tools/                         SiK 数传调试和 AprilTag 位姿可视化
archive/old-main/               早期整机原型和 RTK 转发脚本
docs/repository-layout.md       仓库分工与迁移对照表
Drone Control.py                 早期无人机控制实验
Router_Comm_Test.py              路由通信测试
UDP_MAVLink_Comm_Test.py         MAVLink 双向通信测试
serial_px4_udp_router.py         飞控串口与 UDP 之间的桥接
gps_drift_test/                  GPS 数据记录与绘图工具
hardware/solidworks/             我的 SolidWorks 源文件
.github/assets/                 SVG 概览图，以及原型、组装和飞行测试照片
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

Arduino 草图的使用方式见[固件说明](firmware/README.md)。迁入的 AprilTag 三维查看器需要额外运行 `pip install -r tools/requirements.txt` 安装依赖；各个脚本的用途见[工具说明](docs/repository-layout.md#python-tools)。

运行测试前，需要根据实际硬件确认端口、波特率、网络地址和 PX4 设置。这些程序仍处于研究阶段，用于飞行前需要完成台架检查、拆桨测试和失控保护设置。使用 CAD 文件加工零件前，也需要检查尺寸、材料和紧固件。

## 许可证

[MIT](LICENSE)。
