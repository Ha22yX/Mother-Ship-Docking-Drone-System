# Repository layout and migration / 仓库分工与迁移

I separated the UWB module from the code that connects it to the drone. The main repository now contains the mother/child controller sketches, Pixhawk communication experiments, sensor integration, and whole-system debugging tools. [UWB-Project](https://github.com/Ha22yX/UWB-Project) keeps the ranging, anchor/tag firmware, position solvers, UWB viewers, wiring notes, and module reference material.

我把 UWB 模块本身和无人机整机接入程序分开了：主仓库保存母机/子机控制、飞控通信、传感器接入和整机调试工具；UWB 仓库继续保存测距、锚点/标签固件、位置解算、UWB 可视化及模块资料。

## Migration map / 迁移对照表

Source: [UWB-Project at b06d21f](https://github.com/Ha22yX/UWB-Project/tree/b06d21fc8239b85ccc11ff70ba6198c6e617122c). The 55 migrated files were copied without changing their contents. Their earlier Git history remains in that repository. Sketch folder names and companion source files were kept together.

本次迁移保留了 55 个文件的原始内容。迁移前的历史仍可通过上面的 UWB 提交链接查看；Arduino 草图的目录名和配套源文件一起保留。

| Previous path in UWB-Project | Current path in this repository | Files |
| --- | --- | ---: |
| `firmware/docking/` | [firmware/docking/](../firmware/docking/) | 22 |
| `firmware/pixhawk/` | [firmware/pixhawk/](../firmware/pixhawk/) | 4 |
| `firmware/openmv/` | [firmware/openmv/](../firmware/openmv/) | 2 |
| `firmware/uwb/uwb_follow_pixhawk/` | [firmware/integration/uwb_follow_pixhawk/](../firmware/integration/uwb_follow_pixhawk/) | 1 |
| `archive/old-main/` | [archive/old-main/](../archive/old-main/) | 23 |
| `tools/debug/sik_debug.py` | [tools/debug/sik_debug.py](../tools/debug/sik_debug.py) | 1 |
| `tools/visualization/world_camera.py` | [tools/visualization/world_camera.py](../tools/visualization/world_camera.py) | 1 |
| `docs/reference/px4_mavlink_docs.md` | [docs/reference/px4_mavlink_docs.md](reference/px4_mavlink_docs.md) | 1 |

`uwb_follow_pixhawk` belongs here because it generates flight-controller commands from UWB positions. The standalone UWB solvers stay in the UWB repository. `world_camera.py` visualizes AprilTag pose data, so it moved with the OpenMV integration experiments.

`uwb_follow_pixhawk` 根据 UWB 位置生成飞控指令，所以归入整机接入实验；独立的 UWB 解算程序仍留在 UWB 仓库。`world_camera.py` 查看的是 AprilTag 位姿，因此随视觉接入实验一起迁入主仓库。

## Python tools

Run these from the main repository root after installing `pip install -r requirement.txt`:

| Script | Use / 用途 |
| --- | --- |
| [tools/debug/sik_debug.py](../tools/debug/sik_debug.py) | SiK telemetry-radio diagnostics; inspect arguments with `python tools/debug/sik_debug.py --help`. SiK 数传链路调试。 |
| [firmware/pixhawk/pixhawk_telem1_sniffer/pixhawk_mavlink_decode.py](../firmware/pixhawk/pixhawk_telem1_sniffer/pixhawk_mavlink_decode.py) | Decode a MAVLink serial stream using `pymavlink`. Check the configured serial port first. 解码飞控串口数据，运行前检查端口。 |
| [tools/visualization/world_camera.py](../tools/visualization/world_camera.py) | View AprilTag position and orientation in 3D. Install `pip install -r tools/requirements.txt` for NumPy, PyQtGraph, Qt, and OpenGL support; check the serial settings before launching. AprilTag 位姿三维查看器，需额外安装可视化依赖。 |
| [archive/old-main/rtk_base_forwarder/rtk_base_forwarder.py](../archive/old-main/rtk_base_forwarder/rtk_base_forwarder.py) | Earlier RTK correction forwarding experiment, retained with the old system code. 早期 RTK 修正数据转发实验，随旧版整机程序归档。 |

The [firmware guide](../firmware/README.md) covers the Arduino sketches. These are individual experiments; moving them into one repository does not mean they have been combined into a fully validated docking controller.
