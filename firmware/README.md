# Drone firmware / 无人机固件

These are the ESP32-side sketches I use to explore communication, control, and sensor integration for my docking project. I originally put them in [UWB-Project](https://github.com/Ha22yX/UWB-Project), but they belong with the whole-drone system. Each folder is a separate experiment, not a component that is automatically built into one combined firmware.

这些是我为无人机对接项目写的 ESP32 端通信、控制和传感器接入草图。之前放在 UWB 仓库里，现在随整机项目维护。每个草图目录都是独立的实验，不能把它们全部一起烧录。完整自主空中对接仍未完成验证。

## Sketches / 草图用途

| Folder | Purpose / 用途 |
| --- | --- |
| [docking/UAVDocking_Mother_ESPNOW](docking/UAVDocking_Mother_ESPNOW/) | Mother-side telemetry, web controls, MAVLink, and ESP-NOW commands to the child. 母机端遥测、网页控制，以及与飞控和子机通信。 |
| [docking/UAVDocking_Child_ESPNOW](docking/UAVDocking_Child_ESPNOW/) | Child-side commands, telemetry, follow setpoints, and electromagnet control. 子机端指令、遥测、跟随目标和电磁铁控制。 |
| [pixhawk/pixhawk_telem1_sniffer](pixhawk/pixhawk_telem1_sniffer/) | Inspect the TELEM1 serial stream; includes a Python MAVLink decoder. 查看飞控串口数据，附电脑端解码脚本。 |
| [pixhawk/pixhawk_telem1_mavlink_parser](pixhawk/pixhawk_telem1_mavlink_parser/) | Parse MAVLink telemetry on the ESP32. 在 ESP32 上解析飞控遥测。 |
| [pixhawk/pixhawk_telem1_control_test](pixhawk/pixhawk_telem1_control_test/) | Experiment with commands sent to Pixhawk. 测试向飞控发送控制指令。 |
| [integration/uwb_follow_pixhawk](integration/uwb_follow_pixhawk/) | Solve UWB position and generate MAVLink velocity commands for a follow experiment. 将 UWB 位置解算接入飞控跟随控制。 |
| [openmv/openmv_uart_test](openmv/openmv_uart_test/) | Check the ESP32 serial connection to OpenMV. 测试 ESP32 与 OpenMV 的串口连接。 |
| [openmv/openmv_apriltag_web](openmv/openmv_apriltag_web/) | Receive AprilTag data on the ESP32 and display it on a web page. 在 ESP32 上接收 AprilTag 数据并通过网页显示。 |

The OpenMV folders above contain **ESP32 Arduino sketches**. The camera-side scripts remain in [OpenMV-AprilTag](https://github.com/Ha22yX/OpenMV-AprilTag). UWB-only ranging and position-solving experiments remain in [UWB-Project](https://github.com/Ha22yX/UWB-Project). Earlier whole-drone versions are in [archive/old-main](../archive/old-main/).

上面的 OpenMV 文件夹是 **ESP32 端 Arduino 程序**；摄像头端程序仍在视觉仓库。纯 UWB 测距与解算仍在 UWB 仓库，早期整机版本保存在归档目录。

## Opening a sketch / 打开草图

1. Install the ESP32 board package in Arduino IDE and select the board matching your hardware. These experiments target ESP32-S3 boards.
2. For sketches that include `MAVLink.h`, install the Arduino MAVLink library. ESP-NOW, Wi-Fi, web-server, and hardware-serial support come from the ESP32 core.
3. Open the `.ino` whose name matches its folder. Keep the companion `.h` and `.cpp` files in that same folder; both docking sketches depend on them.
4. Check the sketch's `config.h`, or the constants in its `.ino`, for the actual UART pins, baud rates, IDs, radio settings, and vehicle configuration before building it.

在 Arduino IDE 中选择实际使用的 ESP32-S3 开发板。需要 `MAVLink.h` 的草图还要安装 MAVLink 库。打开与目录同名的 `.ino`，保留同目录全部 `.h`、`.cpp` 文件，并按实际硬件检查引脚、波特率和通信参数。

The [main README](../README.md#where-i-am-now) records the current test status. Moving these files does not change the controller logic or complete the unfinished flight validation. See the [migration map](../docs/repository-layout.md) for their original locations and source revision.
