<div align="center">
  <h1>Mother-Ship Docking Drone System</h1>
  <p>Dual-UAV docking research centered on relative localization between a moving mother UAV and a child UAV.</p>

  <p>
    <a href="README.zh-CN.md">Chinese</a>
    &middot;
    <a href="#quickstart">Quickstart</a>
    &middot;
    <a href="#tech-stack">Tech Stack</a>
    &middot;
    <a href="https://isef.rosebeg.com">Project Website</a>
    &middot;
    <a href="https://github.com/Ha22yX/UWB-Project">UWB Module</a>
    &middot;
    <a href="https://github.com/Ha22yX/OpenMV-AprilTag">Vision Module</a>
  </p>

  <p>
    <img alt="Python: experiments" src="https://img.shields.io/badge/Python-experiments-3776AB?style=for-the-badge&logo=python&logoColor=white" />
    <img alt="PX4 / MAVLink: routing" src="https://img.shields.io/badge/PX4%20/%20MAVLink-routing-2f6f67?style=for-the-badge" />
    <img alt="UWB: relative position" src="https://img.shields.io/badge/UWB-relative%20position-287866?style=for-the-badge" />
    <img alt="AprilTag: terminal vision" src="https://img.shields.io/badge/AprilTag-terminal%20vision-7d73b7?style=for-the-badge" />
    <img alt="Status: research" src="https://img.shields.io/badge/Status-research-6b7f73?style=for-the-badge" />
  </p>
</div>

<p align="center">
  <img src=".github/assets/readme-hero.svg" alt="Mother-Ship Docking Drone System overview image" width="100%" />
</p>

## Why This Exists

Aerial docking cannot be solved by flying to a GPS coordinate. The useful signal is the child drone's position inside the moving mother-drone frame, then turning that relative state into safe PX4/MAVLink experiments.

## Quickstart

```bash
git clone https://github.com/Ha22yX/Mother-Ship-Docking-Drone-System.git
cd Mother-Ship-Docking-Drone-System
pip install -r requirement.txt
python UDP_MAVLink_Comm_Test.py
```

Run individual scripts only after checking ports, baud rates, network addresses, and vehicle safety settings.

## Features

- System-level workspace for GPS drift, PX4/MAVLink, UDP routing, and communication tests.
- Relative localization pipeline: GPS/RTK for approach, UWB for mid-range position, AprilTag for terminal pose.
- Clear companion-module split: UWB-Project handles UWB/embedded tests; OpenMV-AprilTag handles visual pose tests.
- Research-first documentation for the docking architecture and experiment path.

## Tech Stack

| Layer | Technology | Role |
| --- | --- | --- |
| Flight | PX4 / Pixhawk | Autopilot and telemetry experiments. |
| Messaging | MAVLink, MAVSDK, UDP | Vehicle communication and routing tests. |
| Localization | UWB + AprilTag | Relative position and terminal pose references. |
| Analysis | Python, matplotlib | GPS drift logging and experiment scripts. |


## Project Notes

This is the main project. [UWB-Project](https://github.com/Ha22yX/UWB-Project) is the UWB positioning companion repo, and [OpenMV-AprilTag](https://github.com/Ha22yX/OpenMV-AprilTag) is the visual localization companion repo.


## Project Map

```text
gps_drift_test/              GPS drift logger
Drone Control.py             early control experiment
UDP_MAVLink_Comm_Test.py     UDP MAVLink test
serial_px4_udp_router.py     serial-to-UDP routing test
```
