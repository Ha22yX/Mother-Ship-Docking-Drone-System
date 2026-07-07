# Mother-Ship Docking Drone System

Experimental code for a dual-drone docking project. The repository contains
early Python and communication tests for a parent-child UAV system, including
GPS drift logging, PX4/MAVLink routing, UDP communication, and router-side
communication experiments.

## Contents

- `Drone Control.py` - early drone control script.
- `serial_px4_udp_router.py` - serial/PX4 to UDP routing experiment.
- `UDP_MAVLink_Comm_Test.py` - MAVLink communication test over UDP.
- `Router_Comm_Test.py` - router-side communication test.
- `gps_drift_test/GPS_Drift_Logger.py` - GPS drift logging utility.
- `requirement.txt` - Python dependencies used by the scripts.

## Notes

This is an experiment workspace, not a ready-to-fly autopilot package. Check
ports, baud rates, network addresses, and vehicle safety settings before using
any script with real hardware.

## Setup

```bash
pip install -r requirement.txt
```

Run the individual scripts directly after updating their local configuration.
