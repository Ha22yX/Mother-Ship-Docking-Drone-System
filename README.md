# Mother-Ship Docking Drone System

[简体中文](README.zh-CN.md) · [Project website](https://isef.rosebeg.com)

I am building a system that would let a smaller drone approach and dock with a larger “mother” drone in the air. This repository records my hardware, code, and experiments as I work toward that goal.

**Current status:** I have completed the theoretical work and validation of the underlying principles. I have assembled the hardware and carried out preliminary flight tests, including a test with the two drones connected before takeoff. The complete autonomous approach and docking sequence is still unfinished.

## What I am trying to build

I wanted to explore how one drone could find another and line up with it while both are moving. For docking, the smaller drone needs to know its position relative to the mother drone's frame, including how far it is from the docking point.

My design uses different sensors at different stages:

| Stage | Method | What I use it for |
| --- | --- | --- |
| Initial approach | GPS / RTK-GPS | Bring the drones into the same area. |
| Relative positioning | UWB anchors on the mother frame and a tag on the child drone | Calculate the child drone's position relative to the docking frame. |
| Close-range alignment | OpenMV camera and AprilTag markers | Estimate the remaining position and angle offset. |
| Connection | PX4 / MAVLink and an electromagnetic docking mechanism | Control the final approach and connect the two drones. |

The UWB part turns distances from several anchors into a local `x, y, z` estimate. The camera is intended to refine the alignment close to the docking point. The complete sequence still needs to be validated in flight.

## Building the hardware

I designed parts in SolidWorks and assembled physical prototypes for testing. The [CAD files](hardware/solidworks/) include the drone platform, docking frame, battery bay, GPS mounts, clamps, and electromagnet connectors.

The photo below shows the larger test setup with the mother frame and child drone.

<p align="center">
  <img src=".github/assets/project-introduction.jpg" alt="Mother-frame docking structure and child drone in the assembled test setup" width="680" />
</p>

These two photos show my assembly work in more detail:

<table>
  <tr>
    <td align="center" width="50%">
      <img src=".github/assets/drone-assembly-side.jpg" alt="Side view of my drone assembly, showing the central frame, blue mount, wiring, and components underneath" width="360" />
    </td>
    <td align="center" width="50%">
      <img src=".github/assets/drone-assembly-top.jpg" alt="Top view of my drone assembly, showing the arms, motors, central plate, and electronics" width="360" />
    </td>
  </tr>
  <tr>
    <td align="center">Side view during assembly.</td>
    <td align="center">Top view of the frame and electronics during assembly.</td>
  </tr>
</table>

## Testing the two drones while connected

Before attempting docking between independently flying drones, I connected the two drones on the ground and tested them together. I wanted to check whether they could take off and fly safely in that connected configuration.

<p align="center">
  <img src=".github/assets/connected-flight-test.jpg" alt="The two drones airborne during a preliminary test with them already connected before takeoff" width="800" />
</p>

*The two drones during a flight test with them already connected before takeoff.*

This photo records a preliminary test of flying together. Autonomous approach, alignment, and connection in the air still need their own complete validation.

## Where I am now

| Part of the project | Progress |
| --- | --- |
| Theoretical design and validation of the underlying principles | Completed. |
| Hardware design and assembly | Physical prototypes built; CAD files and assembly photos included. |
| Flight with the drones already connected | Preliminary test carried out to investigate whether they could fly together safely. |
| Full flight and docking validation | Incomplete after multiple test attempts. |
| Autonomous mid-air docking | Not yet achieved. |

I have made multiple flight-test attempts, but they have not led to a completed autonomous docking demonstration. My current blocker is unstable altitude data in the existing open-source RTK-GPS software setup I have been using.

During stationary tests, **the reported GPS altitude kept fluctuating by approximately ±5 m even though the drone was not moving**. That makes it unsafe to continue the approach and docking experiments with the current setup. The theoretical work is complete, but the altitude issue and full flight validation still need to be resolved.

The repository includes a [GPS drift logger and plotter](gps_drift_test/GPS_Drift_Logger.py) for recording position data and inspecting altitude changes.

My next steps are to:

1. Resolve the altitude-data problem and check that the height readings are stable.
2. Validate the readings in controlled flight before resuming approach and docking tests.
3. Complete and document the full autonomous docking sequence.

## Code and tools

I use PX4/Pixhawk for flight control, Python for communication and analysis, and ESP32-S3 hardware in the UWB module. The localization code is split into two related repositories:

| Repository | What is in it |
| --- | --- |
| This repository | Flight-controller communication experiments, GPS drift tools, CAD files, and project photos. |
| [UWB-Project](https://github.com/Ha22yX/UWB-Project) | UWB ranging, filtering, position solving, ESP32-S3 firmware, and visualization. |
| [OpenMV-AprilTag](https://github.com/Ha22yX/OpenMV-AprilTag) | AprilTag detection, position and orientation output, and PC visualization tools. |

Some useful files in this repository:

```text
Drone Control.py                 Early drone-control experiment
Router_Comm_Test.py              Router communication test
UDP_MAVLink_Comm_Test.py         Bidirectional MAVLink communication test
serial_px4_udp_router.py         Serial-to-UDP bridge for the flight controller
gps_drift_test/                  GPS recording and plotting tools
hardware/solidworks/             My SolidWorks source files
.github/assets/                 Prototype, assembly, and flight-test photos
requirement.txt                 Python dependencies
```

## Running the experiments

Install Python and Git, then set up the repository:

```bash
git clone https://github.com/Ha22yX/Mother-Ship-Docking-Drone-System.git
cd Mother-Ship-Docking-Drone-System
pip install -r requirement.txt
```

The scripts are individual experiments. For example, `serial_px4_udp_router.py` connects a flight controller's serial link to UDP, and `UDP_MAVLink_Comm_Test.py` checks communication through that router. Their command-line options are available with:

```bash
python serial_px4_udp_router.py --help
python UDP_MAVLink_Comm_Test.py --help
```

Check ports, baud rates, network addresses, and PX4 settings against the actual hardware before running a test. These files are research code; flight use still requires bench checks, propeller-off tests, and failsafe setup. The CAD files also need dimensions, materials, and fasteners checked before fabrication.

## License

[MIT](LICENSE).
