# Unified Mocap Router

This commandline program establishes connection to a motion capture system, 
processes the rigid-body pose information and publishes them for use with
different robotics systems.
```
┌─────────┐             ┌────────────────────────┐
│ Motion  │────────────►│                        │  various    ┌──────────┐
│ Capture │  TCP / UDP  │  Unified Mocap Router  ┼───────────► | Agent(s) |
│ System  │◄────────────┤                        │ protocols   └──────────┘ 
└─────────┘             └────────────────────────┘
```

## Support

### Currently implemented motion capture systems:
* NaturalPoint, Inc. **"OptiTrack"** via its `NatNet SDK` (stable)
* Qualisys, vis its `Qualisys CPP SDK` (experimental)

### Currently implemented agents:

|        Agent              | Means of Publishing                                                                   | Launch Command (Example)                                                             |
|:-------------------------:|---------------------------------------------------------------------------------------|--------------------------------------------------------------------------------------|
| `ROS2`                    | On two ros2 topics `/mocap/pose` and `/mocap/twist`                                   | `./mocap-router optitrack ros2 -f 100 -c enu -r far -s 1 -n far`                           |
| `ROS2PX4`                 | As above + the published on the required PX4 topic `/fmu/in/vehicle_visual_odometry`  | `./mocap-router optitrack ros2px4 -f 100 -c enu -r far -s 1 -n far`                        |
| `udp`                     | Data as a UDP stream (see `agents/udp_agent.hpp` for the structs)                     | `./mocap-router optitrack udp -f 100 -c enu -r far -s 1 -n far -i 192.168.209.100 -p 1234` |
| `ivy`                     | Publish to an IVY bus                                                                 | `./mocap-router optitrack ivy -f 100 -c enu -r far -s 1 -n far`                            |
| `mavlink` (experimental)  | Publish mavlink messages to a UDP endpoint (use mavlink-router to uplink to vehicle!) | `./mocap-router optitrack mavlink -f 100 -c enu -r far -s 1 -n far --autopilot px4`        |
| `console`                 | Only to the terminal (if activated)                                                   | `./mocap-router test console --test_freq 100 -f 10 -c enu -r far -s 1 -n far`              |
| `log`                     | Log to a file (csv only so far)                                                       | `./mocap-router test log --test_freq 100 -f 10 -c enu -r far -s 1 -n far -o ./output.csv`  |

### Currently supported platforms:

Ubuntu >= 22.04 on x86

## Building and Running

### Natively

Prerequisites vary per agent. Currently, these are known:

|   Agent   | Known Prerequisites                                                                      |
|:---------:|------------------------------------------------------------------------------------------|
| `all`     | `libboost-all-dev` installed with `apt`                                       |
| `ivy`     | `ivy-c-dev` installed from `ppa:paparazzi-uav/ppa`                                       |
| `ros2`    | `ros-$ROS_DISTRO-base`, needs to be sourced for compilation (execute `. scripts/source_ros_and_msgs.sh`)                                   |
| `ros2px4` | As above + `px4_msgs` must be sourced to run (execute `. scripts/source_ros_and_msgs.sh`)|

Build for all agents with:
```shell
mkdir build && cd build
cmake .. && make
```

Build only for some agents with (for example):
```shell
mkdir build && cd build
cmake -D'MOCAPS=test;optitrack' -D'AGENTS=console;ivy;ros2;ros2px4' .. && make
```

Run with (see table at beginning on this readme)
```shell
./mocap-router --help
```

## Agent-specific Notes

### MAVLINK

`Unified Mocap Router` emits mavlink over UDP transport.
You can use https://github.com/mavlink-router/mavlink-router to forward these messages to an autopilot endpoint over e.g. serial.

When using `Unified Mocap Router` with PX4 via MAVLINK, the PX4-parameter
[EKF2_EV_NOISE_MD](https://docs.px4.io/main/en/advanced_config/parameter_reference.html#EKF2_EV_NOISE_MD)
must be set to `1`.
This will ensure that the static noise parameters (
[EKF2_EVA_NOISE](https://docs.px4.io/main/en/advanced_config/parameter_reference.html#EKF2_EVA_NOISE)
[EKF2_EVP_NOISE](https://docs.px4.io/main/en/advanced_config/parameter_reference.html#EKF2_EVP_NOISE)
) are used, as `Unified Mocap Router` does not compute a measurement noise covariance estimation.


## How to write your own agent?

To add support for your own agent it has to inheret from the base class `Agent` defined in `agent.hpp` and needs to implement the 

    void publish_data()
function. It _can_ also implement 

    void add_extra_po(boost::program_options::options_description &desc)
    void parse_extra_po(const boost::program_options::variables_map &vm)
Afterward, the agent needs to be added to the `CMakeList.txt` file. A simple example of how to do this is the `ConsoleAgent` defined in `agents/console_agent.hpp`.

## How to add support for a different Motion Capture System?

Please open an Issue and we'll support you!


* 2025-02-28 -- 1.1.0 -- Second Release

## Authors

* Anton Bredenbeck ([@antbre](https://github.com/antbre), Delft University of Technology, a.bredenbeck - @ - tudelft.nl)
* Till Blaha ([@tblaha](https://github.com/tblaha), Delft University of Technology, t.m.blaha - @ - tudelft.nl)

## Structure
```
.
├── agents           # contains the agent definitions
├── include          # headers
├── scripts          # utility scripts
├── src              # implementation of data receiving and pose calculations
├── CMakeLists.txt
└── README.md
```

## License

[![License](https://img.shields.io/badge/License-GPL--3.0--or--later-4398cc.svg?logo=spdx)](https://spdx.org/licenses/GPL-3.0-or-later.html)

The contents of this repository are licensed under **GNU General Public License v3.0** (see `LICENSE` file).

Technische Universiteit Delft hereby disclaims all copyright interest in the
program “Unified Mocap Router" (one line description of the content or function)
written by the Author(s).

Henri Werij, Dean of the Faculty of Aerospace Engineering

© 2024,2025 Anton Bredenbeck, Till Blaha
