# FTI-FT-WAM

Integration of ATI/FTI Mini40 force/torque sensors with Barrett WAM robotic arms using C++ and libbarrett.

This repository provides utilities and examples for reading, transforming, filtering, and using force/torque measurements on WAM manipulators for robotics research applications such as force control, teleoperation, and interaction control.

---

## Features

- ATI Mini40 force/torque sensor integration
- NI-DAQ based acquisition
- libbarrett system integration
- Real-time force/torque streaming
- Frame transformations
- Jacobian-based force/torque mapping
- Signal filtering
- CSV logging

---

## Dependencies

- Ubuntu 20.04
- C++
- libbarrett
- Eigen
- CMake
- NI-DAQmx
- ROS (optional)

---

## Build

```bash
git clone git@github.com:amir-noohian/fti-ft-wam.git
cd fti-ft-wam

mkdir build
cd build

cmake ..
make -j
