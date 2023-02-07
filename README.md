# [MultiContactController](https://github.com/isri-aist/MultiContactController)
Humanoid multi-contact motion controller

[![CI](https://github.com/isri-aist/MultiContactController/actions/workflows/ci-standalone.yaml/badge.svg)](https://github.com/isri-aist/MultiContactController/actions/workflows/ci-standalone.yaml)
[![CI](https://github.com/isri-aist/MultiContactController/actions/workflows/ci-catkin.yaml/badge.svg)](https://github.com/isri-aist/MultiContactController/actions/workflows/ci-catkin.yaml)
[![Documentation](https://img.shields.io/badge/doxygen-online-brightgreen?logo=read-the-docs&style=flat)](https://isri-aist.github.io/MultiContactController/)
[![LICENSE](https://img.shields.io/github/license/isri-aist/MultiContactController)](https://github.com/isri-aist/MultiContactController/blob/master/LICENSE)

## Features
- Completely open source! (controller framework: mc_rtc, simulator: Choreonoid, sample robot model: JVRC1)
- Possible to control bipedal walking and multi-contact motion including surface/grasp contacts in a unified manner.
- Automated management with CI: Dynamics simulation is run on CI to verify multi-contact motion.

## Install

### Requirements
- Compiler supporting C++17
- Tested with `Ubuntu 20.04 / ROS Noetic` and `Ubuntu 18.04 / ROS Melodic`

### Dependencies
This package depends on
- [mc_rtc](https://jrl-umi3218.github.io/mc_rtc)

This package also depends on the following packages. However, manual installation is unnecessary when this package is installed using `wstool` as described in [Controller installation](#controller-installation).
- [QpSolverCollection](https://github.com/isri-aist/QpSolverCollection)
- [ForceControlCollection](https://github.com/isri-aist/ForceControlCollection)
- [TrajectoryCollection](https://github.com/isri-aist/TrajectoryCollection)
- [NMPC](https://github.com/isri-aist/NMPC)
- [CentroidalControlCollection](https://github.com/isri-aist/CentroidalControlCollection)
- [BaselineWalkingController](https://github.com/isri-aist/BaselineWalkingController)

### Preparation
1. (Skip if ROS is already installed.) Install ROS. See [here](http://wiki.ros.org/ROS/Installation) for details.
```bash
$ export ROS_DISTRO=melodic
$ sudo sh -c 'echo "deb http://packages.ros.org/ros/ubuntu $(lsb_release -sc) main" > /etc/apt/sources.list.d/ros-latest.list'
$ wget http://packages.ros.org/ros.key -O - | sudo apt-key add -
$ sudo apt-get update
$ sudo apt-get install ros-${ROS_DISTRO}-ros-base python-catkin-tools python-rosdep
```

2. (Skip if mc_rtc is already installed.) Install mc_rtc. See [here](https://jrl-umi3218.github.io/mc_rtc/tutorials/introduction/installation-guide.html) for details.
```bash
$ curl -1sLf 'https://dl.cloudsmith.io/public/mc-rtc/stable/setup.deb.sh' | sudo -E bash
$ sudo apt-get install libmc-rtc-dev mc-rtc-utils mc-state-observation jvrc-choreonoid libcnoid-dev ros-${ROS_DISTRO}-mc-rtc-plugin ros-${ROS_DISTRO}-mc-rtc-rviz-panel libeigen-qld-dev
```

### Controller installation
1. Setup catkin workspace.
```bash
$ mkdir -p ~/ros/ws_mcc/src
$ cd ~/ros/ws_mcc
$ wstool init src
$ wstool set -t src isri-aist/MultiContactController https://github.com/isri-aist/MultiContactController --git -y
$ wstool update -t src isri-aist/MultiContactController
$ wstool merge -t src src/isri-aist/MultiContactController/depends.rosinstall
$ wstool update -t src
```

2. Install dependent packages.
```bash
$ source /opt/ros/${ROS_DISTRO}/setup.bash
$ rosdep install -y -r --from-paths src --ignore-src
```

3. Build a package.
```bash
$ catkin build multi_contact_controller -DCMAKE_BUILD_TYPE=RelWithDebInfo -DENABLE_QLD=ON --catkin-make-args all tests
```

4. Setup controller
```bash
$ mkdir -p ~/.config/mc_rtc/controllers
$ cp ~/ros/ws_mcc/src/isri-aist/MultiContactController/etc/mc_rtc.yaml ~/.config/mc_rtc/mc_rtc.yaml
```

### Simulation execution
```bash
# Terminal 1
$ source ~/ros/ws_mcc/devel/setup.bash
$ roscore
# Terminal 2
$ source ~/ros/ws_mcc/devel/setup.bash
$ roscd multi_contact_controller/cnoid/project/
$ /usr/share/hrpsys/samples/JVRC1/clear-omninames.sh
$ choreonoid MCC_JVRC1_SampleField.cnoid --start-simulation
# Terminal 3
$ source ~/ros/ws_mcc/devel/setup.bash
$ roslaunch multi_contact_controller display.launch
```
