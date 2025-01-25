# [MultiContactController](https://github.com/isri-aist/MultiContactController)
Humanoid multi-contact motion controller

[![CI](https://github.com/isri-aist/MultiContactController/actions/workflows/ci.yaml/badge.svg)](https://github.com/isri-aist/MultiContactController/actions/workflows/ci.yaml)
[![Documentation](https://img.shields.io/badge/doxygen-online-brightgreen?logo=read-the-docs&style=flat)](https://isri-aist.github.io/MultiContactController/)
[![LICENSE](https://img.shields.io/github/license/isri-aist/MultiContactController)](https://github.com/isri-aist/MultiContactController/blob/master/LICENSE)

https://user-images.githubusercontent.com/6636600/220029546-ef3d5e66-c06d-417e-aac3-27832299f216.mp4

https://user-images.githubusercontent.com/6636600/218400660-a3200de4-4c8b-492f-97e3-c4b7076b9c6a.mp4

https://user-images.githubusercontent.com/6636600/227840423-618b41c1-cd10-436c-bf14-8af3ea85e3b3.mp4

## Features
- Completely open source! (controller framework: mc_rtc, simulator: Choreonoid, sample robot model: JVRC1)
- Unified control of bipedal walking, multi-contact locomotion including surface/grasp contacts, and jumping motion.
- Easy to switch between centroidal trajectory generation methods implemented in [CentroidalControlCollection](https://github.com/isri-aist/CentroidalControlCollection).
- Automated management with CI: Dynamics simulation is run on CI to verify multi-contact motion.

## Technical details
This controller integrates the following elements related to humanoid multi-contact motion:
- Centroidal trajectory generation based on MPC
- Feedback based on centroidal state
- Wrench distribution
- Damping control in limbs

For more information on the technical details, please see the following papers:
- Papers listed in [CentroidalControlCollection](https://github.com/isri-aist/CentroidalControlCollection)
- Papers listed in [ForceControlCollection](https://github.com/isri-aist/ForceControlCollection)
- M. Murooka, et al., "Centroidal trajectory generation and stabilization based on preview control for humanoid multi-contact motion", IEEE Robotics and Automation Letters, 2022. [10.1109/LRA.2022.3186515](https://doi.org/10.1109/LRA.2022.3186515) [(available here)](https://hal.science/hal-03720407)
- M. Murooka, et al., "Whole-body multi-contact motion control for humanoid robots based on distributed tactile sensors", IEEE Robotics and Automation Letters, 2024. [10.1109/LRA.2024.3475052](https://doi.org/10.1109/LRA.2024.3475052) [(available here)](https://hal.science/hal-04729434v1)


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

5. Setup motion configuration file (optional)
```bash
$ roscd multi_contact_controller
$ cd .github/workflows
$ python ./scripts/mergeConfigs.py ./config/MotionSampleField.yaml ./config/DDP.yaml > ~/.config/mc_rtc/controllers/MultiContactController.yaml
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

## Citation
You can cite this work with:
```bib
@ARTICLE{PreviewControlForMultiContactMotion,
  author={Murooka, Masaki and Morisawa, Mitsuharu and Kanehiro, Fumio},
  journal={IEEE Robotics and Automation Letters}, 
  title={Centroidal Trajectory Generation and Stabilization Based on Preview Control for Humanoid Multi-Contact Motion}, 
  year={2022},
  volume={7},
  number={3},
  pages={8225-8232},
  doi={10.1109/LRA.2022.3186515}
}
```

If you focus on whole-body multi-contact motion using tactile sensors, the following paper is also relevant. You can reproduce the simulation experiments with the code in [this branch](https://github.com/mmurooka/MultiContactController/tree/RAL2024).
```bib
@ARTICLE{MultiContactMotionWithTactileSensors,
  author={Murooka, Masaki and Fukumitsu, Kensuke and Hamze, Marwan and Morisawa, Mitsuharu and Kaminaga, Hiroshi and Kanehiro, Fumio and Yoshida, Eiichi},
  journal={IEEE Robotics and Automation Letters}, 
  title={Whole-Body Multi-Contact Motion Control for Humanoid Robots Based on Distributed Tactile Sensors}, 
  year={2024},
  volume={9},
  number={11},
  pages={10620-10627},
  doi={10.1109/LRA.2024.3475052}
}
```
