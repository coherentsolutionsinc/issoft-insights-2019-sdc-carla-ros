# Building Self-Driving Car Architecture with Robot Operating System

**ISsoft Insights 2019 - Workshop** by [AlexeySas](https://github.com/alexeysas) and [OlegKarasik](https://github.com/OlegKarasik)

This workshop goal is to introduce attendee to [Robot Operating System](http://wiki.ros.org/) (ROS) design and create simple self-driving car architecture.

**By completing workshop you will:**

* Learn basics of Robot Operating System (ROS) including: nodes, messages, topics, packages, launches.
* Learn basics about ROS build-tools and infrastructure.
* Learn basics of PID and Stanley controllers.
* Get experience working with CARLA and developing ROS package.
* Have Fun.

# Prerequisites

We have used Windows Virtual Machines ([Standard_NV6](https://docs.microsoft.com/en-us/azure/virtual-machines/windows/sizes-gpu#nv-series)) in Azure to host workshop environment. While it is possible to execute workshop on your local machine here we are listing everything required to run it as we did. 

Here is high-level overview of setup on Windows: 

![](https://user-images.githubusercontent.com/36962980/58967435-d645a280-87bc-11e9-8e70-f19abd86b14b.png)

*The “green” components are what would be installed on Windows and “blue” are what would be installed on Ubuntu*

## Software & Features

Install the following software:

* [7-zip](https://www.7-zip.org/)
* [Anaconda3 with Python 3.7](https://www.anaconda.com/)
* [Visual Studio Code](https://code.visualstudio.com/)
* [VcXsrv](https://sourceforge.net/projects/vcxsrv/)
* [NVIDIA Graphics Driver 425.31](https://docs.microsoft.com/en-us/azure/virtual-machines/windows/n-series-driver-setup#nvidia-grid-drivers)
  > **WARNING** 
  >
  > Installation of "NVIDIA Graphics Driver 425.3" is required only when configuring [Standard_NV6](https://docs.microsoft.com/en-us/azure/virtual-machines/windows/sizes-gpu#nv-series) virtual machines.

Enable the following features:

* [Windows Subsystem for Linux](https://docs.microsoft.com/en-us/windows/wsl/install-win10)

## Downloads

Download the following releases:

* Releases of [CARLA 0.9.3](https://github.com/carla-simulator/carla/releases/tag/0.9.3) (Windows and Linux)
* Release of [CARLA ROS bridge 0.9.3](https://github.com/carla-simulator/ros-bridge/releases/tag/0.9.3)

## Clone

Clone the following repositories:

* [this repository](https://github.com/coherentsolutionsinc/issoft-insights-2019-sdc-carla-ros.git)

## Preparing Ubuntu 18.04

To run the workshop we need to install Ubuntu 18.04 on WSL. Despite traditional way of installing Ubuntu through Windows Store we would install it manually. These steps are extracted from official Microsoft [documentation](https://docs.microsoft.com/en-us/windows/wsl/install-on-server) and adapter to workshop directory structure.

1. Create Directory: `New-Item -Path C:\Workshop -ItemType Directory`
2. Download Ubuntu: `Invoke-WebRequest -Uri https://aka.ms/wsl-ubuntu-1804 -OutFile 'C:\Workshop\Ubuntu.appx' -UseBasicParsing`
3. Rename Download: `Rename-Item 'C:\Workshop\Ubuntu.appx' 'C:\Workshop\Ubuntu.zip'`
4. Extract Archive: `Expand-Archive 'C:\Workshop\Ubuntu.zip' 'C:\Workshop\ubuntu'`

## Preparing Directory structure

Here is the mapping of the above downloads to expected directory structure:

```
C:\Workshop
  - carla
    - PythonAPI <-- move files and directories from `downloads/CARLA_0.9.3.tar.gz/PythonAPI`
  - carla-client
    - PythonAPI <-- move files and directories from `downloads/CARLA_0.9.3.zip/PythonAPI`
    - manual_control.py <-- move `clone/install/carla-client/manual_control.py`
  - carla-ros-bridge
    - catkin_ws
      - src <-- move files and directories from `downloads/ros-bridge-0.9.3.zip`
	  - config <-- overwrite with `settings.yaml` from `clone/install/carla-ros-bridge/config`
  - carla-server <-- move files and directories from `downloads/CARLA_0.9.3.zip`
  - project <-- move files and directories from `clone/src`
  - ubuntu <-- move files and directories from extracted Ubuntu
``` 

When preparations of directory tree is done please execute: `clone/install/install.ps1` script. This script will install Ubuntu to WSL and initialize it with all required software.

The scripts automatically does:

* **Installs** Visual Studio Code Python extension
* **Installs** and **Upgrades** Ubuntu distributive
* **Installs** `python-pip`, `python-protobuf`, `python-scipy` packages and `pip/simple-pid`, `pip/pygame` pip packages
* **Installs** and **Configures** `ros-melodic-desktop-full` package

> NOTE: Installation could take around 1-2 hours

## Executing Workshop

To run the workshop you will need a presentation and guide. Both of these can be found in [documentation](https://github.com/coherentsolutionsinc/issoft-insights-2019-sdc-carla-ros/tree/master/documentation) directory.

# Asknowlegements

We use [ROS](https://github.com/ros), [CARLA Simulator](https://github.com/carla-simulator/carla) as engine and [CARLA ROS bridge](https://github.com/carla-simulator/ros-bridge) to connect ROS to CARLA.

# Authors

This project is owned by Coherent Solutions.

# License

This project is licensed under the MIT License - see the [LICENSE.md](https://github.com/coherentsolutionsinc/issoft-insights-2019-sdc-carla-ros/blob/master/LICENSE.md) for details.
