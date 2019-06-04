# Building Self-Driving Car Architecture with Robot Operating System

**ISsoft Insights 2019 - Workshop** by [AlexeySas](https://github.com/alexeysas) and [OlegKarasik](https://github.com/OlegKarasik)

This workshop goal is to introduce attendee to [Robot Operating System](http://wiki.ros.org/) (ROS) design and create simple self-driving car architecture.

## Preparation

We have used Windows Virtual Machines in Azure to host workshop environment. While it is possible to execute workshop on your local machine here we are listing everything required to run it as we did.

We used [Standard_NV6](https://docs.microsoft.com/en-us/azure/virtual-machines/windows/sizes-gpu#nv-series) virtual machine with the following software installed:

* [7-zip](https://www.7-zip.org/)
* [Anaconda3 with Python 3.7](https://www.anaconda.com/)
* [Visual Studio Code](https://code.visualstudio.com/)
* [VcXsrv](https://sourceforge.net/projects/vcxsrv/)
* [NVIDIA Graphics Driver 425.31](https://sourceforge.net/projects/vcxsrv/)

Besides software you need [CARLA 0.9.3](https://github.com/carla-simulator/carla/releases/tag/0.9.3) (Windows and Linux), [CARLA ROS bridge 0.9.3](https://github.com/carla-simulator/ros-bridge/releases/tag/0.9.3), **this repository** and enabled Windows Subsystem for Linux with Ubuntu 18.04 sources.

Based on the [documentation](https://docs.microsoft.com/en-us/windows/wsl/install-on-server) here are the steps you need to install Ubuntu:
1. Open PowerShell as Administrator and run: `Enable-WindowsOptionalFeature -Online -FeatureName Microsoft-Windows-Subsystem-Linux`
2. Restart your computer when prompted. This reboot is required in order to ensure that WSL can initiate a trusted execution environment.
3. Download Ubuntu: `Invoke-WebRequest -Uri https://aka.ms/wsl-ubuntu-1804 -OutFile ~/Ubuntu.appx -UseBasicParsing`
4. Extract Ubuntu: `Rename-Item ~/Ubuntu.appx ~/Ubuntu.zip` and then `Expand-Archive ~/Ubuntu.zip ~/Ubuntu`

Here is the mapping of the above downloads to expected directory structure:

```
C:\Workshop
  - carla
    - PythonAPI <-- move files and directories from `downloads/CARLA_0.9.3.tar.gz/PythonAPI`
  - carla-client
    - PythonAPI <-- move files and directories from `downloads/CARLA_0.9.3.zip/PythonAPI`
    - manual_control.py <-- move `downloads/repository/install/carla-client/manual_control.py`
  - carla-ros-bridge
    - catkin_ws
      - src <-- move files and directories from `downloads/ros-bridge-0.9.3.zip`
	  - config <-- overwrite with `settings.yaml` from `downloads/repository/install/carla-ros-bridge/config`
  - carla-server <-- move files and directories from `downloads/CARLA_0.9.3.zip`
  - project <-- move files and directories from `downloads/repository/src`
  - ubuntu <-- move files and directories from extracted Ubuntu
``` 

> NOTE: You always can change the paths, but do not forget to modify script and keep sharp eye on guide.

When preparations of directory tree is done please execute: `repository/install/install.ps1` script. This script will install Ubuntu to WSL and initialize it with all required software.

> NOTE: Installation could take around 1-2 hours

## Executing Workshop

To run the workshop you will need a presentation and guide. Both of these can be found in [documentation](https://github.com/coherentsolutionsinc/issoft-insights-2019-sdc-carla-ros/tree/master/documentation) directory.

# Asknowlegements

We use [CARLA Simulator](https://github.com/carla-simulator/carla) as engine and [CARLA ROS bridge](https://github.com/carla-simulator/ros-bridge) to connect ROS to CARLA.

# Authors

This project is owned by Coherent Solutions.

# License

This project is licensed under the MIT License - see the [LICENSE.md](https://github.com/coherentsolutionsinc/issoft-insights-2019-sdc-carla-ros/blob/master/LICENSE.md) for details.
