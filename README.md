# VR Teleoperation Controller for Universal Robots
A Meta Quest 3 light-weight teleoperation controller for Universal Robots manipulators. Robot communication is based on [ur_rtde](https://sdurobotics.gitlab.io/ur_rtde/ "ur_rtde") and is therefore robot-version agnostic and control can be applied for UR3, UR5, UR10, etc.

This software was developed to collect datasets for finetuning of generalist robot policies and Vision-Language-Action models. The default controller features multiprocessing of multiple RealSense camera datastreams and direct control of a Robotiq Hand-E end-effector, but can be readily modified for other hardware setups. 

## Installation
#### 1. Prerequisites
Install [Meta Horizon Link](https://www.meta.com/help/quest/1517439565442928/ "Meta Horizon Link") app
Install [SteamVR](https://store.steampowered.com/app/250820/SteamVR/ "SteamVR") app

#### 2. Python Setup
```
git clone https://github.com/plund-dtu/UniversalRobotsTeleop.git
cd UniversalRobotsTeleop
pip install -r requirements.txt
```
Note: this repository is built on Python 3.10.11

## Usage 
#### 1. Hardware overview and configuration
Rotations matrices are used to transform the HMD axis to:
[![hmd_axis](https://github.com/plund-dtu/UR_VR_Teleop/blob/main/documentation/hmd_axis.png "hmd_axis")](https://github.com/plund-dtu/UR_VR_Teleop/blob/main/documentation/hmd_axis.png "hmd_axis")
To modify axis, rotation matrices in`ur_teleop_utils.py`must be manually modified..

Hand controllers are configured as:
[![controllers](https://github.com/plund-dtu/UR_VR_Teleop/blob/main/documentation/controllers.png "controllers")](https://github.com/plund-dtu/UR_VR_Teleop/blob/main/documentation/controllers.png "controllers")
**Control the robot by moving the right hand-controller**. To modify button and trigger actions,`ur_teleop_utils.py`must be manually modified.
Default configuration for pause/unpause allows movement of the hand-controller while paused and continuing from the current position when unpaused.
**WARNING: Avoid pausing/unpausing if the RTDE receive interface breaks with  `RTDEReceiveInterface boost system Exception: (asio.misc:2) End of file`, this will prevent realignment during unpausing and may cause sudden robot movements!**

#### 2. Setup and calibration
* Start the Meta Horizon Link app and establish USB-C or wireless connection with Meta Quest 3
* Start SteamVR and calibrate controller axis by directing the Y-axis of the HMD along the robot base frame Y-axis and recenter the HMD through SteamVR:
[![steamvr_recenter](https://github.com/plund-dtu/UR_VR_Teleop/blob/main/documentation/steamvr_recenter.png "steamvr_recenter")](https://github.com/plund-dtu/UR_VR_Teleop/blob/main/documentation/steamvr_recenter.png "steamvr_recenter")

#### 2. Simple teleoperation
To run the teleoperation controller without any data collection pipeline, update the IP in `ur_teleop.py` and run. 
The controller will start in paused mode, press X on the left controller to unpause. "Save data" and "reset" buttons are inactive for simple teleoperation. 

#### 3. Teleoperation with default data collection pipeline
Ensure RealSense cameras are connected. The script will identify serial ID for each camera and initiate an individual data stream process for each. 
To run the teleoperation controller with default data collection pipeline, `ur_teleop_data_collection.py` must be updated with:
* Robot IP
* episode_save_path
* episode_task

Default logging frequency is 15 Hz. 

**Note**: wait for data stream processes to report initialization before unpausing the robot. 

#### 4. Teleoperation with custom data collection pipeline
The teleoperation controller is based on sequential readings and steps, which can be extracted from the utils to run in an open loop setup as seen in `ur_teleop_custom_data_collection`.
Custom image and data collection pipelines can be defined in this loop to adapt to various hardware or to write to specific dataset formats such as LeRobot. 
