# PhantomX Pincher Robotic Arm under ROS Indigo 
<p align="center">
<img src="https://github.com/IOJVision/PhantomX-Pincher-Vibot-2019-/blob/master/Images/Hardware/phantomxpincher.jpg" align ="middle" width="60%" height="60%" title="header">
</p>

## Check the Ros Distro Version and Ubuntu Version
__Make sure it was Indigo and Ubuntu 14.04__ : 

     printenv | grep ROS 
     lsb_release –a

## Packages installation

__Do not forget to update to latest software update for ROS and Ubuntu

     sudo apt-get update
     sudo apt-get dist-upgrade

__Turtlebot arm__: 
The turtlebot arm packages provides bringup, description and utilities for using the PhantomX Pincher arm.
Installation from source:

     cd ~/ros/indigo/catkin_ws/src
     git clone https://github.com/IOJVision/Turtlebot_arm.git
     cd .. && catkin_make

__ArbotiX Ros__:
ArbotiX ROS drivers let the ROS interface with the ArbotiX board on the PhantomX Pincher
Installation from source:
     
     cd ~/ros/indigo/catkin_ws/src
     git clone https://github.com/IOJVision/arbotix_ros.git
     cd .. && catkin_make
     
__MoveIt!__:
MoveIt! let you plan the arm movement and visualize the arm in RViz
Installation from deb:

     sudo apt-get install ros-indigo-moveit
     
## ArbotiX arm architecture
__To edit or view the arm architecture__:
     
     roscd turtlebot_arm/turtlebot_arm_bringup/config
     gedit pincher_arm.yaml
     
The arm architecture is described in yaml file. USB port, number of joints, limits...

     source: pincher_arm.yaml
     port: /dev/ttyUSB0
     read_rate: 15
     write_rate: 25
     joints: {
                   arm_shoulder_pan_joint: {id: 1,  neutral: 512, max_angle: 140, min_angle: -140, max_speed: 90, type: dynamixel},
                   arm_shoulder_lift_joint: {id: 2, max_angle: 126, min_angle: -119, max_speed: 90, type: dynamixel},
                   arm_elbow_flex_joint: {id: 3, max_angle: 136, min_angle: -139, max_speed: 90, type: dynamixel},
                   arm_wrist_flex_joint: {id: 4, max_angle: 96, min_angle: -98, max_speed: 90, type: dynamixel},
                   gripper_joint: {id: 5, max_angle: 0, min_angle: -145, max_speed: 90, type: prismatic, radius: .0078, connector: .024, offset: .016}
               }
     controllers: {
                    arm_controller: {type: follow_controller, joints: [arm_shoulder_pan_joint, arm_shoulder_lift_joint, arm_elbow_flex_joint, arm_wrist_flex_joint], action_name: arm_controller/follow_joint_trajectory, onboard: False }
                    }    

## Test the packages installed
  * Make sure the FTDI cable is already plugged in and all the servo cable is already attached correctly
  * As the turtlebot arm package port is already set to “/dev/ttyUSB0”, make sure you got the right port or you can change  all the port setup at the .yaml file inside the turtlebot arm bringup package. 
  * After all the check is done, start your terminal. 
  * Start to bringup the arm first, roslaunch turtlebot_arm_bringup arm.launch
  * Then start the controller, rosrun arbotix_python arbotix_gui
  * After the bringup and the controller is successfully run, start to play with the arm. You can turn on or off the servo in the arbotix_gui controller. 

<p align="center">
<img src="https://github.com/IOJVision/PhantomX-Pincher-Vibot-2019-/blob/master/Images/Hardware/Screenshot%20from%202019-03-28%2011-26-51.png" align ="middle" width="60%" height="60%" title="header">
</p>
