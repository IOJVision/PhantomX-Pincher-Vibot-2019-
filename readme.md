PhantomX Pincher Robot Arm Kit Mark II - Turtlebot Arm
Introduction

The PhantomX Pincher AX-12 Robot Arm is a 5 degree-of-freedom robotic arm and an easy addition to the TurtleBot ROS robot platform.
Product features:

AX-12A Dynamixel Actuators
Solid Needle Roller Bearing Base
Rugged ABS construction
Arbotix Robocontroller for Onboard Processing
Custom Parallel Gripper
Mounting Brackets for Cameras & Sensors
28
th
September 2018

Product Specifications:



Hardware



5x AX-12A Dynamixel Actuators
PhantomX Pincher Arm Hardware & Frame Kit
ArbotiX Robocontroller
12v 5amp power supply
FTDI 5v Programming Cable
Dynamixel AX-12A



The AX-12A robot servo can track its speed, temperature, shaft position, voltage, and load. As if this weren't enough, the control algorithm used to maintain shaft position on the ax-12 actuator can be adjusted individually for each servo, allowing you to control the speed and strength of the motor's response. All the sensor management and position control are handled by the servos built-in micro controller. This distributed approach leaves your main controller free to perform other functions.



ArbotiX Robocontroller



The ArbotiX robocontroller is an advanced control solution for small-to-medium size robots. It incorporates a powerful AVR microcontroller, XBEE wireless radio, dual motor drivers, and 3-pin servo-style headers for IO. In order to run it, must use " Arduino IDE" and for this board to communicate with dynamixel AX-12 servos is by using " PyPose" and " BioloidController".

Robocontroller Specification:

 16MHz AVR microcontroller (ATMEGA644p).
 2 serial ports, 1 dedicated to Bioloid servo controller, the other to the XBEE radio
 32 I/O, 8 of which can function as analog inputs
 Servo style 3-pin headers (gnd, vcc, signal) on all 8 analog inputs, and 8 of the digital IO
 Dual 1A motor drivers, with combined motor/encoder header.
 XBEE radio sold separately. A typical setup will require 2 XBEE radios and an XBEE explorer to be able to wirelessly control your robot from your computer.
 This board requires either an FTDI cable or ISP. We recommend the Sparkfun FTDI breakout.
 8"x2.8" with mounting holes designed to match many Bioloid configurations.
PhantomX Pincher Arm Hardware & Frame Kit

NAME	PARTS
Lower deck	


|
| Upper deck |



|
| Ax hub bearing mounting |



|
| Ax Horn thin spacer |



|
| Arm spacer |



|
| Reverse bracket top |



|
| Reverse bracket bottom |



|
| Thrust bearing |



|
| Steel washer |



|
| PhantomX Parallel gripper kit |



|
| Metal F2 bracket |



|
| Metal F3 bracket |



|
| Metal F4 bracket |



|
| FTDI cable |



|
| 12V 5A Power supply |



|
| 3-pin Dynamixel Cable 200mm |



|
| 3-pin Dynamixel Cable 100mm |



|

Software needed

** ArbotiX-M Robocontroller**
◦◦** Arduino IDE** - The Arduino IDE (Integrated Developer Environment) is an application that you can use to program and interact with Arduino based microcontrollers
Download links: https://www.arduino.cc/en/Main/OldSoftwareReleases

◦◦** FTDI Drivers** - These drives will allow your FTDI-USB cable or UartSBee to function properly
Download links: http://www.ftdichip.com/Drivers/VCP.htm

◦◦** ArbotiX-M hardware and library files**
▪▪ Hardware - allow the Arduino IDE to build programs for the ATMega644p on the ArbotiX-M and contains definitions for the AVRSTK500 serial programmer

▪▪ Libraries - libraries that will help you use the ArbotiX-M to its full potential

▪▪ ArbotiX Sketches - Test code and sample code for various ArbotiX-M projects and InterbotiX robots.

Download links: https://github.com/trossenrobotics/arbotix/archive/master.zip

** Dynamixel servo**

◦◦** DynaManager** - a small application that will set the ID and Baud of various DYNAMIXEL servos. It requires Java and to make it work, it need ROS.ino sketch load into the ArbotiX.

Download links: [https://github.com/trossenrobotics/dynaManager/releases](https://github.com/trossenrobotics/dynaManager/releases)

Download link for ROS.ino : [http://trossenrobotics.com/Shared/Downloads/Arbotix\_ros.zip](http://trossenrobotics.com/Shared/Downloads/Arbotix_ros.zip)
ROS Implementation

Need to install ArbotiX ROS
To interface with the ArbotiX board

git clone https://github.com/IOJVision/arbotix\_ros

sudo apt-get install ros-indigo-arbotix

Get the turtlebot_arm package
In this package provide bring up, description, calibration, simulation and moveit package.
-But we need to change the environment variable from "TURTLEBOT_ARM1" to pincher because we are using PhantomX Pincher.

git clone https://github.com/IOJVision/turtlebot\_arm\_vision (Indigo only)

git clone https://github.com/turtlebot/turtlebot_arm.git (Kinetic and Indigo)

After all the installation have done, we test it with the arbotix_gui and the bringup to see either the arm is functional or not.
To make it easier, we can use moveit as a path planning for the arm and can implement to arm for a better controlling it.
As the test end, we need to calibrate it with the kinect in order to make sure the arm can perform a pick and place of an object.
IMPORTANT: IF PORT ACCESS IS RESTRICTED!!!

SOLUTION: sudo adduser &lt;the user you want to add&gt; dialout

sudo reboot

Step by step ROS Implementation (INDIGO)

Make sure you have a right version of ROS and the ubuntu
To check ROS version, printenv | grep ROS
To check ubuntu version, lsb_release –a
Install the ArbotiX ROS,
From the source,
1. Go to your workspace, cd ros/indigo/catkin_ws/src/
2. Add the github package,
git clone https://github.com/IOJVision/arbotix\_ros.git
3. Return to your catkin workspace, cd ..
4. Build the source, catkin_make
From the deb,
1. sudo apt-get install ros-indigo-arbotix
Get the turtlebot arm package,
From the source,
1. Go to your workspace, cd ros/indigo/catkin_ws/src/
2. Add the github package
git clone https://github.com/IOJVision/Turtlebot\_arm.git
3. Return to your catkin workspace, cd ..
4. Build the source, catkin_make
Before we start to play with the arm, make sure that arduino IDE is already installed to allow the computer to communicate with the ArbotiX Robocontroller.
Before you start, make sure the FTDI cable is already plugged in and all the servo cable is already attached correctly.
As the turtlebot arm package port is already set to "/dev/ttyUSB0", make sure you got the right port or you can change all the port setup at the .yaml file inside the turtlebot arm bringup package.
After all the check is done, start your terminal.
Start to bringup the arm first, roslaunch turtlebot_arm_bringup arm.launch
Then start the controller, rosrun arbotix_python arbotix_gui
After the bringup and the controller is successfully run, start to play with the arm. You can turn on or off the servo in the arbotix_gui controller.
Step by Step MoveIt Tutorial

Simulator
Make sure RViz is installed in your computer.
Go to the turtlebot arm moveit launch file then change the argument of "sim" to "true".
Then run the turtlebot arm moveit launch file,roslaunch turtlebot_arm_moveit_config turtlebot_arm_moveit.launch
Real arm
Plug in the arm usb to the computer
Go to the turtlebot arm moveit launch file then change the argument of "sim" to "false".
Run the turtlebot arm bringup roslaunch turtlebot_arm_bringup arm.launch
Then as usual run the turtlebot arm moveit launch file, roslaunch turtlebot_arm_moveit_config turtlebot_arm_moveit.launch


Step by Step Calibrating the PhantomX Pincher arm to the Kinect

Start up the arm and the camera for testing
roslaunch turtlebot_arm_bringup arm.launch for the arm
Roslaunch freenect_launch freenect.launch for the kinect xbox360/ 1
Check all to make sure it is functioning then terminate it.
To start calibration,
Make sure to have printed Calibration checkerboard (7x6 27mm)
The complete checkerboard have to be in the kinect field of view


Then, manually try to move the arm to make sure it can reach all the point in the checkerboard.
Additional info, the checkerboard must be on the same level of the arm base.
After all the test and set up has been done correctly, run the calibration program
roslaunch turtlebot_arm_kinect_calibration calibrate.launch in that launch file is already include the camera launch and arm bringup also the servo controller.
b.Firstly, it will pop out the image view of the calibration pattern on the checkerboard if it's able to detect it and the arm servo controller. 
3. The trick is the image by the calibration image view is just the picture of the pattern and it does not show the live picture of the movement of the arm, to view the live movement of the arm, can use rosrun image_view with the kinect camera topic or by using the RViz.
4. To move the arm to all the necessary point is very easy if we do by manually and less time taken for the calibration process. Start with uncheck all the servo, to let the motor power off so that we can move it manually but make sure the gripper is widely open and use the right section of the gripper to hit the 4 calibration point.
5.
e.Start with move it to the first point then press enter, continue until the 4th point. 
6. Until all the point reaches, in the terminal will process and it will print out the result like this:-

|

Resulting transform (camera frame - &gt; fixed frame):

-0.998072 0.038788 0.0484632 0.264075

0270779 0.974587 -0.222367 -0.0129825

-0.0558567 -0.220626 -0.973758 1.02946

     0          0          0          1
Resulting transform (fixed frame - &gt; camera frame):

-0.0558568 -0.220626 -0.973758 1.02946

0.998072 -0.0387881 -0.0484632 -0.309075

-0.0270779 -0.974587 0.222367 0.0129825

59135e-41 4.57566e-41 4.57566e-41 1

Static transform publisher (use for external Kinect):

rosrun tf static_transform_publisher x y z qx qy qz qw frame_id child_frame_id period_in_ms

rosrun tf static_transform_publisher 0.366333 0.227789 0.984579 0.436052 0.44573 -0.573806 0.530971 /base_link /camera_link 100 -- &gt; run that for the arm that does not on turtlebot

URDF output (use for Kinect on robot):

&lt;?xml version="1.0" ?&gt;

&lt;robot&gt;

 **\&lt;property**  name=&quot;turtlebot\_calib\_cam\_x&quot; value=&quot;0.366333&quot;  **/\&gt;**
 **\&lt;property**  name=&quot;turtlebot\_calib\_cam\_y&quot; value=&quot;0.227789&quot;  **/\&gt;**
 **\&lt;property**  name=&quot;turtlebot\_calib\_cam\_z&quot; value=&quot;0.984579&quot;  **/\&gt;**
 **\&lt;property**  name=&quot;turtlebot\_calib\_cam\_rr&quot; value=&quot;-0.214587&quot;  **/\&gt;**
 **\&lt;property**  name=&quot;turtlebot\_calib\_cam\_rp&quot; value=&quot;1.3412&quot;  **/\&gt;**
 **\&lt;property**  name=&quot;turtlebot\_calib\_cam\_ry&quot; value=&quot;-1.81876&quot;  **/\&gt;**
 **\&lt;property**  name=&quot;turtlebot\_kinect\_frame\_name&quot; value=&quot;base\_link&quot;  **/\&gt;**
&lt;/robot&gt;
|
| --- |

Run the static transform publisher output, rosrun tf static_transform_publisher 0.366333 0.227789 0.984579 0.436052 0.44573 -0.573806 0.530971 /base_link /camera_link 100 and As long as this command runs, the static_transform_publisher will publish the transform between the arm frame and the kinect frame. If you move the physical camera, you will need to recalibrate again.

Pick and Place Demo via Simulator

Start the MoveIt from turtlebot arm package in the simulation mode(by default, the simulation is off) , roslaunch turtlebot_arm_moveit_config turtlebot_arm_moveit.launch sim:=true –screen
After the commad has been launch, you will see the PhantomX Pincher robot arm model in the RViz. If it show other robot arm model, please change the environment variable from turtlebot_arm1 to pincher.


After that start the pick and place demo by running the rosrun turtlebot_arm_moveit_demos pick_and_place.py in the new terminal.


Pick and Place Demo via Real Arm

For the real arm, we need to use calibration point as we did in above then use the same set up as that cause we do not want to calibrate again if we move the camera or change the set up.
Firstly, we need to have a square box with dimension of 2 cm or 2.5 cm because our gripper is only capable to grab until 3 cm. We can change the dimension of the box inside the launch file and also if our table is not aligned to the base of the arm, we can set that also inside the launch file.


For now, the code is only capable to detect green cube only.
To start the block detection, launch roslaunch turtlebot_arm_block_manipulation block_manip_complete.launch. If our calibration is already good then we do not want to change the position, we can add permanently in our launch file.
Inside the launch file that we launch in step 4, it is included with the arm bringup, camera bringup and also rviz moveit bringup.










Face Detector with The Arm Pose

Before starting, make sure you have the rbx1 vision package from pirobot or git clone https://github.com/IOJVision/rbx1.git at your src folder and catkin make your workspace.
To download the arm script to subscribing the face detector ROI, git clone https://github.com/IOJVision/turtlebot_arm_vision.git
Make sure you have the freenect launcher of the kinect bringup.
First, run this command of bringup the arm, moveit rviz and kinect roslaunch turtlebot_arm_bringup arm_moveit.launch
Then in running the face detector, roslaunch rbx1_vision face_tracker2.launch
Lastly, run the script rosrun turtlebot_arm_moveit_demos head_tracker.py
When there is face detected, the arm will pose a waving pose and when there is no face, the arm will be in resting.
References

Overview of Phantomx Pincher robot arm
◦◦** https://learn.trossenrobotics.com/38-interbotix-robots/122-phantomx-pincher-robot-arm.html**
The list of hardware and guideline for Phantomx Pincher robot arm
◦◦ https://learn.trossenrobotics.com/16-interbotix/robot-arms/pincher-robot-arm/163-phantomx-pincher-robot-arm-assembly-guide.html
How to setup ArbotiX-M Robocontroller
◦◦ https://learn.trossenrobotics.com/arbotix/7-arbotix-quick-start-guide
Overview of ArbotiX-M Robocontroller
◦◦ http://www.vanadiumlabs.com/arbotix.html
How to Set DYNAMIXEL IDs with the DynaManager
◦◦ https://learn.trossenrobotics.com/arbotix/1-using-the-tr-dynamixel-servo-tool#&panel1-1
Overview of Dynamixel ax-12
◦◦ https://www.trossenrobotics.com/dynamixel-ax-12-robot-actuator.aspx
To check the build of the PhantomX Pincher robot arms
◦◦ https://learn.trossenrobotics.com/interbotix/robot-arms/16-phantomx-pincher-robot-arm/25-phantomx-pincher-robot-arm-build-check
ArbotiX ros download
◦◦ https://github.com/vanadiumlabs/arbotix_ros.git
Nathan Crombez guideline on phantomx pincher on ros indigo
◦◦ https://github.com/NathanCrombez/PhantomXPincherArmROS
Guideline on phantomx pincher on ros kinetic
◦◦ http://wiki.ros.org/turtlebot_arm/Tutorials/InstallationInstructions
◦◦** https://github.com/turtlebot/turtlebot\_arm**