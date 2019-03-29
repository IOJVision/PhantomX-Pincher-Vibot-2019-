# PhantomX Pincher Robotic Arm under ROS Indigo 
<p align="center">
<img src="https://github.com/IOJVision/PhantomX-Pincher-Vibot-2019-/blob/master/Images/Hardware/phantomxpincher.jpg" align ="middle" width="60%" height="60%" title="header">
</p>

## Check the Ros Distro Version and Ubuntu Version
__Make sure it was Indigo and Ubuntu 14.04__ : 

     printenv | grep ROS 
     lsb_release –a

## Packages installation

__Do not forget to update to latest software update for ROS and Ubuntu__

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

## 1. Test the packages installed
  * Make sure the FTDI cable is already plugged in and all the servo cable is already attached correctly
  * As the turtlebot arm package port is already set to “/dev/ttyUSB0”, make sure you got the right port or you can change  all the port setup at the .yaml file inside the turtlebot arm bringup package. 
  * After all the check is done, start your terminal. 
  * Start to bringup the arm first, roslaunch turtlebot_arm_bringup arm.launch
  * Then start the controller, rosrun arbotix_python arbotix_gui
  * After the bringup and the controller is successfully run, start to play with the arm. You can turn on or off the servo in the arbotix_gui controller. 

<p align="center">
<img src="https://github.com/IOJVision/PhantomX-Pincher-Vibot-2019-/blob/master/Images/Hardware/Screenshot%20from%202019-03-28%2011-26-51.png" align ="middle" width="60%" height="60%" title="header">
</p>

## IMPORTANT: IF PORT ACCESS IS RESTRICTED!!! 
__SOLUTION__:

      sudo adduser <the user you want to add> dialout 
      sudo reboot

## 2. Step by step MoveIt! tutorial

<p align="center">
<img src="https://github.com/IOJVision/PhantomX-Pincher-Vibot-2019-/blob/master/Images/MoveIt!/MoveIt!Interface.png" align ="middle" width="60%" height="60%" title="MoveItInterface">
</p>

<p align="center">
<img src="https://github.com/IOJVision/PhantomX-Pincher-Vibot-2019-/blob/master/Images/MoveIt!/MoveIt!Planner.png" align ="middle" width="60%" height="60%" title="MoveItPlanner">
</p>

<p align="center">
<img src="https://github.com/IOJVision/PhantomX-Pincher-Vibot-2019-/blob/master/Images/MoveIt!/pathplanning.png" align ="middle" width="60%" height="60%" title="PathPlanning">
</p>

__Simulation__:
* Make sure RViz is installed in your computer. 
* Go to the turtlebot arm moveit launch file then change the argument of “sim” to “true”.  
     
          roscd turtlebot_arm_moveit_config/launch
          gedit turtlebot_arm_moveit.launch
      
* Then run the turtlebot arm moveit launch file
          
          roslaunch turtlebot_arm_moveit_config turtlebot_arm_moveit.launch
  
* Or by change by the launch command, run the turtlebot arm moveit launch file

          roslaunch turtlebot_arm_moveit_config turtlebot_arm_moveit.launch sim:=true --screen 

__Real arm__:
* Plug in the arm usb to the computer 
* Go to the turtlebot arm moveit launch file then change the argument of “sim” to “false”. follow as above step..
* Run the turtlebot arm bringup
     
          roslaunch turtlebot_arm_bringup arm.launch  without the AbotiX ROS
          roslaunch turtlebot_arm_bringup armcontrol.launch  with the ArbotiX ROS
          
* Then as usual run the turtlebot arm moveit launch file
     
          roslaunch turtlebot_arm_moveit_config turtlebot_arm_moveit.launch
          
## 3. Step by step Calibrating the PhantomX Pincher with the KINECT
* Start up the arm and the camera for testing

          roslaunch turtlebot_arm_bringup arm.launch for the arm
          roslaunch freenect_launch freenect.launch for the kinect
          
* Check all to make sure it is functioning then terminate it. 
* To start calibration,

     __Make sure to have printed Calibration checkerboard (7x6 27mm) __: 
     click here to download the checkerboard: [checkerboard](https://github.com/IOJVision/PhantomX-Pincher-Vibot-2019-/blob/master/Images/Calibration/check_7x6_27mm.pdf)
     
     <p align="center">
<img src="https://github.com/IOJVision/PhantomX-Pincher-Vibot-2019-/blob/master/Images/Calibration/Screenshot%20from%202019-03-25%2014:33:05.png" align ="middle" width="60%" height="60%" title="checkerboard">
</p>

     The complete checkerboard must be in the kinect field of view :
     
     <p align="center">
<img src="https://github.com/IOJVision/PhantomX-Pincher-Vibot-2019-/blob/master/Images/Calibration/FieldView.jpg" align ="middle" width="60%" height="60%" title="checkerboard">
</p>
     
     Then, manually try to move the arm to make sure it can reach all the point in the checkerboard.:
     
     Additional info, the checkerboard must be on the same level of the arm base. :
     
     <p align="center">
<img src="https://github.com/IOJVision/PhantomX-Pincher-Vibot-2019-/blob/master/Images/Calibration/LevelofTheboard.jpg" align ="middle" width="60%" height="60%" title="checkerboard">
</p>

<p align="center">
<img src="https://github.com/IOJVision/PhantomX-Pincher-Vibot-2019-/blob/master/Images/Calibration/TbaleHeight.jpg" align ="middle" width="60%" height="60%" title="checkerboard">
</p>
     
* After all the test and set up has been done correctly, run the calibration program
          
          roslaunch turtlebot_arm_kinect_calibration calibrate.launch 
          In that launch file is already include the camera launch and arm bringup also the servo controller.
          
* Firstly, it will pop out the image view of the calibration pattern on the checkerboard if it’s able to detect it and the arm servo controller. 

  <p align="center">
<img src="https://github.com/IOJVision/PhantomX-Pincher-Vibot-2019-/blob/master/Images/Calibration/Screenshot%20from%202019-03-19%2011:21:36.png" align ="middle" width="60%" height="60%" title="checkerboard">
</p>

* The trick is the image by the calibration image view is just the picture of the pattern and it does not show the live picture of the movement of the arm, to view the live movement of the arm, can use rosrun image_view with the kinect camera topic or by using the RViz. 

* To move the arm to all the necessary point is very easy if we do by manually and less time taken for the calibration process. Start with uncheck all the servo, to let the motor power off so that we can move it manually but make sure the gripper is widely open and use the right section of the gripper to hit the 4-calibration point. 

* Start with move it to the first point then press enter, continue until the 4th point. 


__First Point__:

<p align="center">
<img src="https://github.com/IOJVision/PhantomX-Pincher-Vibot-2019-/blob/master/Images/Calibration/1st%20point.JPG" align ="middle" width="60%" height="60%" title="checkerboard">
</p>

__Second Point__:


<p align="center">
<img src="https://github.com/IOJVision/PhantomX-Pincher-Vibot-2019-/blob/master/Images/Calibration/2ndPoint.JPG" align ="middle" width="60%" height="60%" title="checkerboard">
</p>

__Third Point__:

<p align="center">
<img src="https://github.com/IOJVision/PhantomX-Pincher-Vibot-2019-/blob/master/Images/Calibration/3rdPoint.JPG" align ="middle" width="60%" height="60%" title="checkerboard">
</p>

__Fourth Point__:

<p align="center">
<img src="https://github.com/IOJVision/PhantomX-Pincher-Vibot-2019-/blob/master/Images/Calibration/4thPoint.JPG" align ="middle" width="60%" height="60%" title="checkerboard">
</p>

* Until all the point reaches, in the terminal will process and it will print out the result like this:- 

          Resulting transform (camera frame -> fixed frame): 
           0.999483 -0.028142 0.0155747 -0.184333
          -0.0182255 -0.894504 -0.446689  0.314301
           0.0265024  0.446174 -0.894554  0.951596
               0         0         0         1

          Resulting transform (fixed frame -> camera frame): 
           0.0265023   0.446174  -0.894554   0.951596
           -0.999483  0.0281421 -0.0155746   0.139333
           0.0182256   0.894504   0.446689  -0.314301
          4.59163e-41          0          0          1

          Static transform publisher (use for external kinect): 
          rosrun tf static_transform_publisher x y z qx qy qz qw frame_id child_frame_id period_in_ms
          rosrun tf static_transform_publisher 0.119769 -0.147356 0.993819 -0.371373 0.372475 0.589925 0.612645 /base_link /camera_link 100   

          URDF output (use for kinect on robot): 
          <?xml version="1.0"?>
          <robot>
               <property name="turtlebot_calib_cam_x" value="0.119769" />
               <property name="turtlebot_calib_cam_y" value="-0.147356" />
               <property name="turtlebot_calib_cam_z" value="0.993819" />
               <property name="turtlebot_calib_cam_rr" value="-0.0348527" />
               <property name="turtlebot_calib_cam_rp" value="1.10743" />
               <property name="turtlebot_calib_cam_ry" value="1.51147" />
               <property name="turtlebot_kinect_frame_name" value="base_link" />
          </robot>

* Run the static transform publisher output and As long as this command runs, the static_transform_publisher will publish the transform between the arm frame and the kinect frame. If you move the physical camera, you will need to recalibrate again
     
          rosrun tf static_transform_publisher 0.366333 0.227789 0.984579 0.436052 0.44573 -0.573806 0.530971 /base_link /camera_link 100



## 4. Step by step Pick and Place Demo via Simulator 

* Start the MoveIt from turtlebot arm package in the simulation mode(by default, the simulation is false)
          
          roslaunch turtlebot_arm_moveit_config turtlebot_arm_moveit.launch sim:=true –-screen
          
* After the commad has been launch, you will see the PhantomX Pincher robot arm model in the RViz. If it show other robot arm model, please change the environment variable from turtlebot_arm1 to pincher.

<p align="center">
<img src="https://github.com/IOJVision/PhantomX-Pincher-Vibot-2019-/blob/master/Images/MoveIt!/Screenshot%20from%202019-03-10%2021:21:35.png" align ="middle" width="60%" height="60%" title="rvizstartup">
</p>

* After that start the pick and place demo by running the below line in the new terminal. 

          rosrun turtlebot_arm_moveit_demos pick_and_place.py
          
<p align="center">
<img src="https://github.com/IOJVision/PhantomX-Pincher-Vibot-2019-/blob/master/Images/MoveIt!/Screenshot%20from%202019-03-28%2012-51-06.png" align ="middle" width="60%" height="60%" title="rvizstartup">
</p>


## 5. Step by step Pick and Place Demo via Real Arm 

* For the real arm, we need to use calibration point as we did in above then use the same set up as that cause we do not want to calibrate again if we move the camera or change the set up.

* Firstly, we need to have a cube type of object with dimension of 2 cm or 2.5 cm because our gripper is only capable to grab until 3 cm. We can change the dimension of the box inside the launch file and also if our table is not aligned to the base of the arm, we can set that also inside the launch file. 

          <launch>

       <!--  ************* Arm bringup stuff ************* -->

       <include file="$(find turtlebot_arm_bringup)/launch/arm.launch" />

       <node name="arbotix_gui" pkg="arbotix_python" type="arbotix_gui" output="screen"/>


       <!--  ************* Moveit config stuff *************  -->

       <!-- Load the URDF, SRDF and other .yaml configuration files on the param server -->
       <include file="$(find turtlebot_arm_moveit_config)/launch/planning_context.launch">
         <arg name="load_robot_description" value="true"/>
       </include>

       <!-- Run the main MoveIt executable to provide move groups -->
       <include file="$(find turtlebot_arm_moveit_config)/launch/move_group.launch">
         <arg name="allow_trajectory_execution" value="true"/>  
         <arg name="fake_execution" value="false"/>
         <arg name="info" value="true"/>
       </include>


       <!--  ************* Kinect bringup stuff ************* -->
       <include file="$(find freenect_launch)/launch/freenect.launch" />s



       <!--  ************* Block manipulation stuff ************* -->
       <include file="$(find turtlebot_arm_block_manipulation)/launch/block_manipulation.launch" />

       <node name="block_manipulation_demo" pkg="turtlebot_arm_block_manipulation" type="block_manipulation_demo" output="screen" >
         <param name="arm_link" value="/arm_base_link" />
         <param name="gripper_open" value="0.1" />
         <param name="gripper_closed" value="0.001" /> 
         <param name="z_up" value="0.1" />
         <param name="table_height" value="-0.03" />
          <param name="block_size" value="0.020" /> in here we have to change the size of the object
         <!--param name="target_x" value="" /-->
         <!--param name="target_y" value="" /-->
       </node>

       <node name="rviz" pkg="rviz" type="rviz" args="-d $(find turtlebot_arm_block_manipulation)/demo/block_manipulation_demo.rviz" />

       <node pkg="tf" type="static_transform_publisher" name="static_transform_publisher" args="0.552972 0.0311763 1.01794 -0.485351 0.0823443 0.864462 0.101775 /base_link /camera_link 100"/> THIS IS THE CALIBRATION POINT WE INCLUDED IN THE LAUNCH FILE

          </launch>
          
* For now, the code is only capable to detect green cube only. 
* To start the block detection, launch
     
           roslaunch turtlebot_arm_block_manipulation block_manip_complete.launch
           
   If our calibration is already good then we do not want to change the position, we can add permanently in our launch file.

* Inside the launch file that we launch in step 4, it is included with the arm bringup, camera bringup and also rviz moveit bringup. 
* After all of the necessary application launch, run this command to start object detetction and demonstration of pick and place

          rostopic pub -r 1000 /relay/robot_status2 std_msgs/String "vsdone"
          
* Demonstration video for:
     [2cm cube](https://github.com/IOJVision/PhantomX-Pincher-Vibot-2019-/blob/master/Video/Pick%20and%20Place/2cmCube.m4v)
     [2.5cm cube](https://github.com/IOJVision/PhantomX-Pincher-Vibot-2019-/blob/master/Video/Pick%20and%20Place/3cmCUbe.m4v)
     
     

          
## 6. Face Detector with The Arm Pose 
* Before starting, make sure you have the rbx1 vision package from pirobot
     
         cd ~/ros/indigo/catkin_ws/src
         git clone https://github.com/IOJVision/rbx1.git
         cd .. && catkin_make
 
* First, run this command of bringup the arm, moveit rviz and kinect

          roslaunch turtlebot_arm_bringup arm_moveit.launch
         
* Then run the face detector

          roslaunch rbx1_vision face_tracker2.launch 
          
* Lastly, run the script
     
          rosrun turtlebot_arm_moveit_demos head_tracker.py 
          
* When there is face detected, the arm will pose a waving pose and when there is no face, the arm will be in resting. 
