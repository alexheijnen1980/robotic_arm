# Hardware
* 1x 3-DOF leg of Stanford Pupper [Quadruped](https://pupper-independent-study.readthedocs.io/en/latest/reference/design.html#)
    1. 3D printed hip
    2. 3D printed upper leg
    3. 3D printed lower leg
* 3x RoboMaster M2006 P36 Brushless DC Gear [Motor](https://www.robomaster.com/en-US/products/components/detail/1277) 
* 3x RoboMaster C610 Brushless Motor Electronic Speed [Controller](https://www.robomaster.com/en-US/products/components/detail/1277)
* 1x Robotmaster Terminal [Block](https://store.dji.com/product/rm-m3508-accessories-kit) to connect C610 power and CAN cables 
* 1x Waveshare RS485 CAN [HAT](https://www.waveshare.com/rs485-can-hat.htm) 
* 1x Raspberry Pi 3b

![3-dof_robotic_arm](/pictures/3-DOF_robotic_arm.jpg)

# Robot Operating System
* Mac Mini M1: [Docker image](https://hub.docker.com/r/tiryoh/ros-desktop-vnc) running ROS1 Noetic 
* Raspberry Pi 3b: Ubuntu Mate 20.04 installation running ROS1 Noetic

![trajectory_controller_gazebo](/pictures/trajectory_controller_video.gif)

# Denavit-Hartenberg Method to Derive Forward Kinematics

## General outline
[This](https://www.youtube.com/playlist?list=PLT_0lwItn0sAfi3o4xwx-fNfcnbfMrXa7) set of videos does a great job of explaining how to use the Denavit-Hartenberg method. When using this method it is important to adhere to the four rules below. Assign frames according to DH-rules:
1. the z-axis is the axis of rotation for a revolute joint
2. the x_n-axis is perpendicular to both z_n and z_n-1
3. the y-axis is determined using the right hand rule
4. the x_n-axis must intersect the z_n-1-axis
The fourth rule triggered the non-obvious position of the z-axis (and corresponding frame) around which the second joint rotates. In order to have the x-axis of that frame intersect with the z-axis of the previous frame (around which the first joint rotates), it had to be placed in front of the shaft that connects the link of the hip and the link of the upper leg. Note to self to determine if similar rules are required when applying the Product of Exponentials method. 

The same set of videos also covers how to fill out the DH-table:
1. $\theta$: the rotation of frame_n-1 around z_n-1 to align x_n-1 with x_n
2. $\alpha$: the rotation of frame_n-1 around x_n to align z_n-1 with z_n
3. r: distance along the x_n direction between frame centers
4. d: distance along the z_n-1 direction between frame centers 
It is important to pay attention to the subtle differences in how $\theta$ and $\alpha$ and r and d are determined. Additionally, for $\theta$ it is important to also include the rotation of the frame_n-1 itself. 

Finally, after completing the DH-table the homogenous transformation matrix between each frame pair is obtained by filling out the correct parameters in the table below (here, a is used instead of r). The homogenous transformation matrix that is (likely) of most interest expresses the pose of the end-effector in the base frame (world frame). This is obtained through matrix multiplication of all derived homogenous transformation matrices (e.g. H_04 = H_01 * H_12 * H_23 * H34).

![transformation_matrix](/pictures/transformation_matrix_from_DH-parameters.png)

## Denavit-Hartenberg method applied to robotic arm
### Frame 0 (fixture to hip)
![fixture_hip](/pictures/fixture_hip.png)
* $\theta_0$ = 0.0 | x_1 and x_0 are already aligned ($\theta_0$ is 0 since this is a fixed joint)
* $\alpha_0$ = 0.0 | z_1 and z_0 are already aligned
* r_0 = 0.0 | frame centers are offset by 0 m in the x_1 direction
* d_0 = 0.10 | frame centers are offset by 0.10 m in the z_0 direction

### Frame 1 (hip to upper leg)
![hip_upper_leg](/pictures/hip_upper_leg.png)

This frame is 'oddly' positioned to ensure that the x_2-axis intersects the z_1-axis.
* $\theta_1$ = $\theta_1$ | x_2 and x_1 are already aligned, frame_1 rotates along $\theta_1$
* $\alpha_1$ = 90 | align z_1 with z_2 by rotating frame_1 around x_2 by 90 degrees
* r_1 = 0.0 | frame centers are offset by 0 m in the x_2 direction
* d_1 = 0.028 | frame centers are offset by 0.028 m in the z_1 direction

### Frame 2 (upper leg to lower leg)
![upper_leg_lower_leg](/pictures/upper_leg_lower_leg.png)
* $\theta_2$ = $\theta_2$ | x_3 and x_2 are already aligned, frame_2 rotates along $\theta_2$
* $\alpha_2$ = 180 | align z_2 with z_3 by rotating frame_2 around x_3 by 180 degrees
* r_2 = -0.08004 | frame centers are offset by -0.08004 m in the x_3 direction
* d_2 = 0.040 frame centers are offset by 0.040 m in the z_2 direction

### Frame 3 (lower leg to end effector)
![lower_leg_end_effector](/pictures/lower_leg_end_effector.png)
* $\theta_3$ = $\theta_3$ | x_4 and x_3 are already aligned, frame_3 rotates along $\theta_3$
* $\alpha_3$ = 0.0 | z_4 and z_3 are already aligned
* r_3 = -0.11003 | frame centers are offset by -0.11003 m in the x_4 direction
* d_3 = 0.006 | frame centers are offset by 0.006 m in the z_3 direction

### DH-table
The information above can be summarized into the DH table below:
```
   #   | theta   | alpha  | r       | d
   0   | 0       | 0      | 0       | 0.100
   1   | theta_1 | 90     | 0       | 0.028
   2   | theta_2 | 180    |-0.08004 | 0.040
   3   | theta_3 | 0      |-0.11003 | 0.006  
```
### Homogenous transformation matrix (forward kinematics)
Calculating the homogenous transformation matrix that expresses the pose of the end-effector in the base frame (world frame) by hand, would be very error prone since it involves matrix multiplication of four 4x4 matrices. Instead, the [Robotics Toolbox](https://petercorke.github.io/robotics-toolbox-python/intro.html#id1) is used to first define a kinematic model using the DH-table above, then set joint positions ($\theta_0$ has to remain zero as this is a fixed joint), then using the forward kinematics method. E.g.:
``` python
# Import libraries
import numpy as np
import roboticstoolbox as rtb
from spatialmath import SE3

## Create robotic arm using Denavit-Hartenberg parameters
link_1 = rtb.RevoluteDH(d = 0.100, a = 0, alpha = 0)
link_2 = rtb.RevoluteDH(d = 0.028, a = 0, alpha = 0.5*np.pi )
link_3 = rtb.RevoluteDH(d = 0.040, a = -0.08004, alpha = 1.0*np.pi)
link_4 = rtb.RevoluteDH(d = 0.006, a = -0.11003, alpha = 0)
robot = rtb.DHRobot([link_1, link_2, link_3, link_4])

# Set joint positions and determine homogenous transformation matrix to find end effector position
q1 = np.radians(0)
q2 = -1.05
q3 = -1.35
q4 = -1.47
T = robot.fkine([q1, q2, q3, q4])
print("Transformation Matrix :\n", T)
```
## ROS TF library
* Launch joint_state_publisher node, robot_state_publisher node and RVIZ.
``` xml
<?xml version="1.0"?>
<launch>
  <arg name="model" default="$(find bazu)/src/urdf/leg_CAD.urdf"/>
  <param name="robot_description" command="$(find xacro)/xacro $(arg model)"/>
  <node name="joint_state_publisher" pkg="joint_state_publisher_gui" type="joint_state_publisher_gui"/>
  <node name="robot_state_publisher" pkg="robot_state_publisher" type="robot_state_publisher"/>
  <node name="rviz" pkg="rviz" type="rviz"/>
</launch>
```
* Add TF and RobotModel.
* Start tf_echo node to ''echo the transform from the coordinate frame of the source_frame to the coordinate frame of the target_frame''. I.e. to display the pose of the end_effector frame expressed in the world frame: ``rosrun tf tf_echo world end_effector``.

``` bash
# display tf_tree
rosrun qrt_tf_tree rqt_tf_tree
```

# Other Useful Commands
``` bash
# ssh into Raspberry Pi
ssh pi@raspberrypi

# source directory
source ~/robotic_arms_ws/devel/setup.bash

# copy files from Raspberry Pi to current host directory
scp pi@raspberrypi:~/Robot/<file> .

# copy files from docker docker container to current host directory
sudo docker cp <container_name>:/home/ubuntu/robotic_arms_ws/<file> .
```

