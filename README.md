# robotic_arm

### Hardware
* 1x 3-DOF leg of Stanford Pupper [Quadruped](https://pupper-independent-study.readthedocs.io/en/latest/reference/design.html#)
    1. 3D printed hip
    2. 3D printed upper leg
    3. 3D printed lower leg
* 3x RoboMaster M2006 P36 Brushless DC Gear [Motor](https://www.robomaster.com/en-US/products/components/detail/1277) 
* 3x RoboMaster C610 Brushless Motor Electronic Speed [Controller](https://www.robomaster.com/en-US/products/components/detail/1277)
* 1x Robotmaster Terminal [Block](https://store.dji.com/product/rm-m3508-accessories-kit) to connect C610 power and CAN cables 
* 1x Waveshare RS485 CAN [HAT](https://www.waveshare.com/rs485-can-hat.htm) 
* 1x Raspberry Pi 3b

### Robot Operating System
* Mac Mini M1: [Docker image](https://hub.docker.com/r/tiryoh/ros-desktop-vnc) running ROS1 Noetic 
* Raspberry Pi 3b: Ubuntu Mate 20.04 installation running ROS1 Noetic

![trajectory_controller_gazebo](/pictures/trajectory_controller_video.gif)

