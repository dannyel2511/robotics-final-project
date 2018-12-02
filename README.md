# Robotics Final Project

This repository contains the files used to program the robot and control its operation using Arduino and ROS Kinetic. This projects aims to develop the autonomous navigation of a mobile robot using ROS, to control the logic of the robot; and Arduino, as an interface between the onboard computer and the motors.

![Imagen del robot](https://i.imgur.com/9DvviGGl.jpg)


In the following sections, there are some explanations to start the robot and set the environment in the PC and onboard computer to run the project-shark package, included in this repository. You can read more information about how the robot was built reading the Report.pdf.

## Previous requeriments
* Install the Arduino IDE.
* Download to your PC the [Arduino Code](https://github.com/dannyel2511/robotics-final-project/tree/dannyel/arduino_interface). Compile it and upload the program to the Arduino DUE in the robot.
 * Install ROS Kinetic in a computer running Ubuntu 16.04 or Linux Mint 18.3 http://wiki.ros.org/kinetic/Installation
 * Install ROS Kinetic in the onboard computer (Raspberry PI recommended) running Ubuntu Mate. Make sure that the SSH service is enabled. You can follow the steps in [Install Ubuntu Mate  on Raspberry PI](http://www.hospitableit.com/howto/installing-ubuntu-mate-16-04-2-lts-on-a-raspberry-pi-3/)
 * In the onboard computer, install the rosserial and rosserial-arduino packages.
    ```sh
    sudo apt-get install ros-kinetic-rosserial-arduino
    sudo apt-get install ros-kinetic-rosserial
    ```
* Clone this repository in your PC and copy all the contents from the ```ROS_PC/project_shark``` to your ```catkin workspace```.
    ```sh
    git clone https://github.com/dannyel2511/robotics-final-project.git
    cp -Rf robotics-final-project/ROS_PC/project_shark ~/catkin_ws/src
    ```
* In the raspberry, clone this repository and copy all the contents from the ```ROS_Raspberry/project_shark``` to your ```catkin workspace```.
    ```sh
    git clone https://github.com/dannyel2511/robotics-final-project.git
    cp -Rf robotics-final-project/ROS_Raspberry/project_shark ~/catkin_ws/src
    ```
* Teleoperation:
    * In the PC, install the **teleop_twist_keyboard** to be able to control the robot remotely
        ```sh
        sudo apt-get install ros-kinetic-teleop-twist-keyboard
        ```
* Semi and full autonomous navigation
    * In the onboard computer, install the **RPLIDAR node**. Follow the steps in [Install RPLIDAR node](https://github.com/robopeak/rplidar_ros)
    * Install the **hector_slam** package
        ```sh
        sudo apt-get install ros-kinetic-hector-slam ros-kinetic-hector-geotiff ros-kinetic-hector-geotiff-plugins
        ```
    * Install the **nav2d package**
        ```sh
        apt-get install ros-kinetic-nav2d
        ```
# Manual operation

**1. Check that both batteries are charged:**
* Power bank, it has 4 blue LEDs indicating the percentage of charge.
* Principal battery, measure the voltage using a voltimeter at the terminals in the back of the robot.
Check out that all wires are well connected in their corresponding pins, you can refer to the labels attached to both ends to check where they must be connected.

**2. Turn on both switches at the top of the robot.**
Verify that both LEDs turn on. If not, return to step one.
Red LED indicates that the circuit has been energized. Blue LED indicates that the motors are turned on.

**3. Use the joystick to move the robot**
Press and hold the joystick to activate the manual operation. Then, move it in the desired direction and the robot wil start moving at full speed in that direction. It can go forward, backward, turn left and rigth.

**4. To stop the manual operation release the joystick**
The robot will stop responding to the manual operation. Then, you can turn off both switches to save energy.

# Teleoperation
It is possible to control the robot from a remote computer, the only requirement is that both computers (remote PC and onboard computer) are connected to the same network. Follow these steps to start the teleoperation mode of the robot.
**1. Connect both computers to the same network (LAN)**. Verify that they can communicate each other by doing a ping to the IP assigned.
**2.Modifyt the ```.bashrc``` in both computers**. You need to indicate where the ROSCORE will be running, in this case it will run on the PC. Moreover, the onboard computer has to be able to communicate with it. To do this, it is neccesary to change three ROS environment variables. Run this commands:
**On PC**
```sh
echo "export ROS_IP=Your_PC_IP" >> ~/.bashrc
echo "export ROS_HOSTNAME=Your_PC_IP" >> ~/.bashrc
echo "export ROS_MASTER_URI=http://Your_PC_IP:11311" >> ~/.bashrc
source ~/.bashrc
```
**On the onboard computer**
```sh
echo "export ROS_IP=Your_Raspberry_IP" >> ~/.bashrc
echo "export ROS_HOSTNAME=Your_Raspberry_IP" >> ~/.bashrc
echo "export ROS_MASTER_URI=http://Your_PC_IP:11311" >> ~/.bashrc
source ~/.bashrc
```
**_Note_**: Substitute Your_PC_IP and Your_Raspberry_IP by the corresponding values. You need to run these instructions every time your network parameters change. If you are using static IPs, you do not need to worry about running these commands more than once.

## Semi autonomous operation
The semiautonomous operation consists in avoiding obstacles by modifying the initial route of the robot. It is achieved by getting data from the environment using a RPLIDAR camera and using this data to make a planning of the route. To do this, the nav2d package is necessary.
Follow these steps to start running
1. Go to yout catkin_ws
```sh
$ cd ~/catkin_ws
$ catkin_make
```

## Autonomous operation

> **Created by**:
> Daniel Jeronimo Gomez Antonio
> Esteban Abiel Rico Garcia
> Andres Cortez Villao
> Rodrigo Herrera Ponce
> Luis Rodrigo Garcia Hernandez
