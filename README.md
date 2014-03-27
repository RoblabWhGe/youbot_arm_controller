youbot_arm_controller
=====================
This GUI was developed to control the youBot arm. It provides a simple wrapper to the youBot API which just focusses the arm operations.
The main purpose is to provide students a simple human machine interface in which they can developed basis robotic algorithms and test their solutions.
The GUI also provides an offline simulator thus you doesn't have to connect the manipulator all the time.

The kinematics solver isn't implemented yet due to educational reasons. Feel free to implement it by yourself or wait till we provide the solution :)

![gui_screenshot](http://s1.directupload.net/images/140326/zrrbxf7m.png "The arm controller interface in action")

#Needed system dependencies
Eigen, QT and Boost are required to ruild and run the software.
* sudo apt-get install qt4-dev-tools libboost-dev libboost-thread-dev libboost-filesystem-dev libboost-regex-dev libeigen3-dev

## Installation
To compile the programm simply run the following commands:
* mkdir build
* cd build
* cmake ..
* make

## Usage 
You can run the programm in simulation mode or with a connected youBot arm.
In case you want to control a connected arm, you have to run the program with root permissions 

