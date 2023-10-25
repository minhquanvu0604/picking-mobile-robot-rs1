# picking-mobile-robot-rs1
## This is a robotic project serving the Robotics Studio 1 subject. 
===================================


This is a meta-package that contains multiple packages regarding the components of the Warehouse Picking Robot.

Please [create individual packages](http://wiki.ros.org/ROS/Tutorials/CreatingPackage) in this directory (the same level of this README.md file).

## Dependencies 
[Battery Plugin](https://github.com/nilseuropa/gazebo_ros_battery) - Clone and build in catkin workspace

## Usage
### Start Gazebo Simulation
```
roslaunch picking_mobile_robot_rs1_gazebo amazon_warehouse.launch
```
### Start Navigation Node

To use AMCL only
```
roslaunch picking_mobile_robot_rs1_gazebo navigation.launch
```
To use RTABMAP for SLAM, use the below code. This will continue the existing database.
```
roslaunch picking_mobile_robot_rs1_gazebo navigationRTABmap.launch
```
If wanting to test a new database, set the new_db parameter to true
```
roslaunch picking_mobile_robot_rs1_gazebo amazon_warehouse.launch new_db:=true
```