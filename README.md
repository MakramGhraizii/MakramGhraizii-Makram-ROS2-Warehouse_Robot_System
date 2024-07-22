# MakramGhraizii-Makram-ROS2-Warehouse_Robot_System

## Description
This ROS 2 package simulates a warehouse robot system where the robot can deliver items and check stock levels.

## Nodes

### Item Delivery Action Server
Handles item delivery requests.

### Stock Checker Service Server
Checks the stock level of a requested item.

### Item Delivery Action Client
Sends delivery requests to the action server.

### Stock Checker Service Client
Sends stock check requests to the service server and visualizes stock levels.

## Installation
```sh
cd ~/ros2_ws/src
git clone <repository_url>
cd ~/ros2_ws
colcon build
source install/setup.bash
```
## Launch the Package
```sh
ros2 launch warehouse_robot launch.py item_name:=item1 quantity:=40

```
