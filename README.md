
# multi_angle_interface

## Prerequisites

### Install prerequisites from APT repositories
```
sudo apt-get update
sudo apt-get install -y ros-noetic-jsk-perception ros-noetic-rviz-animated-view-controller
```

## Clone the package to your catkin workspace and build the package

In your catkin workspace run:
```
cd ~/catkin_ws
catkin build
```

## Launch the interface at the teleoperator's computer
```
roslaunch multi_angle_interface start_interface.launch
```

## Launch the interface at the vehicles computer
```
roslaunch multi_angle_interface simage_processing.launch
```

