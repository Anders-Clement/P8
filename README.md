### P8


## Installation instructions

ROS installation: 
install ros-noetic-desktop-full as per the instructions on the ROS wiki site

additional installs:
sudo apt install ros-noetic-rosbridge-suite
sudo apt install ros-noetic-moveit
sudo apt install ros-noetic-industrial-trajectory-filters

UR10 repositories must be installed after cloning, do the following from the root of this workspace:
```
# clone the driver
$ git clone https://github.com/UniversalRobots/Universal_Robots_ROS_Driver.git src/Universal_Robots_ROS_Driver

# clone fork of the description. This is currently necessary, until the changes are merged upstream.
$ git clone -b calibration_devel https://github.com/fmauch/universal_robot.git src/fmauch_universal_robot

# install dependencies
$ sudo apt update -qq
$ rosdep update
$ rosdep install --from-paths src --ignore-src -y

# build the workspace
$ catkin_make

# activate the workspace (ie: source it)
$ source devel/setup.bash
```

## Running UR10 simulation in gazebo
build, source and run in two terminals:
```
roslaunch rob8 ur10_bringup.launch
roslaunch rob8 ur10_controllers.launch
```

## Install ros driver for Astra camera
```
$ sudo apt install ros-noetic-rgbd-launch ros-noetic-libuvc-camera ros-noetic-libuvc-ros

# go to src folder in work space
$ git clone https://github.com/orbbec/ros_astra_camera

$ roscd astra_camera
$ ./scripts/create_udev_rules

# go to root folder
$ catkin_make

# use camera:
$ roslaunch astra_camera astra.launch
```

