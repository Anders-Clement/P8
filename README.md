### P8


## Installation instructions

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
