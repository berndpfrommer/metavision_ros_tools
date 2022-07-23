# metavision_ros_tools

This repository holds tools for converting metavision event based
camera data under ROS and ROS2

## Supported platforms

Currently tested on Ubuntu 20.04 under under ROS Noetic and ROS2
Galactic. Only tested on Metavision Gen3 sensors, will almost
certainly not work on Gen4 sensors.


## How to build
Create a workspace (``metavision_ros_tools_ws``), clone this repo, and use ``wstool``
to pull in the remaining dependencies:

```
mkdir -p ~/metavision_ros_tools_ws/src
cd ~/metavision_ros_tools_ws
git clone https://github.com/berndpfrommer/metavision_ros_tools.git src/metavision_ros_tools
wstool init src src/metavision_ros_tools/metavision_ros_tools.rosinstall
# to update an existing space:
# wstool merge -t src src/metavision_ros_tools/metavision_ros_tools.rosinstall
# wstool update -t src
```

### configure and build on ROS1:

```
catkin config -DCMAKE_BUILD_TYPE=RelWithDebInfo  # (optionally add -DCMAKE_EXPORT_COMPILE_COMMANDS=1)
catkin build
```

### configure and build on ROS2:

```
cd ~/metavision_ros_tools_ws
colcon build --symlink-install --cmake-args -DCMAKE_BUILD_TYPE=RelWithDebInfo  # (optionally add -DCMAKE_EXPORT_COMPILE_COMMANDS=1)
```

## Tools

### Data conversion tools

These tools convert between Gen3 sensor (EVT 3.0) raw data (.raw
files) and rosbags (``event_array_msgs``).

How to use (ROS1):
```
rosrun metavision_ros_tools bag_to_raw -t /event_camera/events -b foo.bag -o foo.raw
rosrun metavision_ros_tools raw_to_bag -t /event_camera/events -i foo.raw -b foo.bag -f frame_id -w width -h height
```

How to use (ROS2):
```
ros2 run metavision_ros_tools bag_to_raw -t /event_camera/events -b foo.bag -o foo.raw
ros2 run metavision_ros_tools raw_to_bag -t /event_camera/events -i foo.raw -b foo.bag -f frame_id -w width -h height
```

## License

This software is issued under the Apache License Version 2.0.
