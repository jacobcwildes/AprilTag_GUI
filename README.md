# AprilTag_GUI

## Purpose:

Utilize preexisting ROS2 AprilTag codebases and create a GUI to show which tag is being shown, identify the tag ID, tag family, and draw a box around it.

### To install from source:
*************************************************************************
Create a ROS2 workspace, i.e. apriltag_ws/src
```
$ sudo mkdir apriltag_ws/src
```
#### Note: Make sure that all packages are installed to the source folder

Install the ROS2 AprilTag core from Christian Rauch's repository: https://github.com/christianrauch/apriltag_ros
```
$ git clone https://github.com/christianrauch/apriltag_ros
```

Next, install the ROS2 message definitions: https://github.com/christianrauch/apriltag_ros
```
$ git clone https://github.com/christianrauch/apriltag_ros
```
Install the GUI and time synchronizer from this repository
```
$ git clone https://github.com/jacobcwildes/AprilTag_GUI
```

Source the ROS2 workspace
```
$ source /opt/ros/foxy/setup.bash
```

Build
```
$ colcon build
```

*************************************************************************

### To run:
*************************************************************************
First, source the workspace:
```
$ source apriltag_ws/install/setup.bash
```
Then run:
```
$ ros2 run image_tools cam2image --ros-args -p device_id:=0 -p width:=640 -p height:=480 -r /image:=/camera/image

$ ros2 run py_pysub talker --ros-args -r /camera_info:=/camera/camera_info

$ ros2 launch apriltag_ros tag_16h5_all.launch.py

$ ros2 run cam_subscriber listener
```
*************************************************************************
