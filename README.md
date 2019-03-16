# alfons_robot_bringup

## Overview

This is a [ROS] package developed for alfons which is a mobile robot that is carlike. This means it is controlled via ackermann steering. This package is actually the main package to bringup the alfons robot it utilizes a launch file that includes the launch file of many other alfons packages (see Dependencies)

This package has been tested under ROS Kinetic and Ubuntu 16.04. This is research code, expect that it changes often and any fitness for a particular purpose is disclaimed.

**Author: Markus Lamprecht<br />
Maintainer: Markus Lamprecht, 2f4yor@gmail.com<br />**

<img alt="alfons" src="data/alfons.png" width="700">

## Installation

### Dependencies

This software is built on the Robotic Operating System ([ROS]), which needs to be [installed](http://wiki.ros.org) first. Additionally, this package depends on following software:

- [alfons_msgs] messages.
- [aruco_detector_osv]
- [aruco_detector_osv]

### Building

In order to install this package, clone the latest version from this repository into your catkin workspace and compile the package using [catkin_tools](https://catkin-tools.readthedocs.io/en/latest/)

    cd catkin_workspace/src
    git clone https://github.com/cesmak/alfons_robot_bringup.git
    cd ..
    catkin init
    catkin build


## Basic Usage

### Main Launch file

### 

## Nodes

### Node: name1

description

#### Subscribed Topics

* **`/points`** ([sensor_msgs/PointCloud2])

    The distance measurements.


#### Published Topics

* **`name`** ([grid_map_msg/GridMap])

     description


#### Services

* **`trigger_fusion`** ([std_srvs/Empty])

    Trigger the fusing process for the entire elevation map and publish it. For example, you can trigger the map fusion step from the console with

        rosservice call /elevation_mapping/trigger_fusion

#### Parameters

* **`point_cloud_topic`** (string, default: "/points")

    The name of the distance measurements topic.

## Bugs & Feature Requests

Please report bugs and request features using the Issue Tracker


[ROS]: http://www.ros.org
[rviz]: http://wiki.ros.org/rviz
[grid_map_msg/GridMap]: https://github.com/anybotics/grid_map/blob/master/grid_map_msg/msg/GridMap.msg
[sensor_msgs/PointCloud2]: http://docs.ros.org/api/sensor_msgs/html/msg/PointCloud2.html
[geometry_msgs/PoseWithCovarianceStamped]: http://docs.ros.org/api/geometry_msgs/html/msg/PoseWithCovarianceStamped.html
[tf/tfMessage]: http://docs.ros.org/kinetic/api/tf/html/msg/tfMessage.html
[std_srvs/Empty]: http://docs.ros.org/api/std_srvs/html/srv/Empty.html
[grid_map_msg/GetGridMap]: https://github.com/anybotics/grid_map/blob/master/grid_map_msg/srv/GetGridMap.srv
[grid_map_msgs/ProcessFile]: https://github.com/ANYbotics/grid_map/blob/master/grid_map_msgs/srv/ProcessFile.srv




















## Demo
* Connect robot camera with PC
* Connect arduino USB with PC
* Start the software: 
    ``` 
    roslaunch alfons_robot_bringup start_robot.launch
    ```
* Turn on Battery (for more force)


## Problems
1) Load code to arduino:
markus@markus:~$ cd ~/Desktop/
markus@markus:~/Desktop$ ./arduino 

2) Try to upload the code to the board in case of error:

My tests:
-> cable is not the problem
-> usb port is not the problem
-> memory is not the problem
-> brocken pin is not the problem
-> Test it with windows arduino program failed

avrdude: ser_open(): can't open device "/dev/ttyACM1": No such file or directory
Problem uploading to board.  See http://www.arduino.cc/en/Guide/Troubleshooting#upload for suggestions.

-> simply go to Tools change device input to ttyACM0
-> Note that you also have to change this in the launch file!

Low memory available, stability problems may occur.
avrdude: stk500_recv(): programmer is not responding
avrdude: stk500_getsync() attempt 1 of 10: not in sync: resp=0x00
avrdude: stk500_recv(): programmer is not responding
avrdude: stk500_getsync() attempt 2 of 10: not in sync: resp=0x00
avrdude: stk500_recv(): programmer is not responding

--> more solutions [here](https://stackoverflow.com/questions/19765037/arduino-sketch-upload-issue-avrdude-stk500-recv-programmer-is-not-respondi)

In the end I bought a new arduino for 20€

avrdude -v -v -v -v
avrdude: Version 6.2
         Copyright (c) 2000-2005 Brian Dean, http://www.bdmicro.
com/
         Copyright (c) 2007-2014 Joerg Wunsch


DO NOT connect lipo battery voltage output and voltage measurement with board at same time!!! causes short cut!!!!!



TODO 
![aruco_detector_example](https://github.com/CesMak/aruco_detector_ocv/blob/master/data/rviz_example.png)

The aruco_detector_osv (aruco_detector_opencv) uses the #include <opencv2/aruco.hpp> library instead of using #include <aruco/aruco.h> (ros-kinetic-aruco). This package was tested on ubuntu 16.04, ROS Kinetic with a Logitech C920 camera. 

With this package you are able to:

* detect position and orientation of an [aruco marker](http://chev.me/arucogen/) relatively to the camera. The corresponding **tf** is published.
* a certainty parameter of how robust the arcuco marker is detected. 
* a result image with the detected markers highlighted is published.

Please calibrate your camera first using: [camera_calibration](http://wiki.ros.org/camera_calibration).

If you get (when using the camera)

``` 
[ERROR] [1551630633.628039127]: Cannot identify '/dev/video0': 2, No such file or directory
```

do 

``` 
sudo modprobe uvcvideo
``` 

See all adjustments in the **aruco_detector_osv/launch/detector.launch** 

## Installation
install dependencies, git, use ros kinetic, ubuntu 16.04.

``` 
cd ~/Desktop
source /opt/ros/kinetic/setup.bash
mkdir -p catkin_ws/src
cd catkin_ws/src/
git clone git@github.com:CesMak/aruco_detector_ocv.git (takes some time due to included bag to test this package)
cd ..
catkin init -w .
catkin build
source devel/setup.bash
roscd roscd aruco_detector_ocv/data/
rosbag decompress 640x480_logitech_aruco3_compressed.orig.bag 
roslaunch aruco_detector_ocv detector.launch 
```


## Launch

``` 
roslaunch aruco_detector_ocv detector.launch 
``` 


## Dependencies:
cv_bridge image_geometry geometry_msgs roscpp rospy std_msgs tf2 tf2_ros image_transport std_msgs

## Further adjustements

There are many opencv parameters that can be adjusted internally to reduce the effect of lightening conditions.
See [opencv_tutorial](https://docs.opencv.org/3.1.0/d5/dae/tutorial_aruco_detection.html)

In order to adjust camera options use **scripts/marker_filter.py**

## License BSD
If you want to use this package please contact: [me](https://simact.de/about_me).

