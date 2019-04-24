# smoke
ROS package for smoke detection and recognition with Turtlebot.

## Prerequisites
* ROS kinetic
* [darknet_ros](https://github.com/leggedrobotics/darknet_ros)
* OpencV (>=3.3.0)

## Usage
* launch webcam using [usb_cam](https://github.com/ros-drivers/usb_cam) package
* launch darknet_ros
  ```
  roslaunch darknet_ros darknet_ros.launch
  ```
* launch nodes of this package
  * run all nodes
    ```shell
    roslaunch smoke smoke.launch
    ```
  * run alarm node
  
  You can modify the `.launch` file to specify with nodes to run.
  