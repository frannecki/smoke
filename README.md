# smoke
ROS package for smoke detection and recognition with Turtlebot (__not complete__).

## Prerequisites
* ROS kinetic
* [darknet_ros](https://github.com/leggedrobotics/darknet_ros)
* OpenCV (>=3.3.0)
* [rbx1](https://github.com/pirobot/rbx1) (optional)

## Usage
* launch webcam using [usb_cam](https://github.com/ros-drivers/usb_cam) package
* launch darknet_ros
  ```
  roslaunch darknet_ros darknet_ros.launch
  ```
* launch nodes of this package
  * launch darknetDetector node
    ```shell
    rosrun smoke darknet_svm_node
    ```
  * launch svm server node
    ```shell
    rosrun smoke svm_server
    ```
  * launch kobuki alarm node
    ```shell
    rosrun smoke kobuki_actSrv.py
    ```
  * launch alarm node
    ```shell
    rosrun smoke alarm_sub.py
    ```
  
* launch files
  * smoke.1.launch
    test with real smoke and real robot

  * darknet_ros_smoke.launch
    launch darknet-ros

  * test_darknet.launch
    test the object detection framework

  * test_act.launch
    test the action server for robot pose inference

  * test_srv.launch
    test the service '/kinectdev/smoke/smoke_svm'

  You can modify the `.launch` file to specify with nodes to run.

## Messages, Services and Actions
```yaml
subscribers:

  camera_reading:
    topic: /camera/rgb/image_rect_color
    queue_size: 1

  alarm_sub:
    topic: /kinectdev/smoke/alarm
    queue_size: 1

  bbox_sub:
    topic: /darknet_ros/bounding_boxes
    queue_size: 1

  img_bbox_sub:
    topic: /darknet_ros/image_bounding_boxes

services:

  image_svm_srv:
    name: /kinectdev/smoke/smoke_svm_srv

  image_srv:
    name: /kinectdev/smoke/smoke_srv

  image_darknet_svm_srv:
    name: /kinectdev/smoke/smoke_darknet_svm_srv


publishers:

  alarm_pub:
    topic: /kinectdev/smoke/alarm
    queue_size: 1
    latch: false

  image_pub:
    topic: /kinedctev/smoke/img_adapted
    queue_size: 1
    latch: false

actions:

  kobuki_alarm:
    name: /kinectdev/smoke/kobuki_alarm
```
  