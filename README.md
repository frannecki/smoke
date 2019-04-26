# smoke
ROS package for smoke detection and recognition with Turtlebot (__not complete__).

## Prerequisites
* ROS kinetic
* [darknet_ros](https://github.com/leggedrobotics/darknet_ros)
* OpenCV (>=3.3.0)

## Usage
* launch webcam using [usb_cam](https://github.com/ros-drivers/usb_cam) package
* launch darknet_ros
  ```
  roslaunch darknet_ros darknet_ros.launch
  ```
* launch nodes of this package
  * launch all nodes
    ```shell
    roslaunch smoke smoke.launch
    ```
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
  