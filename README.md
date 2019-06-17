# smoke
ROS package for smoke detection and recognition with Turtlebot.

__This package is not sufficient for performing the task.__

## Prerequisites
* ROS kinetic
* [darknet](https://github.com/pjreddie/darknet)
* [darknet_ros](https://github.com/leggedrobotics/darknet_ros)
* OpenCV (>=3.3.0)
* [rbx1](https://github.com/pirobot/rbx1) (optional)
* Qt5 (optional)

## Usage
1. Build libraries and executables (within an ROS workspace)
   ```
   catkin_make -DCATKIN_WHITELIST_PACKAGES="smoke"
   ```
   If you have Qt5 installed, a gui would be built for simple visualization.

   The first build may not finish successfully. Please try again.

2. Run
   * launch webcam using [usb_cam](https://github.com/ros-drivers/usb_cam).
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
     * visualization
       ```shell
       rosrun smoke appviz_node
       ```
  
## launch files
  * `smoke.1.launch`
    
    test with real smoke and real robot

  * `darknet_ros_smoke.launch`
    
    launch darknet_ros

  * `test_darknet.launch`
    
    test the object detection framework

  * `test_act.launch`
    
    test the action server for robot pose inference

  * `test_srv.launch`
    
    test the service '/kinectdev/smoke/smoke_svm'


  You can modify the `.launch` file to specify which nodes to run. Also modify the [config file](config/darknet_svm.yml).
  
