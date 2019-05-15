## 0.1. 备忘录
### 0.1.1. 安装turtlebot的问题
* 安装不完整，应该多次尝试，单独安装提示安装失败的包。
* catkin_make编译失败。 `rocon`, `kobuki`和`turtlebot`这三个工作空间按照依赖关系应该按顺序编译，在每一个编译前将前面的加入环境变量
  ```bash
  source xx/devel/setup.bash
  ```
* 遇到```bash pyrcc4/5 not found```的情况，可以安装
  ```bash
  apt-get install pyqt4-dev-tools pyqt5-dev-tools
  ```
<br/>

### 0.1.2. ROS备忘录
#### 0.1.2.1. 基本操作
* 创建工作空间：
  ```bash
  mkdir -p catkin_ws/src
  cd catkin_ws
  catkin_make
  ```

* 创建package：
  ```bash
  cd src
  catkin_create_pkg pack rospy roscpp std_msgs
  ```
<br/>

#### 0.1.2.2. 消息与服务
* 编译必须在工作区间根目录，如`catkin_ws`

* 编译之前，要注意包中的`CMakeLists.txt`中`generate_messages()`必须在`catkin_package()`之前执行。
  > 包中的`package.xml`中不能含有`run_depend`项，因此`CMakeLists.txt`中的`catkin_package()`中的`CATKIN_DEPENDS`项也不能有`message_runtime`.
  >因此只需要写
  >```xml
  ><build_depend>message_generation</build_depend>
  >```
  对应于`find package()`中。
  相应的依赖都需要写进`package.xml`中.

* 运行程序前注意
  ```bash
  source <ws>/devel/setup.bash
  ```

* 采用`python`编写`msg`或者`srv`，要注意将相关文件设为可执行
  ```bash
  sudo chmod a+x xxserver/client/talker/listener.py
  ```

* 采用数组作为`srv`参数时，如
  ```bash
  int64[] a
  int64[] b
  ---
  int64 res
  ```
  格式要求比较严格，_注意空格的位置_
  <br/>

* [注意各种构造函数原型](http://docs.ros.org/kinetic/api/roscpp/html/namespaceros.html)
  * `rospy.Subscriber`/`ros::Subscriber`
    ```python
    __init__(self, name, data_class, callback=None, callback_args=None, queue_size=None, buff_size=65536, tcp_nodelay=False)
    ```
    ```cpp
    Subscriber(const std::string &                    topic,
               const NodeHandle &                     node_handle,
               const SubscriptionCallbackHelperPtr &  helper 
              ) 
    ```
    > 其中`callback_args`为订阅器回调函数除消息外的其它参数。

  * `rospy.Publisher`/`ros::Publisher`
    
    ```python
    __init__(self, name, data_class, subscriber_listener=None, tcp_nodelay=False, latch=False, headers=None, queue_size=None)
    ```
    ```cpp
    Publisher()
  ```
  
* 发布和订阅ROS图片
  ROS图片是作为`sensor_msgs/Image`或者`sensor_msgs/ImagePtr`格式发布的，`cv::Mat`[可以这样转换](http://wiki.ros.org/image_transport/Tutorials/PublishingImages)：
  ```cpp
  #include <ros/ros.h>
  #include <image_transport/image_transport.h>
  #include <opencv2/highgui/highgui.hpp>
  #include <cv_bridge/cv_bridge.h>
  ros::init(argc, argv, "image_publisher");
  ros::NodeHandle nh;
  void imageCallback(const sensor_msgs::ImageConstPtr& msg){}
  cv::Mat img;
  // ...
  image_transport::ImageTransport it(nh);
  image_transport::Publisher pub = it.advertise("camera/image", 1);  // publisher
  image_transport::Subscriber sub = it.subscribe("in_image_base_topic", 1, imageCallback);  // subscriber
  sensor_msgs::ImagePtr msg = cv_bridge::CvImage(std_msgs::Header(), "bgr8", image).toImageMsg();
  pub.publish(msg);
  // 其中"bgr8代表" CV_8UC3，灰度图可以使用"8UC3"或者TYPE_8UC3( = "8UC3")。 发布的msg是指针类型。
  ```
  此处应该注意采用的是`image_transport`模块代替`ROS::Publisher`，采用官网原说明，这样做的好处是：

  > image_transport publishers advertise individual ROS Topics for each available transport - unlike ROS Publishers, which advertise a single topic. The topic names follow a standard naming convention, outlined below. Note, however, that all code interfaces take only a "base topic" name (to which the transport type is automatically appended); typically you should not directly reference the transport-specific topic used by a particular plugin. 

  各种函数原型：
  ```cpp
  // image_transport: sensor::msgs::ImageConstPtr to cv::Mat
  CvImagePtr cv_bridge::toCvCopy(const sensor_msgs::Image &, const std::string& encoding  = std::string());  // 拷贝  sensor_msgs to cv_bridge
  CvImagePtr cv_bridge::toCvCopy(const sensor_msgs::ImagePtr &, const std::string& encoding  = std::string());  // 拷贝  sensor_msgs to cv_bridge
  ImagePtr cv_bridge::toImageMsg() const;  // cv_bridge to sensor_msgs
  cv_bridge::toImageMsg(sensor_msgs::Image &) const;  // cv_bridge to sensor_msgs
  CvImageConstPtr toCvShare(const sensor_msgs::ImageConstPtr& source, const std::string& encoding = std::string());  // 共享（共享内存）
  CvImageConstPtr toCvShare(const sensor_msgs::Image& source, 
                            const boost::shared_ptr<void const>& tracked_object, const std::string& encoding = std::string());
  // image_transport: cv::Mat to sensor::msgs::ImageConstPtr
  ```

* 消息队列
  使用订阅器时，如果回调函数中有类似`rospy.sleep()`之类的语句，node会__暂停接受消息__。如果规定了消息队列的容量进行测试就会发现，一串消息会同时进入消息队列依次处理，__直到此队列为空时下一串消息才会进入__，这种机制导致不能以一定频率处理单个消息，除非消息队列容量设为1。如果设置`queue_size=None`，相当于消息队列无穷大。
  

#### 0.1.2.3. SimpleActionClient
##### 1.1.2.3.0. Items to append
* xxx.action
  ```txt
  # define goal
  int32 order
  ---
  # define result
  int32 res
  ---
  # define feedback
  float32 fback
  ```

* package.xml
  ```xml
  <build_depend>actionlib</build_depend>
  <build_depend>actionlib_msgs</build_depend>
  <exec_depend>actionlib</exec_depend>
  <exec_depend>actionlib_msgs</exec_depend>
  ```

* CMakeLists.txt
  ```cmake
  find_package(catkin REQUIRED COMPONENTS
  genmsg
  message_generation
  actionlib_msgs
  actionlib
  )
  #specify action file
  add_action_files(
  FILES
  xxx.action
  )
  #generate messages
  generate_messages(
  DEPENDENCIES
  actionlib_msgs
  )
  #catkin pkg
  catkin_package(
  CATKIN_DEPENDS actionlib_msgs actionlib
  )
  ```

* source files
  1. Client
     ```cpp
     #include <actionlib/client/simple_action_client.h>  //catkin generated source
     #include <monitor/AlarmAction.h>
     actionlib::SimpleActionClient<pkg::xxxAction> xxxClient("topic_name", true);  // true -> don't need ros::spin()
     xxxClient.waitForServer();
     pkg::xxxGoal goal;
     // TODO: specify goal
     xxxClient.sendGoal(goal);
     // ...
     ```

     ```python
     
```
     
  2. Server
     ```cpp
   ros::ServiceServer server = nh.advertiseService(topicname, callback)
  ```
     
     ```python
     import monitor.msgs
   import roslib
  import actionlib
     
     rospy.init_node('kobuki_actSrv')
     self.server = actionlib.SimpleActionServer(self._name, AlarmAction, 
            execute_cb=self.actCallback, auto_start=False)
     self._feedback = monitor.msg.AlarmFeedback()  # feedback during the process
     self._res = monitor.msg.AlarmResult()  # final result
     self.rate = rospy.Rate(1)
   self.server.start()
  rospy.spin()
   
  def actCallback(self, goal):
     
        for i in range(0, goal.order):
            self._feedback.fback = 0
          self.server.publish_feedback(self._feedback)
         self.rate.sleep()
     
        self._res.res = 1
        self.server.set_succeeded(self._res)
     ```


### 0.1.3. kinect v1
```
resolution:
    color camera – 640 * 480 @ 30fps
    depth camera – 320 * 240
depth distance: 0.4-4.5m
field of view: 57 degrees (horizontal), 43 degrees (vertical)
```
* 使用`kinect`之前，需要安装[`freenect`](https://github.com/OpenKinect/libfreenect)

* __防止`USB autosuspend`__：
  ```bash
  sudo sh -c "echo -1 > /sys/module/usbcore/parameters/autosuspend"
  ```

* `ROS`获取`kinect`视频流：
  ```bash
  roslaunch freenect_launch freenect-registered-xyzrgb.launch
  ```
  ```bash
  roslaunch freenect_launch freenect.launch
  ```

* `freenect` __不支持多进程访问__，需要通过`ROS`调用`kinect`，需要的依赖为`sensor_msgs`, `cv_bridge`和`image_transport`
  1. 在`package.xml`中加入如下语句：
     ```xml
     <build_depend>sensor_msgs</build_depend>
     <build_depend>cv_bridge</build_depend>
     <build_depend>image_transport</build_depend>
     <build_export_depend>sensor_msgs</build_export_depend>
     <build_export_depend>cv_bridge</build_export_depend>
     <build_export_depend>image_transport</build_export_depend>
     <exec_depend>sensor_msgs</exec_depend>
     <exec_depend>cv_bridge</exec_depend>
     <exec_depend>image_transport</exec_depend>
     ```
  2. 在`CMakeLists.txt`中加入如下语句：
     ```cmake
     find_package(catkin REQUIRED COMPONENTS
       sensor_msgs
       cv_bridge
       image_transport
     )
      generate_messages(
       DEPENDENCIES
      sensor_msgs
     )
     ```
  3. 调用库文件
     ```cpp
     #include "cv_bridge/cv_bridge.h"
     #include "image_transport/image_transport.h"
     ```
     在编写的`spy_client.cpp`文件中：
     主程序中添加语句
     ```cpp
     ros::spin();
     // same effects
     while(ros::ok()){ros::spinOnce();}
     ```
     __不要直接使用未初始化的全局`Mat`变量作为`cv_bridge::toCvCopy()`的返回值__，否则调用将失败。
     向`server`传递`std::vector`，在编写的`spy_server.py`中，注意模型的输入是四维张量，可以将多个图片数据一并输入进行预测。
<br/>

### 0.1.4. keras/tensorflow
1. error:
   ```bash
   Tensor Tensor("dense_2/Softmax:0", shape=(?, 2), dtype=float32) is not an element of this graph.
   ```
   solution:
   ```python
   import tensorflow as tf
   global graph
   graph = tf.get_default_graph()
   with graph.as_default():
   feature = model.predict_classes(img)
   ```
<br/>

### 0.1.5. Turtlebot
1. Unable to connect to Turtlebot
   ```bash
   roscore
   ls /dev/kobuki
   #not exist
   rosrun kobuki_ftdi create_udev_rules
   ```

2. Packages and dependencies:
   * install gmapping
     ```bash
     sudo apt-get install ros-kinetic-gmapping
     ```
   * install package depthimage (in workspace 'turtlebot')
     ```bash
     cd turtlebot/src
     git clone https://github.com/ros-perceptron/depthimage_to_laserscan
     cd ..
     catkin_make
     ```

3. Navigation
   ```bash
   roslaunch turtlebot_bringup minimal.launch
   roslaunch turtlebot_navigation gmapping_demo.launch
   roslaunch turtlebot_rviz_launchers view_navigation.launch
   roslaunch turtlebot_teleop keyboard_teleop.launch (teleop to move)
   rosrun map_server map_saver -f /tmp/my_map
   ```

4. Autonomous Driving
   ```bash
   roslaunch turtlebot_bringup minimal.launch
   roslaunch turtlebot_navigation amcl_demo.launch map_file:=xx.yaml
   #note that amcl_demo.launch acesses kinect at the beginning, but no longer does after odom received, which means that package monitor could work.)
   roslaunch turtlebot_rviz_launchers view_navigation.launch --screen    # launching rviz
   ```
<br/>


### 0.1.6 [darknet_ros](https://github.com/leggedrobotics/darknet_ros)
* Bugs
  1. cfg files are not properly parsed, raising the following error:
     ```bash
     First section must be [net] or [network]: Resource temporarily unavailable
     ```
     Solution: Modify the `list *read_cfg(char*)` function in `darknet/src/parser.c`.
     ```cpp
     // ...
     while((line=fgetl(file)) != 0){
        ++ nu;
        strip(line);
        line[strcspn(line, "\r\n")] = '\0';  // the line added
        switch(line[0]){
          // ...
        }
        // ...
     }
     ```

* Configuration
  1. Specify image message from camera.
     
     Modify file `darknet_ros/config/ros.yaml` (which should be parsed by `rosparam` when launching node `yolo_object_detector_node`), specifying `subscriber/camera_reading/name`.

  2. Specify network and corresponding weight file.

     Modify launch file (e.g. `darknet_ros/launch/darknet_ros.launch`), specifying argument `network_param_file`.

* Messages and SimpleActions
  
  > The node `yolo_object_detector_node` implements a couple of message and action servers/clients with some shared variables and exploy threads to avoid conflicts. 

  * imageSubscriber_           --reading images from an external publisher
  ```cpp
  image_transport::ImageTransport imageTransport_;
  image_transport::Subscriber imageSubscriber_  = imageTransport_.subscribe(cameraTopicName, cameraQueueSize, &YoloObjectDetector::cameraCallback, this);
  ```

  * objectPublisher_           --publishing detected objects
  ```cpp
  ros::NodeHandle nodeHandle_;
  ros::Publisher objectPublisher_ = nodeHandle_.advertise<std_msgs::Int8>(objectDetectorTopicName, objectDetectorQueueSize, objectDetectorLatch);
  ```

  * boundingBoxesPublisher_    --publishing bboxes
  ```cpp
  ros::Publisher boundingBoxesPublisher_ = nodeHandle_.advertise<darknet_ros_msgs::BoundingBoxes>( boundingBoxesTopicName, boundingBoxesQueueSize, boundingBoxesLatch);
  ```

  * detectionImagePublisher_   --publishing raw image with bboxes
  ```cpp
  ros::Publisher detectionImagePublisher_ = nodeHandle_.advertise<sensor_msgs::Image>(detectionImageTopicName, detectionImageQueueSize, detectionImageLatch);
  ```

  * checkForObjectsActionServer_    --accepting an image and return bboxes as result
  ```cpp
  typedef actionlib::SimpleActionServer<darknet_ros_msgs::CheckForObjectsAction> CheckForObjectsActionServer;
  typedef std::shared_ptr<CheckForObjectsActionServer> CheckForObjectsActionServerPtr;
  CheckForObjectsActionServerPtr checkForObjectsActionServer_;
  checkForObjectsActionServer_.reset(
      new CheckForObjectsActionServer(nodeHandle_, checkForObjectsActionName, false));
  checkForObjectsActionServer_->registerGoalCallback(
      boost::bind(&YoloObjectDetector::checkForObjectsActionGoalCB, this));
  checkForObjectsActionServer_->registerPreemptCallback(
      boost::bind(&YoloObjectDetector::checkForObjectsActionPreemptCB, this));
  checkForObjectsActionServer_->start();
  ```

* Modification
  Added `bboxImagePublisher_`
  ```cpp
  // inside function 'void* darknet_ros::publishInThread()'
  bboxImageResults_.bboxes = boundingBoxesResults_;
  sensor_msgs::ImagePtr sensor_img = cv_bridge::CvImage(std_msgs::Header(), 
        sensor_msgs::image_encodings::BGR8, cvImage.clone()).toImageMsg();
  // do not use shared memory of cvImage (the msg would fail to be published).
  bboxImageResults_.img = *sensor_img;
  bboxImagePublisher_.publish(bboxImageResults_);
  ```

### 0.1.7. Markdown
* Comments and empty lines at the end of code blocks not allowed.
* Use indent to create nested lists. The contents should be at least aligned left with the sub title. (typically 3 ch for ordered, 2 for unordered)

### 0.1.8. Cuda
* Cuda installation
  1. Download `cuda` runfile and install
     ```bash
     sh cuda.run  # follow the commands
     ```
     __Do not install OpenGL if Nvidia GPU is not used for display.__

  2. Install `cudnn`
     Download `cudnn` file and unzip. copy the files to `cuda` directory.

  3. Set as path
     ```bash
     echo 'PATH=cuda_dir/bin:$PATH' >> ~./bashrc
     echo 'PATH=cuda_dir/lib64:$PATH' >> ~./bashrc
     ```

* `OpenCV` with `cuda` (version 3.1.0)
  * Problems encountered compiling opencv with cuda
    1. `CUDA_nppi_LIBRARY (ADVANCED)` set to NOTFOUND
       ```bash
       CMake Error: The following variables are used in this project, but they are set to NOTFOUND.
       Please set them or make sure they are set and tested correctly in the CMake files:
       CUDA_nppi_LIBRARY (ADVANCED)
       ```

       Solution: modifying `opencv/cmake/FindCUDA.cmake && OpenCVDetectCUDA.cmake`
       find the three lines below:
       ```cmake
       find_cuda_helper_libs(nppi)

       set(CUDA_npp_LIBRARY "${CUDA_nppc_LIBRARY};${CUDA_nppi_LIBRARY};${CUDA_npps_LIBRARY}")

       unset(CUDA_nppi_LIBRARY CACHE)
       ```

       and change them to
       ```cmake
       find_cuda_helper_libs(nppial)
       find_cuda_helper_libs(nppicc)
       find_cuda_helper_libs(nppicom)
       find_cuda_helper_libs(nppidei)
       find_cuda_helper_libs(nppif)
       find_cuda_helper_libs(nppig)
       find_cuda_helper_libs(nppim)
       find_cuda_helper_libs(nppist)
       find_cuda_helper_libs(nppisu)
       find_cuda_helper_libs(nppitc)

       set(CUDA_npp_LIBRARY "${CUDA_nppc_LIBRARY};${CUDA_nppial_LIBRARY};${CUDA_nppicc_LIBRARY};${CUDA_nppicom_LIBRARY};${CUDA_nppidei_LIBRARY};${CUDA_nppif_LIBRARY};${CUDA_nppig_LIBRARY};${CUDA_nppim_LIBRARY};${CUDA_nppist_LIBRARY};${CUDA_nppisu_LIBRARY};${CUDA_nppitc_LIBRARY};${CUDA_npps_LIBRARY}")

       unset(CUDA_nppial_LIBRARY CACHE)
       unset(CUDA_nppicc_LIBRARY CACHE)
       unset(CUDA_nppicom_LIBRARY CACHE)
       unset(CUDA_nppidei_LIBRARY CACHE)
       unset(CUDA_nppif_LIBRARY CACHE)
       unset(CUDA_nppig_LIBRARY CACHE)
       unset(CUDA_nppim_LIBRARY CACHE)
       unset(CUDA_nppist_LIBRARY CACHE)
       unset(CUDA_nppisu_LIBRARY CACHE)
       unset(CUDA_nppitc_LIBRARY CACHE)
       ```
       
       Whthin OpenCVDetectCUDA.cmake find lines with `__cuda_arch_bin` and delete the unsupported 1.x and 2.x versions.
       <br/>
       
    2. Errors regarding source code
       ```bash
       /home/fran/opencv/modules/cudalegacy/src/graphcuts.cpp:120:54: error: ‘NppiGraphcutState’ has not been declared
       ```

       Solution:
       find the file `/home/fran/opencv/modules/cudalegacy/src/graphcuts.cpp` and modify the first line:
       ```cpp
       #if !defined (HAVE_CUDA) || defined (CUDA_DISABLER)
       ```

       change it to
       ```cpp
       #if !defined (HAVE_CUDA) || defined (CUDA_DISABLER) || (CUDART_VERSION >= xxxx)
       # specify xxxx which represents the cuda version. For cuda9.x, use 9000.
       ```