#pragma once
// ROS
#include <std_msgs/Header.h>
//#include "smoke/BoundingBox.h"
//#include "smoke/BoundingBoxes.h"
#include "smoke/BboxImage.h"
#include "smoke/darknet_svm_node.h"
#include <cv_bridge/cv_bridge.h>
#include <std_msgs/Bool.h>

// OpenCV
#include "image_transport/image_transport.h"
#include "opencv2/opencv.hpp"
#include "opencv2/highgui.hpp"
#include "opencv2/imgproc.hpp"
//#include "LBP.h"

// CXX
#include <chrono>
#include <pthread.h>
#include <boost/thread.hpp>
#include <boost/thread/locks.hpp>
#include <vector>
#include <string>

namespace darknet_svm{
class darknet_svm{
private:
    // shared variables
    smoke::BoundingBoxes bboxes;
    std::vector<smoke::BoundingBox> bounding_boxes;

    cv_bridge::CvImage img_bridge;
    sensor_msgs::Image imgs;
    std_msgs::Header header;
    std::vector<sensor_msgs::Image> imgsVec;
    
    cv::Mat img, obj;
    cv::Mat objLBP, tmp;
    ros::NodeHandle nh_;

    bool isNodeRunning;
    bool subscriberStatus;
    bool srvStatus;
    std_msgs::Bool alarm;

    // mutex and thread
    boost::shared_mutex mutexImgsStatus;
    boost::shared_mutex mutexNodeStatus;
    boost::shared_mutex mutexAlarmStatus;
    boost::shared_mutex mutexSrvStatus;
    boost::shared_mutex mutexSubscriberStatus;
    pthread_t mainThread;
    int callSVMThreadStatus;
    int alarmThreadStatus;

    // subscribers
    ros::Subscriber img_bbox_sub;

    // publisher
    ros::Publisher alarm_pub;

    // service client
    smoke::darknet_svm_node svm_srv;
    ros::ServiceClient sc;

    int count;
public:
    darknet_svm(ros::NodeHandle nh);
    ~darknet_svm();
    void init();
    void bboxCallback(const smoke::BoundingBoxes&);
    void imgCallback(const sensor_msgs::Image&);
    void bboxImgCallback(const smoke::BboxImageConstPtr&);
    static void *callSVMInThread(void*);
    static void *alarmInThread(void*);
    static void *workInThread(void*);
    bool getNodeStatus();
    bool getSubscriberStatus();
};
}