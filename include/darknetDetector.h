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

// CXX
#include <stdlib.h>
#include <unistd.h>
#include <limits>
#include <chrono>
#include <pthread.h>
#include <boost/thread.hpp>
#include <boost/thread/locks.hpp>
#include <vector>
#include <string>
#include <algorithm>
#include <iomanip>
#include <fstream>

namespace darknet_svm{
class RateNP: public ros::Rate{
public:
    RateNP(): ros::Rate(1.){};
    RateNP(double freq): ros::Rate(freq){};
};

class darknet_svm{
private:
    float ovthresh;
    // shared variables
    smoke::BoundingBoxes bboxes;
    //smoke::BoundingBoxes bounding_boxes, bounding_boxes_u;
    std::vector<smoke::BoundingBox> bounding_boxes, bounding_boxes_u;
    std::vector<int> bounding_box_u_response;

    cv_bridge::CvImage img_bridge;
    cv_bridge::CvImagePtr img_bridge_sub;
    sensor_msgs::Image imgs;
    sensor_msgs::Image img_srv;
    std_msgs::Header header;
    
    cv::Mat img, obj;
    cv::Mat objLBP, tmp;
    ros::NodeHandle nh_;

    bool isNodeRunning;
    bool subscriberStatus;
    bool srvStatus;
    std_msgs::Bool alarm;

    // mutex and thread
    boost::shared_mutex mutexImgsStatus;  // 'image' and 'bbox' messages
    boost::shared_mutex mutexNodeStatus;  // 'isNodeRunning'
    boost::shared_mutex mutexAlarmStatus;  // 'alarm'
    //boost::shared_mutex mutexSrvStatus;
    boost::shared_mutex mutexSubscriberStatus;  // 'subscriberStatus'
    boost::shared_mutex mutexBboxesUStatus;  // 'bounding_boxes_u'
    boost::shared_mutex mutexLogWriterStatus; // 'logwriter'
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

    // thread sleep
    RateNP subscriberStatusDelay;
    RateNP mainThreadDelay;
    RateNP imgCallbackDelay;

    int count;
    float htthresh, wdthresh;
    float chim_prob_thresh;

    // log file writer
    std::ofstream logwriter;
    std::string logfile;
    std::string logpath;

    //functions
    void bboxCallback(const smoke::BoundingBoxes&);
    void imgCallback(const sensor_msgs::Image&);
    void bboxImgCallback(const smoke::BboxImageConstPtr&);
    static void *callSVMInThread(void*);
    static void *alarmInThread(void*);
    static void *workInThread(void*);
    bool getNodeStatus();
    bool getSubscriberStatus();

public:
    darknet_svm(ros::NodeHandle nh);
    ~darknet_svm();
    void init();
};
} // namespace 'darknet_svm'