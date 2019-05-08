#pragma once
#include "ros/ros.h"
#include "cv_bridge/cv_bridge.h"
#include "image_transport/image_transport.h"
#include "std_msgs/Bool.h"  // necessary if std_msgs::Bool is required
#include "opencv2/opencv.hpp"
#include "opencv2/highgui.hpp"
#include "opencv2/imgproc.hpp"
#include <pthread.h>
#include <string>
using namespace cv;

class rgbData{
private:
    image_transport::ImageTransport it;
    image_transport::Subscriber rgbSub;
    cv::Mat *rgbMat;
    std::string imgSrc;
    int qsize;
public:
    rgbData(cv::Mat&, ros::NodeHandle&);  // message queue size and shape of the dst image;
    ~rgbData(){};
    void imgcallback(const sensor_msgs::ImageConstPtr&);
};
