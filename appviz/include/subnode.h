#ifndef SUBNODE_H
#define SUBNODE_H
// ROS libraries
#ifndef Q_MOC_RUN
#include <ros/ros.h>
#endif
#include <std_msgs/Header.h>
#include <cv_bridge/cv_bridge.h>
#include <std_msgs/Bool.h>
#include <image_transport/image_transport.h>

// OpenCV libraries
#include "opencv2/opencv.hpp"
#include "opencv2/highgui.hpp"
#include "opencv2/imgproc.hpp"

// Qt libraries
#include <QMainWindow>
#include <QStringListModel>
#include <QPixmap>
#include <QThread>

// CXX standard libraries
#include <string>
#include <vector>

#include "cvqtimage.h"

class QNode : public QThread {
    Q_OBJECT
public:
    QNode(int argc, char** argv );
    virtual ~QNode();
    bool init();
    bool init(const std::string &master_url, const std::string &host_url);
    void run();

    /*********************
    ** Logging
    **********************/
    QStringListModel* loggingModel();
    QPixmap* pixMap();


signals:
    void loggingUpdated();
    void imgUpdated();
    void rosShutdown();

private:
    int init_argc;
    char** init_argv;
    QStringListModel logging_model;
    QPixmap pixmap;

    // variables for ROS node
    cv_bridge::CvImage img_bridge;
    cv_bridge::CvImagePtr img_bridge_sub;
    cv::Mat img;
    ros::NodeHandle nh_;
    ros::Subscriber img_sub, alarm_sub;

    void imgShowCallback(const sensor_msgs::ImageConstPtr&);
    void alarmCallback(const std_msgs::Bool::ConstPtr&);
};

#endif