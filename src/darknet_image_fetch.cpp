#include "rgbData.h"
using namespace cv;

rgbData::rgbData(cv::Mat& img, ros::NodeHandle& n):it(n)
{
    rgbMat = &img;
    //n.param("/smoke/subscribers/camera_reading/topic", imgSrc, std::string("/camera/rgb/image_rect_color"));
    imgSrc = std::string("/darknet_ros/detection_image");
    n.param("/smoke/subscribers/camera_reading/queue_size", qsize, 1);
    //rgbSub = it.subscribe("/camera/rgb/image_rect_color", qsize, &rgbData::imgcallback, this);
    rgbSub = it.subscribe("/darknet_ros/detection_image", qsize, &rgbData::imgcallback, this);
}

void rgbData::imgcallback(const sensor_msgs::ImageConstPtr& msg){
    cv::Mat tmp;
    try{
        tmp = cv_bridge::toCvCopy(*msg, sensor_msgs::image_encodings::TYPE_8UC3)->image;
        //Do not replace tmp with a global variable not initialized.
    }
    catch(cv_bridge::Exception& e){
        ROS_ERROR("cv_bridge exception: %s", e.what());
        return;
    }
    cv::resize(tmp, (*rgbMat), (*rgbMat).size(), 0, 0, cv::INTER_AREA);
}

int main(int argc, char** argv){
    
    ros::init(argc, argv, "spy_client");
    if (argc != 1){
        ROS_INFO("usage: spy_client");
        return -1;
    }
    ros::NodeHandle nh;
    cv::Mat rgbcus(Size(640, 480), CV_8UC3);
    rgbData rD(rgbcus, nh);
    cv::namedWindow("rgb", CV_WINDOW_AUTOSIZE);
    while(ros::ok()){
        cv::imshow("rgb", rgbcus);
        if(cv::waitKey(40) == 27)
            break;
        ros::spinOnce();
    }
    return 0;
}