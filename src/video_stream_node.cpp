#include <ros/ros.h>
#include <opencv2/opencv.hpp>
#include <opencv2/highgui.hpp>
#include <opencv2/imgproc.hpp>
#include <cv_bridge/cv_bridge.h>
#include <std_msgs/Header.h>
#include <image_transport/image_transport.h>
#include <string>


int main(int argc, char **argv){
    ROS_INFO("[video_stream_node] Started Node (video_stream_node).");
    ros::init(argc, argv, "video_stream");
    ros::NodeHandle nh;
    ros::Rate rate(20);
    image_transport::ImageTransport it(nh);
    //image_transport::Publisher pub = it.advertise("/video/image_raw", 1);
    image_transport::Publisher pub = it.advertise("/usb_cam/image_raw", 1);
    //const char* filepath = "~/Videos/vtest.avi";
    std::string fname = std::string("/home/fran/Videos/vtest.avi");
    cv::VideoCapture vicap(fname);
    cv::Mat frame;
    cv_bridge::CvImage img_bridge;
    std_msgs::Header header;
    sensor_msgs::ImagePtr msg;
    int count = 0;
    //cv::namedWindow("demo", cv::WINDOW_AUTOSIZE);
    while(ros::ok()){
        vicap >> frame;
        header.seq = ++count;
        header.stamp = ros::Time::now();
        img_bridge = cv_bridge::CvImage(header, "bgr8", frame);
        msg = img_bridge.toImageMsg();
        pub.publish(msg);
        //usleep(40000);
        rate.sleep();
        //cv::imshow("demo", frame);
        //if(cv::waitKey(40) == 27)  break;
    }
    //cv::destroyAllWindows();
    return 0;
}