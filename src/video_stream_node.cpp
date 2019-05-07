#include <ros/ros.h>
#include <string>
#include <vector>
#include <opencv2/opencv.hpp>
#include <opencv2/highgui.hpp>
#include <opencv2/imgproc.hpp>
#include <cv_bridge/cv_bridge.h>
#include <std_msgs/Header.h>
#include <image_transport/image_transport.h>
#include "smoke/BoundingBoxes.h"
#include "smoke/BboxImage.h"


int main(int argc, char **argv){
    ROS_INFO("[video_stream_node] Started Node (video_stream_node).");
    ros::init(argc, argv, "video_stream");
    ros::NodeHandle nh;
    ros::Rate rate(20);
    image_transport::ImageTransport it(nh);
    //image_transport::Publisher pub = it.advertise("/video/image_raw", 1);
    image_transport::Publisher pub = it.advertise("/usb_cam/image_raw", 1);
    ros::Publisher bboxpub = nh.advertise<smoke::BboxImage>("/darknet_ros/image_bounding_boxes", 1, true);
    //const char* filepath = "~/Videos/vtest.avi";
    std::string fname = std::string("/home/fran/Videos/vtest.avi");
    cv::VideoCapture vicap(fname);
    cv::Mat frame;
    cv_bridge::CvImage img_bridge;
    std_msgs::Header header;
    sensor_msgs::ImagePtr msg;
    int count = 0;

    smoke::BoundingBoxes bbox;
    smoke::BboxImage img_bbox;
    smoke::BoundingBox box;
    std::vector<smoke::BoundingBox> bboxvec;
    sensor_msgs::Image imgs;
    //cv::namedWindow("demo", cv::WINDOW_AUTOSIZE);
    while(ros::ok()){
        vicap >> frame;
        header.seq = ++count;
        header.stamp = ros::Time::now();
        img_bridge = cv_bridge::CvImage(header, "bgr8", frame);
        msg = img_bridge.toImageMsg();
        pub.publish(msg);
        //usleep(40000);
        box.Class = std::string("random");
        box.probability = .5;
        box.xmin = 0;  box.ymin = 0;  box.xmax = frame.cols / 2;  box.ymax = frame.rows / 2;
        bboxvec.push_back(box);
        bbox.bounding_boxes = bboxvec;
        img_bbox.bboxes = bbox;
        img_bbox.img = *msg;
        bboxpub.publish(img_bbox);
        bboxvec.clear();

        rate.sleep();
        //cv::imshow("demo", frame);
        //if(cv::waitKey(40) == 27)  break;
    }
    //cv::destroyAllWindows();
    return 0;
}