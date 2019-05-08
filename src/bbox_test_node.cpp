#include <vector>
#include <string>
#include "smoke/BoundingBox.h"
#include "smoke/BoundingBoxes.h"
#include "smoke/BboxImage.h"
#include <cv_bridge/cv_bridge.h>
#include "image_transport/image_transport.h"
#include "opencv2/opencv.hpp"
#include "opencv2/highgui.hpp"
#include "opencv2/imgproc.hpp"

smoke::BoundingBoxes bboxes;
std::vector<smoke::BoundingBox> bounding_boxes;
sensor_msgs::Image imgs;
cv_bridge::CvImagePtr img_bridge_sub;

void bboxImgCallback(const smoke::BboxImageConstPtr& msg){
    ROS_INFO("[bbox_test_node] Subscriber (img_bbox_sub) callback.");
    bboxes = (*msg).bboxes;
    bounding_boxes = bboxes.bounding_boxes;
    imgs = (*msg).img;
    try{
        img_bridge_sub = cv_bridge::toCvCopy(imgs, sensor_msgs::image_encodings::BGR8);
        //img_bridge_sub = cv_bridge::toCvShare((*msg).img, sensor_msgs::image_encodings::BGR8);
    }
    catch(cv_bridge::Exception& e){
        ROS_ERROR("[bbox_test_node] cv_bridge exception: %s", e.what());
        return;
    }
    if(img_bridge_sub){
        ROS_INFO("[bbox_test_node] Received image along with %lu bounding boxes.", bounding_boxes.size());
    }
    else{
        ROS_ERROR("[bbox_test_node] Warning: Invalid image!");
    }
}

void bboxCallback(const smoke::BoundingBoxesConstPtr& boxes){
    ROS_INFO("[bbox_test_node] Subscriber (img_bbox_sub) callback.");
    bounding_boxes = (*boxes).bounding_boxes;
    ROS_INFO("[bbox_test_node] Received %lu bounding boxes.", bounding_boxes.size());
    for(int i = 0; i < bounding_boxes.size(); ++i){
        ROS_INFO("[bbox_test_node][bbox %d]: %s (%ld,%ld)->(%ld,%ld)", 
            i+1, bounding_boxes[i].Class.c_str(), bounding_boxes[i].xmin, bounding_boxes[i].ymin, bounding_boxes[i].xmax, bounding_boxes[i].ymax);
    }
}

int main(int argc, char **argv){
    ros::init(argc, argv, "bbox_test");
    ros::NodeHandle nh;
    ros::Rate rate(5.);
    //ros::Subscriber bboximgsub= 
    //       nh.subscribe<smoke::BoundingBoxes>("/darknet_ros/bounding_boxes", 1, bboxCallback);
    ros::Subscriber bboximgsub= 
           nh.subscribe<smoke::BboxImage>("/darknet_ros/image_bounding_boxes", 1, bboxImgCallback);
    ROS_INFO("[bbox_test_node] Node Started.");
    while(ros::ok()){
        rate.sleep();
        ros::spinOnce();
    }
    return 0;
}