#include <vector>
#include <string>
#include "smoke/BoundingBox.h"
#include "smoke/BoundingBoxes.h"
#include "smoke/BboxImage.h"
#include "smoke/darknet_svm_node.h"
#include <cv_bridge/cv_bridge.h>
#include "image_transport/image_transport.h"
#include "opencv2/opencv.hpp"
#include "opencv2/highgui.hpp"
#include "opencv2/imgproc.hpp"

smoke::BoundingBoxes bboxes;
std::vector<smoke::BoundingBox> bounding_boxes;
sensor_msgs::Image imgs;
cv_bridge::CvImagePtr img_bridge_sub;

bool srvCallback(smoke::darknet_svm_node::Request &req, smoke::darknet_svm_node::Response &res){
    ROS_INFO("[service_test_node] service callback.");
    
    cv::Mat img, obj, tmp, dst, img_gray;
    sensor_msgs::Image imgs = req.img;
    std::vector<smoke::BoundingBox> bounding_boxes = req.bboxes.bounding_boxes;
    std::vector<int> bbox_indexes;
    img = cv_bridge::toCvCopy(imgs, sensor_msgs::image_encodings::BGR8)->image.clone();
    ROS_INFO("[service_test_node] Received %lu bounding boxes", bounding_boxes.size());
    for(int i = 0; i < bounding_boxes.size(); ++i){
        int xmin = bounding_boxes[i].xmin;
        int xmax = bounding_boxes[i].xmax;
        int ymin = bounding_boxes[i].ymin;
        int ymax = bounding_boxes[i].ymax;
        double prob = bounding_boxes[i].probability;
        ROS_INFO("[service_test_node] Bounding Box %d: %s %.2lf (%d, %d) -> (%d, %d)", i, bounding_boxes[i].Class.c_str(), prob, xmin, ymin, xmax, ymax);
    }
    std::vector<int> resp(bounding_boxes.size(), 0);
    res.res = resp;
    return true;
}

int main(int argc, char **argv){
    ros::init(argc, argv, "bbox_test");
    ros::NodeHandle nh;
    ros::Rate rate(5.);
    ros::ServiceServer ss = nh.advertiseService("/kinectdev/smoke/smoke_darknet_svm_srv", srvCallback);
    ROS_INFO("[service_test_node] Node Started.");
    while(ros::ok()){
        rate.sleep();
        ros::spinOnce();
    }
    return 0;
}