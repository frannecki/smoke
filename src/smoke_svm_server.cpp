#include "ros/ros.h"
#include "LBP.h"
#include "smoke/smoke_svm.h"
#include "smoke/darknet_svm_node.h"
#include "cv_bridge/cv_bridge.h"
#include "image_transport/image_transport.h"
#include "opencv2/opencv.hpp"
#include "opencv2/highgui.hpp"
#include "opencv2/imgproc.hpp"
#include "smoke/BboxImage.h"
bool svmcallback(smoke::smoke_svm::Request &req, smoke::smoke_svm::Response &response){
    sensor_msgs::Image msg = req.hst;
    int bsize = req.dim;
    cv::Mat hist = cv_bridge::toCvCopy(msg, sensor_msgs::image_encodings::TYPE_8UC1)->image;
    lbp::LBPSVM SVM(25, 3);
    std::vector<int> pred = SVM.Predb(hist);
    response.res = 0;
    for(int i = 0; i < pred.size(); ++i){
        if(pred[i] == 1)  response.res = 1;
    }
    return true;
}

bool darknetsvmcallback(smoke::darknet_svm_node::Request &req, smoke::darknet_svm_node::Response &res){
    cv::Mat img, obj, tmp, dst;
    lbp::LBPSVM SVM(25, 3);
    sensor_msgs::Image imgs = req.img;
    std::vector<smoke::BoundingBox> bounding_boxes = req.bboxes.bounding_boxes;
    img = cv_bridge::toCvCopy(imgs, sensor_msgs::image_encodings::TYPE_8UC1)->image.clone();

    std::vector<int> resp(bounding_boxes.size(), 0);
    for(int i = 0; i < bounding_boxes.size(); ++i){
        int xmin = bounding_boxes[i].xmin;
        int ymin = bounding_boxes[i].ymin;
        int width = bounding_boxes[i].xmax - bounding_boxes[i].xmin;
        int height = bounding_boxes[i].ymax - bounding_boxes[i].ymin;
        obj = img(cv::Rect(xmin, ymin, width, height));
        cv::resize(obj, tmp, Size(100, 100));
        float response = SVM.Predict(tmp);
        resp.push_back(static_cast<int>(response));
    }
    
    res.res = resp;
    /*
    int countpos = 0;
    for(int i = 0; i < resp.size(); ++i){
        countpos += resp[i];
    }
    if(countpos >= resp.size()/5){
        res.res = 1;
    }
    */
    return true;
}

int main(int argc, char **argv){
    ros::init(argc, argv, "smoke_svm_server");
    if (argc != 1){
        ROS_INFO("usage: smoke_svm_server");
        return -1;
    }
    ros::NodeHandle nh;
    std::string imgSVMSrvServer, imgDarknetSVMSrvServer;
    nh.param("services/image_svm_srv/name", imgSVMSrvServer, std::string("/kinectdev/smoke/smoke_svm_srv"));
    nh.param("services/image_darknet_svm_srv/name", imgSVMSrvServer, std::string("/kinectdev/smoke/smoke_svm_srv"));
    ros::ServiceServer service = nh.advertiseService(imgSVMSrvServer, svmcallback);
    ros::ServiceServer ss = nh.advertiseService(imgDarknetSVMSrvServer, darknetsvmcallback);
    ros::spin();
    return 0;
}