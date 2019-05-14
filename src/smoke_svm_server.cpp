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

ros::ServiceClient sc;
smoke::darknet_svm_node svm_nn_srv;

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
    cv::Mat img, obj, tmp, dst, img_gray;
    lbp::LBPSVM SVM(25, 3);
    sensor_msgs::Image imgs = req.img;
    std::vector<smoke::BoundingBox> bounding_boxes = req.bboxes.bounding_boxes, bounding_boxes_u;
    std::vector<int> bbox_indexes;
    img = cv_bridge::toCvCopy(imgs, sensor_msgs::image_encodings::TYPE_8UC1)->image.clone();
    cv::cvtColor(img, img_gray, cv::COLOR_BGR2GRAY);

    std::vector<int> resp(bounding_boxes.size(), 0);
    for(int i = 0; i < bounding_boxes.size(); ++i){
        int xmin = bounding_boxes[i].xmin;
        int ymin = bounding_boxes[i].ymin;
        int width = bounding_boxes[i].xmax - bounding_boxes[i].xmin + 1;
        int height = bounding_boxes[i].ymax - bounding_boxes[i].ymin + 1;
        obj = img_gray(cv::Rect(xmin, ymin, width, height));
        cv::resize(obj, tmp, Size(100, 100));
        float response = SVM.Predict(tmp);
        resp.push_back(static_cast<int>(response));
    }

    // using nn classifier
    for(int i = 0; i < resp.size(); ++i){
        if(i == 1)  bounding_boxes_u.push_back(bounding_boxes[i]);
        bbox_indexes.push_back(i);
    }

    cv_bridge::CvImage(std_msgs::Header(), sensor_msgs::image_encodings::BGR8, img).toImageMsg(svm_nn_srv.request.img);
    svm_nn_srv.request.bboxes.bounding_boxes = bounding_boxes_u;
    if(sc.call(svm_nn_srv)){
        std::vector<int> nnpos = svm_nn_srv.response.res;
        for(int i = 0; i < nnpos.size(); ++i){
            resp[bbox_indexes[i]] = nnpos[i];
        }
    }
    res.res = resp;
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
    nh.param("/smoke/services/image_svm_srv/name", imgSVMSrvServer, std::string("/kinectdev/smoke/smoke_svm_srv"));
    nh.param("/smoke/services/image_darknet_svm_srv/name", imgSVMSrvServer, std::string("/kinectdev/smoke/smoke_svm_srv"));
    ros::ServiceServer service = nh.advertiseService(imgSVMSrvServer, svmcallback);
    ros::ServiceServer ss = nh.advertiseService(imgDarknetSVMSrvServer, darknetsvmcallback);

    std::string svmnnSrv;
    nh.param("/smoke/services/img_bbox_sub/img_svm_nn_srv/name", svmnnSrv, std::string("/kinectdev/smoke/smoke_svm_nn_srv"));
    sc = nh.serviceClient<smoke::darknet_svm_node>(svmnnSrv);
    ros::spin();
    return 0;
}