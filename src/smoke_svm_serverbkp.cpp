#include <string.h>
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
#include <boost/function.hpp>

ros::ServiceClient sc;
smoke::darknet_svm_node svm_nn_srv;
lbp::LBPSVM SVM(0, 0, "", "");

bool svmcallback(smoke::smoke_svm::Request &req, smoke::smoke_svm::Response &response, lbp::LBPSVM SVM){
    sensor_msgs::Image msg = req.hst;
    int bsize = req.dim;
    cv::Mat hist = cv_bridge::toCvCopy(msg, sensor_msgs::image_encodings::TYPE_8UC1)->image;
    std::vector<int> pred = SVM.Predb(hist);
    response.res = 0;
    for(int i = 0; i < pred.size(); ++i){
        if(pred[i] == 1)  response.res = 1;
    }
    return true;
}

bool darknetsvmcallback(smoke::darknet_svm_node::Request &req, 
                        smoke::darknet_svm_node::Response &res)
{
    cv::Mat img, obj, tmp, dst, img_gray;
    sensor_msgs::Image imgs = req.img;
    std::vector<smoke::BoundingBox> bounding_boxes = req.bboxes.bounding_boxes, bounding_boxes_u;
    std::vector<int> bbox_indexes;
    img = cv_bridge::toCvCopy(imgs, sensor_msgs::image_encodings::BGR8)->image.clone();
    cv::cvtColor(img, img_gray, cv::COLOR_BGR2GRAY);

    // ROS_INFO("[smoke_svm_server] Using saved SVM model: %s; EigenMat: %s", svm_model, eigen_mat.c_str());
    std::vector<int> resp(bounding_boxes.size(), 0);
    for(int i = 0; i < bounding_boxes.size(); ++i){
        int xmin = bounding_boxes[i].xmin;
        int ymin = bounding_boxes[i].ymin;
        int width = bounding_boxes[i].xmax - bounding_boxes[i].xmin + 1;
        int height = bounding_boxes[i].ymax - bounding_boxes[i].ymin + 1;
        obj = img_gray(cv::Rect(xmin, ymin, width, height));
        try{
            cv::resize(obj, tmp, Size(100, 100));
        }
        catch(cv::Exception &e){
            ROS_ERROR("[smoke_svm_server] Size of image: (%d, %d) .%s", obj.rows, obj.cols, e.what());
        }
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
    int cellsize, radius;
    std::string eigen_mat;
    nh.param("/smoke/services/image_svm_srv/name", imgSVMSrvServer, std::string("/kinectdev/smoke/smoke_svm_srv"));
    nh.param("/smoke/services/image_darknet_svm_srv/name", imgDarknetSVMSrvServer, std::string("/kinectdev/smoke/smoke_darknet_svm_srv"));
    nh.param("/smoke/svm/lbp_params/cellsize", cellsize, 25);
    nh.param("/smoke/svm/lbp_params/radius", radius, 2);
    std::string svmnnSrv, modelpath;
    nh.param("/smoke/services/img_bbox_sub/img_svm_nn_srv/name", svmnnSrv, std::string("/kinectdev/smoke/smoke_svm_nn_srv"));
    nh.param("/smoke/svm/modelpath", modelpath, std::string("./../models/svm/model_"));
    nh.param("/smoke/svm/eigen/eigenmat", eigen_mat, std::string("/home/fran/ROS/catkin_ws/src/smoke/data/eigenMat.yml"));
    SVM = lbp::LBPSVM(cellsize, radius, eigen_mat.c_str(), modelpath.c_str());
    SVM.getEigen();
    // ros::ServiceServer service = nh.advertiseService(imgSVMSrvServer, svmcallback);
    sc = nh.serviceClient<smoke::darknet_svm_node>(svmnnSrv);
    ros::ServiceServer ss = nh.advertiseService<smoke::darknet_svm_node::Request, smoke::darknet_svm_node::Response>
            (imgDarknetSVMSrvServer, darknetsvmcallback);
    ros::spin();
    return 0;
}
