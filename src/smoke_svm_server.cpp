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
                        smoke::darknet_svm_node::Response &res, lbp::LBPSVM *SVM,
                        ros::ServiceClient *sc, smoke::darknet_svm_node *svm_nn_srv)
{
    cv::Mat img, obj, tmp, dst, img_gray;
    sensor_msgs::Image imgs = req.img;
    std::vector<smoke::BoundingBox> bounding_boxes = req.bboxes.bounding_boxes, bounding_boxes_u;
    std::vector<int> bbox_indexes;
    img = cv_bridge::toCvCopy(imgs, sensor_msgs::image_encodings::BGR8)->image.clone();
    cv::cvtColor(img, img_gray, cv::COLOR_BGR2GRAY);

    // ROS_INFO("[smoke_svm_server] Using saved SVM model: %s; EigenMat: %s", svm_model, eigen_mat.c_str());
    ROS_INFO("[smoke_svm_server] Received %lu bounding boxes", bounding_boxes.size());
    std::vector<int> resp(bounding_boxes.size(), 0);
    for(int i = 0; i < bounding_boxes.size(); ++i){
        int xmin = bounding_boxes[i].xmin;
        int ymin = bounding_boxes[i].ymin;
        int width = bounding_boxes[i].xmax - bounding_boxes[i].xmin + 1;
        int height = bounding_boxes[i].ymax - bounding_boxes[i].ymin + 1;
        // printf("%d %d\n", width, height);
        try{
            obj = img_gray(cv::Rect(xmin, ymin, width-1, height-1));
            cv::resize(obj, tmp, Size(100, 100));
        }
        catch(cv::Exception &e){
            ROS_ERROR("[smoke_svm_server] (xmax, ymax): (%d, %d) .%s", ymin+height, xmin+width, e.what());
        }
        float response = SVM->Predict(tmp);
        resp[i] = static_cast<int>(response);

        if(resp[i] == 1){
            bounding_boxes_u.push_back(bounding_boxes[i]);
            bbox_indexes.push_back(i);
        }
    }

    // using nn classifier
    if(bbox_indexes.size() > 0){
        cv_bridge::CvImage(std_msgs::Header(), sensor_msgs::image_encodings::BGR8, img).toImageMsg(svm_nn_srv->request.img);
        svm_nn_srv->request.bboxes.bounding_boxes = bounding_boxes_u;
        ROS_INFO("[smoke_svm_server] Detected %lu suspicious box(es).", bbox_indexes.size());
        if(sc->call(*svm_nn_srv)){
            bool flag = false;
            ROS_INFO("[smoke_svm_server] Succeeded in calling nn server.");
            std::vector<int> nnpos = svm_nn_srv->response.res;
            for(int i = 0; i < nnpos.size(); ++i){
                resp[bbox_indexes[i]] = nnpos[i];
                if(nnpos[i] == 1)
                    flag = true;
            }
            if(flag){
                ROS_INFO("[smoke_svm_server][Warning] Detected bounding box with smoke.\n");
            }
        }
        else{
            ROS_ERROR("[smoke_svm_server] Failed to call nn server!");
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
    ros::ServiceClient sc;
    smoke::darknet_svm_node svm_nn_srv;
    std::string imgSVMSrvServer, imgDarknetSVMSrvServer;
    int cellsize, radius;
    nh.param("/smoke/services/image_svm_srv/name", imgSVMSrvServer, std::string("/kinectdev/smoke/smoke_svm_srv"));
    nh.param("/smoke/services/image_darknet_svm_srv/name", imgDarknetSVMSrvServer, std::string("/kinectdev/smoke/smoke_darknet_svm_srv"));
    nh.param("/smoke/svm/lbp_params/cellsize", cellsize, 25);
    nh.param("/smoke/svm/lbp_params/radius", radius, 2);
    std::string svmnnSrv, modelpath, eigen_mat;
    nh.param("/smoke/services/img_bbox_sub/img_svm_nn_srv/name", svmnnSrv, std::string("/kinectdev/smoke/smoke_svm_nn_srv"));
    nh.param("/smoke/svm/modelpath", modelpath, std::string("/home/fran/ROS/catkin_ws/src/smoke/models/svm/model_"));
    nh.param("/smoke/svm/eigen/eigenmat", eigen_mat, std::string("/home/fran/ROS/catkin_ws/src/smoke/data/eigenMat.yml"));
    lbp::LBPSVM SVM(cellsize, radius, eigen_mat.c_str(), modelpath.c_str());
    SVM.getEigen();
    // ros::ServiceServer service = nh.advertiseService(imgSVMSrvServer, svmcallback);
    sc = nh.serviceClient<smoke::darknet_svm_node>(svmnnSrv);
    ros::ServiceServer ss = nh.advertiseService<smoke::darknet_svm_node::Request, smoke::darknet_svm_node::Response>
            (imgDarknetSVMSrvServer, boost::bind(&darknetsvmcallback, _1, _2, &SVM, &sc, &svm_nn_srv));
    while(ros::ok()){
        ros::spinOnce();
    }
    return 0;
}
