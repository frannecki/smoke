#include "darknetDetector.h"

namespace darknet_svm{
darknet_svm::darknet_svm(ros::NodeHandle nh): nh_(nh){
    init();
};

darknet_svm::~darknet_svm(){
    {
        boost::unique_lock<boost::shared_mutex> lockNodeStatus(mutexNodeStatus);
        isNodeRunning = false;
    }
    pthread_join(mainThread, NULL);
}

void darknet_svm::init(){
    std::string bboxImgSub;
    int bboxImgSub_qs;

    std::string alarmPub;
    int alarmPub_qs;
    bool alarmPub_latch;

    std::string darknetSVMSrv;

    nh_.param("subscribers/img_bbox_sub/topic", bboxImgSub, std::string("/darknet_ros/image_bounding_boxes"));
    nh_.param("subscribers/img_bbox_sub/queue_size", bboxImgSub_qs, 1);

    nh_.param("publishers/img_sub/topic", alarmPub, std::string("/kinectdev/smoke/alarm"));
    nh_.param("publishers/img_sub/queue_size", alarmPub_qs, 1);
    nh_.param("publishers/img_bbox_sub/latch", alarmPub_latch, false);
    alarm_pub = nh_.advertise<std_msgs::Bool>(alarmPub, alarmPub_qs, alarmPub_latch);

    nh_.param("services/img_bbox_sub/img_darknet_svm_srv/name", darknetSVMSrv, std::string("/kinectdev/smoke/smoke_darknet_svm_srv"));
    sc = nh_.serviceClient<smoke::darknet_svm_node>(darknetSVMSrv);

    count = 0;
    alarm.data = false;

    img_bbox_sub = nh_.subscribe<smoke::BboxImage>(bboxImgSub, bboxImgSub_qs, &darknet_svm::bboxImgCallback, this);
    int mainThreadStatus = pthread_create(&mainThread, NULL, workInThread, NULL);
}

void darknet_svm::bboxImgCallback(const smoke::BboxImageConstPtr& msg){
    bboxes = (*msg).bboxes;
/*
    if(!imgs){
        boost::unique_lock<boost::shared_mutex> lockImgCallback(mutexImgsStatus);
        subscriberStatus = false;
    }
    else{
        boost::unique_lock<boost::shared_mutex> lockImgCallback(mutexImgsStatus);
        subscriberStatus = false;
    }
*/
    bounding_boxes = bboxes.bounding_boxes;
    imgs = (*msg).img;
    try{
        img = cv_bridge::toCvCopy(imgs, sensor_msgs::image_encodings::TYPE_8UC3)->image;
    }
    catch(cv_bridge::Exception& e){
        ROS_ERROR("cv_bridge exception: %s", e.what());
        return;
    }
    cv::cvtColor(img, img, cv::COLOR_BGR2GRAY);
    boost::unique_lock<boost::shared_mutex> lockImgCallback(mutexImgsStatus);
    for(int i = 0; i < bounding_boxes.size(); ++i){
        int xmin = bounding_boxes[i].xmin;
        int ymin = bounding_boxes[i].ymin;
        int width = bounding_boxes[i].xmax - bounding_boxes[i].xmin;
        int height = bounding_boxes[i].ymax - bounding_boxes[i].ymin;
        obj = img(cv::Rect(xmin, ymin, width, height));
        header.seq = ++count;   // user defined counter
        header.stamp = ros::Time::now();   // time
        img_bridge = cv_bridge::CvImage(header, sensor_msgs::image_encodings::TYPE_8UC1, obj);
        img_bridge.toImageMsg(imgs);
        imgsVec.push_back(imgs);
    }
}

void *darknet_svm::callSVMInThread(void* param){
    darknet_svm *ds = (darknet_svm*)param;
    {
        boost::shared_lock<boost::shared_mutex> lockCallSVM(ds->mutexImgsStatus);
        ds->svm_srv.request.imgs = std::vector<sensor_msgs::Image>(ds->imgsVec);
    }
    if(ds->sc.call(ds->svm_srv)){
        int res = ds->svm_srv.response.res;
        boost::unique_lock<boost::shared_mutex> lockAlarm(ds->mutexAlarmStatus);
        if(res == 1){
            ds->alarm.data = true;
        }
        else{
            ds->alarm.data = false;
        }
    }
    else{
        ROS_ERROR("Failed to call service smoke");
        ROS_INFO("Waiting for service server to correctly respond...");
        sleep(1);
    }
}

void *darknet_svm::alarmInThread(void* param){
    darknet_svm *ds = (darknet_svm*)param;
    {
        boost::shared_lock<boost::shared_mutex> lockAlarm(ds->mutexAlarmStatus);
        ds->alarm_pub.publish(ds->alarm);
    }
}

void *darknet_svm::workInThread(void* param){
    darknet_svm *ds = (darknet_svm*)param;
    pthread_t callSVMThread;
    pthread_t alarmThread;

    while(ros::ok()){
        if(!ds->isNodeRunning)  break;
        if(!ds->subscriberStatus)  continue;
        int callSVMThreadStatus = pthread_create(&callSVMThread, NULL, callSVMInThread, NULL);
        if(ds->callSVMThreadStatus != 0){
            ROS_ERROR("Failed to start callSVMThread");
            exit(-1);
        }
        
        int alarmThreadStatus = pthread_create(&alarmThread, NULL, alarmInThread, NULL);
        if(alarmThreadStatus != 0){
            ROS_ERROR("Failed to start alarmThread");
            exit(-1);
        }

        pthread_join(callSVMThread, NULL);
        pthread_join(alarmThread, NULL);
    }
}
}