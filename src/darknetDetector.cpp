#include "darknetDetector.h"
#include <iostream>

namespace darknet_svm{
bool cmp_by_prob(const smoke::BoundingBox& bbox1, const smoke::BoundingBox& bbox2){
    return bbox1.probability > bbox2.probability;
}

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
    ROS_INFO("Started Node darknet_detector");
    std::string bboxImgSub;
    int bboxImgSub_qs;

    std::string alarmPub;
    int alarmPub_qs;
    bool alarmPub_latch;

    std::string darknetSVMSrv;

    nh_.param("/smoke/subscribers/img_bbox_sub/topic", bboxImgSub, std::string("/darknet_ros/image_bounding_boxes"));
    nh_.param("/smoke/subscribers/img_bbox_sub/queue_size", bboxImgSub_qs, 1);

    nh_.param("/smoke/publishers/alarm_pub/topic", alarmPub, std::string("/kinectdev/smoke/alarm"));
    nh_.param("/smoke/publishers/alarm_pub/queue_size", alarmPub_qs, 1);
    nh_.param("/smoke/publishers/alarm_pub/latch", alarmPub_latch, false);
    alarm_pub = nh_.advertise<std_msgs::Bool>(alarmPub, alarmPub_qs, alarmPub_latch);

    nh_.param("/smoke/services/img_bbox_sub/img_darknet_svm_srv/name", darknetSVMSrv, std::string("/kinectdev/smoke/smoke_darknet_svm_srv"));
    sc = nh_.serviceClient<smoke::darknet_svm_node>(darknetSVMSrv);

    count = 0;
    alarm.data = false;
    isNodeRunning = true;
    ovthresh = .7;
    subscriberStatus = false;

    img_bbox_sub = nh_.subscribe<smoke::BboxImage>(bboxImgSub, bboxImgSub_qs, &darknet_svm::bboxImgCallback, this);
    int mainThreadStatus = pthread_create(&mainThread, NULL, workInThread, (void*)this);
}

void darknet_svm::bboxImgCallback(const smoke::BboxImageConstPtr& msg){
    bboxes = (*msg).bboxes;
    bounding_boxes = bboxes.bounding_boxes;
    // start to modify shared variables imgs, image_bridge, img_srv and bounding_boxes_u
    boost::unique_lock<boost::shared_mutex> lockImgCallback(mutexImgsStatus);
    imgs = (*msg).img;
    try{
        img_bridge_sub = cv_bridge::toCvCopy(imgs, sensor_msgs::image_encodings::TYPE_8UC3);
    }
    catch(cv_bridge::Exception& e){
        ROS_ERROR("cv_bridge exception: %s", e.what());
        return;
    }
    if(img_bridge_sub){
        boost::unique_lock<boost::shared_mutex> lockSubscriber(mutexSubscriberStatus);
        subscriberStatus = true;
    }
    else{
        boost::unique_lock<boost::shared_mutex> lockSubscriber(mutexSubscriberStatus);
        subscriberStatus = false;
        sleep(1);
        return;
    }
    img = img_bridge_sub->image.clone();
    cv::cvtColor(img, img, cv::COLOR_BGR2GRAY);
    bounding_boxes_u.clear();

    header.seq = ++count;   // user defined counter
    header.stamp = ros::Time::now();   // time
    img_bridge = cv_bridge::CvImage(header, sensor_msgs::image_encodings::TYPE_8UC1, img);
    img_bridge.toImageMsg(img_srv);
    // selecting bounding boxes
    std::vector<int> overlap(bounding_boxes.size(), 0);
    std::sort(bounding_boxes.begin(), bounding_boxes.end(), cmp_by_prob);
    for(int i = 0; i < bounding_boxes.size(); ++i){
        if(overlap[i] != 0){
            bounding_boxes_u.push_back(bounding_boxes[i]);
            continue;
        }
        for(int j = i+1; j < bounding_boxes.size(); ++j){
            // rid of redundant bboxes
            int xmin_1 = bounding_boxes[i].xmin;
            int xmax_1 = bounding_boxes[i].xmax;
            int ymin_1 = bounding_boxes[i].ymin;
            int ymax_1 = bounding_boxes[i].ymax;
            int xmin_2 = bounding_boxes[j].xmin;
            int xmax_2 = bounding_boxes[j].xmax;
            int ymin_2 = bounding_boxes[j].ymin;
            int ymax_2 = bounding_boxes[j].ymax;
            int x_min = std::max(xmin_1, xmin_2);
            int y_min = std::max(ymin_1, ymin_2);
            int x_max = std::min(xmax_1, xmax_2);
            int y_max = std::min(ymax_1, ymax_2);
            float intersec = static_cast<float>((x_max-x_min)*(y_max-y_min));
            float uni = static_cast<float>((xmax_1-xmin_1)*(ymax_1-ymin_1) + (xmax_1-xmin_1)*(ymax_1-ymin_1)) - intersec;
            float ovlap = intersec / uni;
            if(ovlap > ovthresh){
                overlap[j] = 1;
            }
        }
    }
    sleep(1);
}

void *darknet_svm::callSVMInThread(void* param){
    darknet_svm *ds = (darknet_svm*)param;
    {
        boost::shared_lock<boost::shared_mutex> lockCallSVM(ds->mutexImgsStatus);
        ds->svm_srv.request.img = ds->img_srv;
        ds->svm_srv.request.bboxes.bounding_boxes = ds->bounding_boxes_u;
    }
    if(ds->sc.call(ds->svm_srv)){
        //int res = ds->svm_srv.response.res;
        ds->bounding_box_u_response = ds->svm_srv.response.res;
        
        boost::unique_lock<boost::shared_mutex> lockAlarm(ds->mutexAlarmStatus);
        int res = 0;
        for(int i = 0; i < ds->bounding_box_u_response.size(); ++i){
            res += ds->bounding_box_u_response[i];
        }
        if(res >= 1){ ds->alarm.data = true; }
        else{ ds->alarm.data = false; }
    }
    else{
        ds->alarm.data = false;
        ROS_ERROR("Failed to call service node (svm classifier)");
        ROS_INFO("Waiting for service server to normally respond...");
    }
    sleep(0.5);
}

void *darknet_svm::alarmInThread(void* param){
    ROS_INFO("Alarm! Smoke occurs.");
    darknet_svm *ds = (darknet_svm*)param;
    
    boost::shared_lock<boost::shared_mutex> lockAlarm(ds->mutexAlarmStatus);
    ds->alarm_pub.publish(ds->alarm);
    if(ds->alarm.data == true){
        int count = 0;
        for(int i = 0; i < ds->bounding_box_u_response.size(); ++i){
            if(ds->bounding_box_u_response[i] == 1){
                ROS_INFO("Position %d: (%ld, %ld) -> (%ld, %ld)", ++count, 
                    ds->bounding_boxes_u[i].xmin, ds->bounding_boxes_u[i].ymin, ds->bounding_boxes_u[i].xmax, ds->bounding_boxes_u[i].ymax);
            }
        }
    }
}

void *darknet_svm::workInThread(void* param){
    darknet_svm *ds = (darknet_svm*)param;
    pthread_t callSVMThread;
    pthread_t alarmThread;
    void* voidPtr;
    int callSVMThreadStatus, alarmThreadStatus;
    if(!(ds->subscriberStatus)){ sleep(0.5); }

    while(ros::ok()){
        {
            boost::shared_lock<boost::shared_mutex> lockNodeStatus(ds->mutexNodeStatus);
            if(!ds->isNodeRunning)  break;
        }
        {
            boost::shared_lock<boost::shared_mutex> lockSubscriber(ds->mutexSubscriberStatus);
            if(!ds->subscriberStatus){ sleep(0.5); continue; }
        }
        callSVMThreadStatus = pthread_create(&callSVMThread, NULL, callSVMInThread, param);
        if(callSVMThreadStatus != 0){
            ROS_ERROR("Failed to start thread (callSVMThread).");
            return voidPtr;
        }
        
        alarmThreadStatus = pthread_create(&alarmThread, NULL, alarmInThread, param);
        if(alarmThreadStatus != 0){
            ROS_ERROR("Failed to start thread (alarmThread).");
            return voidPtr;
        }

        pthread_join(callSVMThread, NULL);
        pthread_join(alarmThread, NULL);
    }
}

bool darknet_svm::getNodeStatus(){
    boost::shared_lock<boost::shared_mutex> lockNodeStatus(mutexNodeStatus);
    return isNodeRunning;
}

bool darknet_svm::getSubscriberStatus(){
    boost::shared_lock<boost::shared_mutex> lockSubscriber(mutexSubscriberStatus);
    return subscriberStatus;
}
}