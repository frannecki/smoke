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

    nh_.param("publishers/alarm_pub/topic", alarmPub, std::string("/kinectdev/smoke/alarm"));
    nh_.param("publishers/alarm_pub/queue_size", alarmPub_qs, 1);
    nh_.param("publishers/alarm_pub/latch", alarmPub_latch, false);
    alarm_pub = nh_.advertise<std_msgs::Bool>(alarmPub, alarmPub_qs, alarmPub_latch);

    nh_.param("services/img_bbox_sub/img_darknet_svm_srv/name", darknetSVMSrv, std::string("/kinectdev/smoke/smoke_darknet_svm_srv"));
    sc = nh_.serviceClient<smoke::darknet_svm_node>(darknetSVMSrv);

    count = 0;
    alarm.data = false;
    ovthresh = .7;

    img_bbox_sub = nh_.subscribe<smoke::BboxImage>(bboxImgSub, bboxImgSub_qs, &darknet_svm::bboxImgCallback, this);
    int mainThreadStatus = pthread_create(&mainThread, NULL, workInThread, NULL);
}

void darknet_svm::bboxImgCallback(const smoke::BboxImageConstPtr& msg){
    bboxes = (*msg).bboxes;
    
    if(!imgs){
        boost::unique_lock<boost::shared_mutex> lockImgCallback(mutexImgsStatus);
        subscriberStatus = false;
    }
    else{
        boost::unique_lock<boost::shared_mutex> lockImgCallback(mutexImgsStatus);
        subscriberStatus = false;
    }

    bounding_boxes = bboxes.bounding_boxes;
    // start to modify shared variables imgs, image_bridge, img_srv and bounding_boxes_u
    boost::unique_lock<boost::shared_mutex> lockImgCallback(mutexImgsStatus);
    imgs = (*msg).img;
    try{
        img = cv_bridge::toCvCopy(imgs, sensor_msgs::image_encodings::TYPE_8UC3)->image;
    }
    catch(cv_bridge::Exception& e){
        ROS_ERROR("cv_bridge exception: %s", e.what());
        return;
    }
    cv::cvtColor(img, img, cv::COLOR_BGR2GRAY);
    bounding_boxes_u.clear();

    header.seq = ++count;   // user defined counter
    header.stamp = ros::Time::now();   // time
    img_bridge = cv_bridge::CvImage(header, sensor_msgs::image_encodings::TYPE_8UC1, img);
    img_bridge.toImageMsg(img_srv);
    // select reasonable bounding boxes
    if(!bounding_boxes.empty()){
        bounding_boxes_u.push_back(bounding_boxes[i]);
    }
    for(int i = 0; i < bounding_boxes.size(); ++i){
        for(int j = i+1; j < bounding_boxes.size(); ++j){
            // rid of bboxes with high overlap
            int xmin_1 = int xmin = bounding_boxes[i].xmin;
            int xmax_1 = int xmin = bounding_boxes[i].xmax;
            int ymin_1 = int xmin = bounding_boxes[i].ymin;
            int ymax_1 = int xmin = bounding_boxes[i].ymax;
            int xmin_2 = int xmin = bounding_boxes[j].xmin;
            int xmax_2 = int xmin = bounding_boxes[j].xmax;
            int ymin_2 = int xmin = bounding_boxes[j].ymin;
            int ymax_2 = int xmin = bounding_boxes[j].ymax;
            int x_min = std::max(xmin_1, xmin_2);
            int y_min = std::max(ymin_1, ymin_2);
            int x_max = std::min(xmax_1, xmax_2);
            int y_max = std::min(ymax_1, ymax_2);
            float ovlap = static_cast<float>((xmax-xmin)*(ymax-ymin)) / static_cast<float>((xmax-xmin)*(ymax-ymin) + (xmax-xmin)*(ymax-ymin));
            if(ovlap < ovthresh){}
        }
    }
}

void *darknet_svm::callSVMInThread(void* param){
    darknet_svm *ds = (darknet_svm*)param;
    {
        boost::shared_lock<boost::shared_mutex> lockCallSVM(ds->mutexImgsStatus);
        ds->svm_srv.request.img = img_srv;
        ds->svm_srv.request.bboxes = bounding_boxes_u;
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