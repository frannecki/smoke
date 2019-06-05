#include "darknetDetector.h"

namespace darknet_svm{
bool cmp_by_prob(const smoke::BoundingBox& bbox1, const smoke::BoundingBox& bbox2){
    return bbox1.probability > bbox2.probability;
}

std::string current_std_time(){
    std::string current_time;
    std::time_t end_time = std::chrono::system_clock::to_time_t(std::chrono::system_clock::now());
    char *timestr = std::ctime(&end_time);
    for(int i = 0; i < strlen(timestr)-1, timestr[i] != '\n';){
        if(timestr[i] == ' '){
            current_time.push_back('_');
            while(timestr[++i] == ' ');
        }
        else current_time.push_back(timestr[i++]);
    }
    current_time += std::string(".log");
    return current_time;
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
    logwriter.close();
}

void darknet_svm::init(){
    ROS_INFO("[darknetDetector] Node Started.");
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

    nh_.param("/smoke/logs/smoke_warning/path", logpath, std::string("./../log"));
    nh_.param("/smoke/logs/smoke_warning/logfile", logfile, std::string("./../log/warnings_"));
    logfile += current_std_time();
    ROS_INFO("[darknetDetector] log file path: %s", logfile.c_str());
    logwriter.open(logfile.c_str(), std::ios::app);

    count = 0;
    alarm.data = false;
    isNodeRunning = true;
    ovthresh = .7;
    subscriberStatus = false;
    subscriberStatusDelay = RateNP(4.);
    mainThreadDelay = RateNP(10.);
    imgCallbackDelay = RateNP(15.);
    htthresh = 20.;
    wdthresh = 20.;
    chim_prob_thresh = .8;

    img_bbox_sub = nh_.subscribe<smoke::BboxImage>(bboxImgSub, bboxImgSub_qs, &darknet_svm::bboxImgCallback, this);
    //ros::spinOnce();
    int mainThreadStatus = pthread_create(&mainThread, NULL, workInThread, (void*)this);
    if(mainThreadStatus != 0)
        ROS_ERROR("[darknetDetector] Failed to start thread (mainThread)");
}

void darknet_svm::bboxImgCallback(const smoke::BboxImageConstPtr& msg){
    // ROS_INFO("[darknetDetector] Subscriber (img_bbox_sub) callback.");
    bboxes = (*msg).bboxes;
    bounding_boxes = bboxes.bounding_boxes;
    // Modify shared variables 'imgs', 'image_bridge', 'img_srv' and 'bounding_boxes_u'
    boost::unique_lock<boost::shared_mutex> lockImgCallback(mutexImgsStatus);
    imgs = (*msg).img;
    try{
        img_bridge_sub = cv_bridge::toCvCopy(imgs, sensor_msgs::image_encodings::BGR8);
        //cv_bridge::CvImageConstPtr img_bridge_sub = cv_bridge::toCvShare((*msg).img, sensor_msgs::image_encodings::BGR8);
    }
    catch(cv_bridge::Exception& e){
        ROS_ERROR("[darknetDetector] cv_bridge exception: %s", e.what());
        return;
    }
    if(img_bridge_sub){
        boost::unique_lock<boost::shared_mutex> lockSubscriber(mutexSubscriberStatus);
        subscriberStatus = true;
    }
    else{
        {
            boost::unique_lock<boost::shared_mutex> lockSubscriber(mutexSubscriberStatus);
            subscriberStatus = false;
            ROS_ERROR("[darknetDetector] Warning: Invalid image!");
        }
        imgCallbackDelay.sleep();
        return;
    }
    // ROS_INFO("[darknetDetector] Received an image along with %lu bounding boxes", bounding_boxes.size());
    img = img_bridge_sub->image.clone();
    // cv::cvtColor(img, img, cv::COLOR_BGR2GRAY);
    bounding_boxes_u.clear();

    header.seq = ++count;   // user defined counter
    header.stamp = ros::Time::now();   // time
    img_bridge = cv_bridge::CvImage(header, sensor_msgs::image_encodings::BGR8, img);  // published image is of bgr.
    img_bridge.toImageMsg(img_srv);
    // selecting bounding boxes
    std::vector<int> overlap(bounding_boxes.size(), 0);
    std::vector<smoke::BoundingBox> chimbox;
    std::vector<smoke::BoundingBox> bounding_boxes_tmp;
    std::sort(bounding_boxes.begin(), bounding_boxes.end(), cmp_by_prob);
    for(int i = 0; i < bounding_boxes.size(); ++i){
        int xmin_1 = bounding_boxes[i].xmin;
        int xmax_1 = bounding_boxes[i].xmax;
        int ymin_1 = bounding_boxes[i].ymin;
        int ymax_1 = bounding_boxes[i].ymax;
        float height_1 = static_cast<float>(std::max(ymax_1-ymin_1+1, 0));
        float width_1 = static_cast<float>(std::max(xmax_1-xmin_1+1, 0));
        if(overlap[i] == 0 && bounding_boxes[i].Class == "smoke" && 
            /*bounding_boxes[i].probability > .7 &&*/ height_1 >= htthresh && width_1 >=wdthresh)
        {    
            bounding_boxes_tmp.push_back(bounding_boxes[i]);
        }
        else continue;
        for(int j = i+1; j < bounding_boxes.size(); ++j){
            // rid of redundant bboxes (according to overlap)
            if(bounding_boxes[j].Class == "chimney" && 
               bounding_boxes[j].probability >= chim_prob_thresh)
            {
                chimbox.push_back(bounding_boxes[j]);
                continue;
            }
            int xmin_2 = bounding_boxes[j].xmin;
            int xmax_2 = bounding_boxes[j].xmax;
            int ymin_2 = bounding_boxes[j].ymin;
            int ymax_2 = bounding_boxes[j].ymax;
            int x_min = std::max(xmin_1, xmin_2);
            int y_min = std::max(ymin_1, ymin_2);
            int x_max = std::min(xmax_1, xmax_2);
            int y_max = std::min(ymax_1, ymax_2);
            float height = static_cast<float>(std::max(y_max-y_min+1, 0));
            float width = static_cast<float>(std::max(x_max-x_min+1, 0));
            float intersec = height * width;
            float area1 = static_cast<float>((xmax_1-xmin_1+1)*(ymax_1-ymin_1+1));
            float area2 = static_cast<float>((xmax_2-xmin_2+1)*(ymax_2-ymin_2+1));
            float uni = (area1 + area2) - intersec;
            float ovlap = intersec / (uni + std::numeric_limits<float>::epsilon());
            if(ovlap > ovthresh){
                overlap[j] = 1;
            }
        }
    }

    std::vector<int> vapor(bounding_boxes_tmp.size(), 0);
    // rid of bounding boxes of vapor emitted from chimneys
    for(int i = 0; i < chimbox.size(); ++i){
        int xmin_chim = chimbox[i].xmin;
        int xmax_chim = chimbox[i].xmax;
        int ymin_chim = chimbox[i].ymin;
        int ymax_chim = chimbox[i].ymax;
        for(int j = 0; j < bounding_boxes_tmp.size(); ++j){
            int xmin_smk = bounding_boxes_tmp[j].xmin;
            int xmax_smk = bounding_boxes_tmp[j].xmax;
            int ymin_smk = bounding_boxes_tmp[j].ymin;
            int ymax_smk = bounding_boxes_tmp[j].ymax;
            if(ymax_smk > (ymax_chim + ymin_chim) / 2)  continue;
            else if(ymin_smk > ymin_chim)  continue;
            else if(xmax_smk <= xmin_chim || xmin_smk >= xmax_chim)  continue;
            else  vapor[j] = 1;
        }
    }

    for(int i = 0; i < bounding_boxes_tmp.size(); ++i){
        if(vapor[i] == 0)
            bounding_boxes_u.push_back(bounding_boxes_tmp[i]);
    }
    imgCallbackDelay.sleep();
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
        ROS_ERROR("[darknetDetector] Failed to call SVC classifier!");
        ROS_INFO("[darknetDetector] Waiting for server to properly respond...");
    }
}

void *darknet_svm::alarmInThread(void* param){
    darknet_svm *ds = (darknet_svm*)param;
    
    boost::shared_lock<boost::shared_mutex> lockAlarm(ds->mutexAlarmStatus);
    ds->alarm_pub.publish(ds->alarm);
    if(ds->alarm.data == true){
        boost::unique_lock<boost::shared_mutex> lockLogwriterStatus(ds->mutexLogWriterStatus);
        ROS_INFO("[darknetDetector] Alarm! Smoke occurred.");
        ds->logwriter << "[darknetDetector][Alarm] Smoke occurred" << std::endl;
        int count_ = 0;
        for(int i = 0; i < ds->bounding_box_u_response.size(); ++i){
            if(ds->bounding_box_u_response[i] == 1){
                ROS_INFO("[darknetDetector] Position %d: (%ld, %ld) -> (%ld, %ld)", ++count_, 
                    ds->bounding_boxes_u[i].xmin, ds->bounding_boxes_u[i].ymin, ds->bounding_boxes_u[i].xmax, ds->bounding_boxes_u[i].ymax);

                std::time_t currtime = std::chrono::system_clock::to_time_t(std::chrono::system_clock::now());
                char *timestr = std::ctime(&currtime);
                timestr[strlen(timestr)-1] = '\0';
                ds->logwriter << std::setiosflags(std::ios::left) << "[darknetDetector][Alarm " << timestr << "][Position] " << count_ \
                              << ": (" << ds->bounding_boxes_u[i].xmin << ", " << ds->bounding_boxes_u[i].ymin << ") -> " \
                              << "(" << ds->bounding_boxes_u[i].xmax << ", " << ds->bounding_boxes_u[i].ymin << ")" << std::endl;
            }
        }
    }
}

void *darknet_svm::workInThread(void* param){
    ROS_INFO("[darknetDetector] mainThread running.");
    darknet_svm *ds = (darknet_svm*)param;
    pthread_t callSVMThread;
    pthread_t alarmThread;
    void* voidPtr;
    int callSVMThreadStatus, alarmThreadStatus;
    //if(!(ds->subscriberStatus)){ sleep(1); }

    while(ros::ok()){
        {
            boost::shared_lock<boost::shared_mutex> lockNodeStatus(ds->mutexNodeStatus);
            if(!ds->isNodeRunning)  break;
        }
        bool ts;
        {
            boost::shared_lock<boost::shared_mutex> lockSubscriber(ds->mutexSubscriberStatus);
            ts = ds->subscriberStatus;
        }
        if(!ts)  { 
            ds->subscriberStatusDelay.sleep(); 
            //ROS_INFO("[darknetDetector] Waiting for subscriber to respond");
            continue; 
        }
        callSVMThreadStatus = pthread_create(&callSVMThread, NULL, callSVMInThread, param);
        if(callSVMThreadStatus != 0){
            ROS_ERROR("[darknetDetector] Failed to start thread (callSVMThread).");
            return voidPtr;
        }
        
        alarmThreadStatus = pthread_create(&alarmThread, NULL, alarmInThread, param);
        if(alarmThreadStatus != 0){
            ROS_ERROR("[darknetDetector] Failed to start thread (alarmThread).");
            return voidPtr;
        }

        pthread_join(callSVMThread, NULL);
        pthread_join(alarmThread, NULL);
        ds->mainThreadDelay.sleep();
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
}  // namespace 'darknet_svm'