#include "subnode.h"
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
    return current_time;
}

QNode::QNode(int argc, char** argv ): init_argc(argc), init_argv(argv){}

QNode::~QNode() {
    if(ros::isStarted()) {
      ros::shutdown();
      ros::waitForShutdown();
    }
    wait();
}

bool QNode::init() {
    ros::init(init_argc,init_argv,"qtapp");
    if ( ! ros::master::check() ) {
    	return false;
    }
    ros::start(); // explicitly needed since our nodehandle is going out of scope.
    ros::NodeHandle n;
    // ROS node related
    std::string imgsub, alarmsub;
    nh_.param("/qtdemo/subscribers/detection_image/topic", imgsub, std::string("/darknet_ros/detection_image"));
    nh_.param("/qtdemo/subscribers/alarm_sub/topic", alarmsub, std::string("/kinectdev/smoke/alarm"));
    img_sub = nh_.subscribe<sensor_msgs::Image>(imgsub, 1, &QNode::imgShowCallback, this);
    alarm_sub = nh_.subscribe<std_msgs::Bool>(alarmsub, 1, &QNode::alarmCallback, this);
	start();
	return true;
}

bool QNode::init(const std::string &master_url, const std::string &host_url) {
    std::map<std::string,std::string> remappings;
    remappings["__master"] = master_url;
    remappings["__hostname"] = host_url;
    ros::init(remappings,"qtapp");
    if ( ! ros::master::check() ) {
    	return false;
    }
    ros::start(); // explicitly needed since our nodehandle is going out of scope.
    ros::NodeHandle n;

    // ROS node related
    std::string imgsub, alarmsub;
    nh_.param("/qtdemo/subscribers/detection_image/topic", imgsub, std::string("/darknet_ros/detection_image"));
    nh_.param("/qtdemo/subscribers/alarm_sub/topic", alarmsub, std::string("/kinectdev/smoke/alarm"));
    img_sub = nh_.subscribe<sensor_msgs::Image>(imgsub, 1, &QNode::imgShowCallback, this);
    alarm_sub = nh_.subscribe<std_msgs::Bool>(alarmsub, 1, &QNode::alarmCallback, this);
    start();
    return true;
}

void QNode::imgShowCallback(const sensor_msgs::ImageConstPtr& msg){
    try{
        img_bridge_sub = cv_bridge::toCvCopy(*msg, sensor_msgs::image_encodings::BGR8);
    }
    catch(cv_bridge::Exception& e){
        return;
    }
    if(!img_bridge_sub){
        //imgCallbackDelay.sleep();
        return;
    }
    cv::Mat img = img_bridge_sub->image.clone();
    cv::resize(img, img, cv::Size(640, 480));
    //ROS_INFO("(%d, %d)", img.rows, img.cols);
    pixmap = cvMatToQPixmap(img);
    emit imgUpdated();
}

void QNode::alarmCallback(const std_msgs::Bool::ConstPtr& msg){
    if(msg->data == false)  return;
    logging_model.insertRows(logging_model.rowCount(),1);
    std::stringstream logging_model_msg;
    logging_model_msg << "[INFO] [" << current_std_time() << "]: " << "Alarm!";
    QVariant new_row(QString(logging_model_msg.str().c_str()));
    logging_model.setData(logging_model.index(logging_model.rowCount()-1),new_row);
    emit loggingUpdated();
}

void QNode::run() {
    ros::Rate loop_rate(20);
    int count = 0;
    while ( ros::ok() ) {
    	ros::spinOnce();
    	loop_rate.sleep();
    	++count;
    }
    std::cout << "Ros shutdown, proceeding to close the gui." << std::endl;
    emit rosShutdown(); // used to signal the gui for a shutdown (useful to roslaunch)
}

QStringListModel* QNode::loggingModel() { return &logging_model; }

QPixmap* QNode::pixMap() { return &pixmap; }

void QNode::loggingUpdated(){}

void QNode::imgUpdated(){}

void QNode::rosShutdown(){}