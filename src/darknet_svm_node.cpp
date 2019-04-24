#include "darknetDetector.h"

int main(int argc, char **argv){
    ros::init(argc, argv, "darknet_detector");
    ros::NodeHandle nh("~");
    darknet_svm::darknet_svm dsvm(nh);
    ros::spin();
    return 0;
}