#include "LBP.h"
#include "rgbData.h"
#include <vector>
#include "smoke/smoke_svm.h"
#include <sensor_msgs/Image.h>
#include <sensor_msgs/image_encodings.h>
#include <actionlib/client/simple_action_client.h>

int main(int argc, char **argv){
    ros::init(argc, argv, "smoke_svm_client");
    if (argc != 2){
        ROS_INFO("usage: smoke_svm_client batch_size");
        return -1;
    }

    ros::Rate rate(5);
    int shape[] = {1, 100, 100, 3};
    char c = argv[1][0];
    int bsize = atoi(&c);  // 'bsize' stands for "batch size"
    lbp::LBP extractor(8, 2, 25);
    cv::Mat rgbcus(Size(shape[1], shape[2]), CV_8UC3);
    
    std_msgs::Header header; // empty header
    cv_bridge::CvImage img_bridge;
    sensor_msgs::Image img_msg; // >> message to be sent (LBP histograms)

    smoke::smoke_svm srv;
    ros::NodeHandle nh;
    //image_transport::ImageTransport it(nh);
    std::string imgSVMSrvClient, alarmPub;
    int qsize;
    nh.param("services/image_svm_srv/name", imgSVMSrvClient, std::string("/kinectdev/smoke/smoke_svm_srv"));
    nh.param("publishers/alarm_pub/topic", alarmPub, std::string("/kinectdev/smoke/alarm"));
    nh.param("publishers/alarm_pub/queue_size", qsize, 1);
    ros::ServiceClient client = nh.serviceClient<smoke::smoke_svm>(imgSVMSrvClient.c_str());  //recognize emergencies.
    ros::Publisher alarm_pub = nh.advertise<std_msgs::Bool>(alarmPub, qsize);  //Alarm. Arguments are topic and queue_size;

    //std::vector<cv::Mat> graycus;
    cv::Mat graycus, hist, dst, hists;
    rgbData rD(shape, rgbcus);

    int counter = 0;
    while(ros::ok()){
        for(int i = 1; i < bsize; ++i){
            sleep(0.5);
            cv::cvtColor(rgbcus, graycus, CV_BGR2GRAY);
            extractor.RILBP(graycus, dst);
            hist = extractor.RILBPHistogram(dst);
            hists.push_back(hist);
            // converting cv::Mat to sensor::msgs::Image via cv_bridge
            ros::spinOnce();
        }
        header.seq = counter++; // user defined counter
        header.stamp = ros::Time::now(); // time
        img_bridge = cv_bridge::CvImage(header, sensor_msgs::image_encodings::TYPE_8UC1, hists);
        img_bridge.toImageMsg(img_msg); // from cv_bridge to sensor_msgs::Image
        srv.request.hst = img_msg;
        srv.request.dim = bsize;
        if(client.call(srv)){
            int res = srv.response.res;
            std_msgs::Bool msg;
            if(res == 1){
                ROS_INFO("Alarm!");
                ROS_ERROR("Alarm! Attention required.");
                msg.data = true;
                //kobukiAlarmClient.sendGoal(goal);
                //kobukiAlarmClient.waitForResult(ros::Duration(5.0));
            }
            else{
                ROS_INFO("Clear.");
                msg.data = false;
            }
            alarm_pub.publish(msg);
        }
        else{
            ROS_ERROR("Failed to call service smoke");
            ROS_INFO("Waiting for server smoke to correctly respond...");
            sleep(1);
            continue;
        }
        rate.sleep();
    }
    return 0;
}