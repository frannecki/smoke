#include "rgbData.h"
#include "smoke/smoke.h"
#include <actionlib/client/simple_action_client.h>
void getFrame(std::vector<int> &frame, cv::Mat& src){
    int i = src.cols, j = src.rows;
    int k, m, t;
    frame.clear();
    CvScalar s;
    for(k = 0; k < i; ++k)
      for(m = 0; m < j; ++m){
        s = src.at<Vec3b>(k, m);
          for(t = 0; t < 3; ++t)
            frame.push_back(s.val[t]);
      }
}

int main(int argc, char** argv){
    
    ros::init(argc, argv, "smoke_client");
    if (argc != 1){
        ROS_INFO("usage: smoke_client");
        return -1;
    }

    smoke::smoke srv;
    ros::NodeHandle nh;
    
    std::string imgSrvClient;
    std::string alarmPub;
    int qsize;
    nh.param("services/image_srv/name", imgSrvClient, std::string("/kinectdev/smoke/smoke_srv"));
    nh.param("publishers/image/topic", alarmPub, std::string("/kinectdev/smoke/alarm"));
    nh.param("publishers/image/queue_size", qsize, 1);
    ros::ServiceClient client = nh.serviceClient<smoke::smoke>(imgSrvClient);  //recognize emergencies.
    ros::Publisher alarm_pub = nh.advertise<std_msgs::Bool>(alarmPub, qsize);  //Alarm. Arguments are topic and queue_size;
    //actionlib::SimpleActionClient<smoke::AlarmAction> kobukiAlarmClient("/kinectdev/smoke/kobukiAlarm", true);  // true -> don't need ros::spin()
    //kobukiAlarmClient.waitForServer();
    //smoke::AlarmGoal goal;
    //goal.order = 3;

    int shape[] = {1, 100, 100, 3};
    cv::Mat rgbcus(Size(shape[1], shape[2]), CV_8UC3);
    rgbData rD(shape, rgbcus);
    int res;
    std::vector<int> rgbVec;
    std::vector<int> vshape(shape, shape+4);

    cv::namedWindow("rgb", CV_WINDOW_AUTOSIZE);
    while(ros::ok()){
        getFrame(rgbVec, rgbcus);
        srv.request.frame = rgbVec;
        srv.request.dim = vshape;
        if (client.call(srv)){
            res = (int)srv.response.res;
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
        cv::imshow("rgb", rgbcus);
        if(cv::waitKey(40) == 27)
            break;
        ros::spinOnce();
    }
    return 0;
}