#!/usr/bin/env python
import rospy
import actionlib
import smoke.msg

class kobuki_actclient():
    def __init__(self, ntime):
        self.ntime = ntime
        rospy.on_shutdown(self.shutdown)
        self.rate = rospy.Rate(1. / float(self.ntime))
        self.client = actionlib.SimpleActionClient('/kinectdev/smoke/kobuki_alarm', smoke.msg.AlarmAction)
        self.goal = smoke.msg.AlarmGoal(self.ntime)
    
    def pub_action(self):
        for i in range(0, 5):
            self.client.send_goal(self.goal)
            self.client.wait_for_result(rospy.Duration.from_sec(self.ntime))
            self.rate.sleep()

    def shutdown(self):
        rospy.loginfo('Stop kobuki action client')


if __name__ == '__main__':
    rospy.init_node('kobuki_action_test_node')
    client = kobuki_actclient(2.)
    client.pub_action()
    
