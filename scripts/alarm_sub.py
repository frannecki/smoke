#!/usr/bin/env python
import rospy
from std_msgs.msg import Bool
import time
import smoke.msg
# for robot action
import actionlib

class listener():
    def __init__(self):
        rospy.on_shutdown(self.shutdown)  # behavior on shutdown
        
        self.rate = rospy.Rate(.5)
        try:
            self.alarmSub = rospy.get_param("/smoke/subscribers/alarm_sub/topic")
            self.alarmSub_qs = rospy.get_param("/smoke/subscribers/alarm_sub/queue_size")
            self.actname = rospy.get_param('/smoke/actions/kobuki_alarm/name')
        except KeyError:
            self.alarmSub = '/kinectdev/smoke/alarm'
            self.alarmSub_qs = 1
            self.actname = '/kinectdev/smoke/kobuki_alarm'

        rospy.Subscriber(self.alarmSub, Bool, callback=self.alarmcallback, queue_size=self.alarmSub_qs)
        self.client = actionlib.SimpleActionClient(self.actname, smoke.msg.AlarmAction)
        rospy.spin()

    def alarmcallback(self, msg):
        rospy.loginfo('[alarm_sub_node] [{0}]Heard Alarm {1}'.format(self.timestr(), msg.data))
        if (msg.data == True):
            rospy.loginfo('[alarm_sub_node] Alarm!')
            print('-----------------------------------')
            goal = smoke.msg.AlarmGoal(2)
            self.client.send_goal(goal)
            self.client.wait_for_result(rospy.Duration.from_sec(5.0))
            self.rate.sleep()
        else:       
            rospy.sleep(0.5)
            pass

    def timestr(self):
        return time.strftime('%H:%M:%S', time.localtime(time.time()))

    def shutdown(self):
        rospy.loginfo('Stop alarm subscriber node')

if __name__ == '__main__':
    rospy.init_node('alarm_sub_node', anonymous=True)
    listener()