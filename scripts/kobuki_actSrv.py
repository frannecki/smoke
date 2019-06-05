#!/usr/bin/env python
import rospy
from std_msgs.msg import Bool
import time
from geometry_msgs.msg import Twist
from math import radians
import actionlib
import smoke.msg

class kobuki_actSrv(object):
    def __init__(self, name, topic_vel):
        self._name = name
        self.topic_vel = topic_vel
        self.server = actionlib.SimpleActionServer(self._name, smoke.msg.AlarmAction, 
            execute_cb=self.kobukialarmcallback, auto_start=False)
        self._feedback = smoke.msg.AlarmFeedback()
        self._res = smoke.msg.AlarmResult()

        self.react = rospy.Publisher(topic_vel, Twist, queue_size = 10)
        self.move_cmd = Twist()
        self.move_cmd.linear.x = .0
        self.move_cmd.angular.z = 1.
        self.rate = rospy.Rate(10) # 10Hz
        self.server.start()

    def kobukialarmcallback(self, goal):
        self._feedback.fback = 0.0
        self._res.res = 0
        #rospy.loginfo('order of goal: {}'.format(goal.order))
        for i in range(0, int(goal.order * 10)):
            self.react.publish(self.move_cmd)
            self._feedback.fback += 1 / float(goal.order * 10)
            #rospy.loginfo('[kobuki_action_node] {0} Progress: {1}'.format(self._name, self._feedback.fback))
            self.server.publish_feedback(self._feedback)
            self.rate.sleep()

        self._res.res = 1.
        self.server.set_succeeded(self._res)
        rospy.loginfo('[kobuki_action_node] {0} complete: {1}'.format(self._name, self._res.res))
            

if __name__ == '__main__':
    rospy.init_node('kobuki_actSrv', anonymous=True)
    try:
        name = rospy.get_param('/smoke/actions/kobuki_alarm/name')
    except KeyError:
        name = '/kinectdev/smoke/kobuki_alarm'
    try:
        topic_vel = rospy.get_param('/smoke/nav/veltopic')
    except KeyError:
        topic_vel = 'cmd_vel_mux/input/navi'
    
    kobuki_actSrv(name, topic_vel)
    #kobuki_actSrv('/kinectdev/smoke/kobukiAlarm')
    rospy.spin()
