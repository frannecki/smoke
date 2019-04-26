#!/usr/bin/env python
import rospy
from std_msgs.msg import Bool
import time
from geometry_msgs.msg import Twist
from math import radians
import roslib
import actionlib
import smoke.msg
roslib.load_manifest('smoke')

class kobuki_actSrv(object):
    def __init__(self, name):
        rospy.init_node('kobuki_actSrv')
        self._name = name
        self.server = actionlib.SimpleActionServer(self._name, smoke.msg.AlarmAction, 
            execute_cb=self.kobukialarmcallback, auto_start=False)
        self._feedback = smoke.msg.AlarmFeedback()
        self._res = smoke.msg.AlarmResult()

        self.react = rospy.Publisher('cmd_vel_mux/input/navi', Twist, queue_size = 10)
        self.move_cmd = Twist()
        self.move_cmd.linear.x = 0
        self.rate = rospy.Rate(1) # 5Hz
        self.server.start()

    def kobukialarmcallback(self, goal):
        self._feedback.fback = 0.0
        self._res.res = 0
        rospy.loginfo('{0} Progress: {1}'.format(self._name, self._feedback.fback))
        for i in range(0, goal.order):
            self.react.publish(self.move_cmd)
            self._feedback.fback = (i+1) / goal.order
            self.server.publish_feedback(self._feedback)
            self.rate.sleep()

        if(self._feedback.fback == 1.0):
            self._res.res = 1
            self.server.set_succeeded(self._res)
            rospy.loginfo('{0} Complete: {1}'.format(self._name, self._res.res))

if __name__ == '__main__':
    name = rospy.get_param("actions/kobuki_alarm/name")
    kobuki_actSrv(name)
    #kobuki_actSrv('/kinectdev/smoke/kobukiAlarm')
    rospy.spin()