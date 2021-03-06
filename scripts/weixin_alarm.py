#!/usr/bin/env python
import rospy
from std_msgs.msg import Bool
import time
import smoke.msg
# for weixin alarm
import itchat

class listener():
    def __init__(self):
        rospy.on_shutdown(self.shutdown)  # behavior on shutdown
        itchat.auto_login(hotReload=True)  # log in to Weixin  
        itchat.send('[{0}]This is a trivial robot sending messages. Session started.'.format(self.timestr()), toUserName='filehelper')
        
        self.rate = rospy.Rate(.2)
        try:
            self.alarmSub = rospy.get_param("/smoke/subscribers/alarm_sub/topic")
            self.alarmSub_qs = rospy.get_param("/smoke/subscribers/alarm_sub/queue_size")
        except KeyError:
            self.alarmSub = '/kinectdev/smoke/alarm'
            self.alarmSub_qs = 1

        rospy.Subscriber(self.alarmSub, Bool, callback=self.alarmcallback, queue_size=self.alarmSub_qs)
        self.client = actionlib.SimpleActionClient(self.actname, smoke.msg.AlarmAction)
        rospy.spin()

    def alarmcallback(self, msg):
        rospy.loginfo('[alarm_sub_node] [{0}]Heard Alarm {1}'.format(self.timestr(), msg.data))
        if (msg.data == True):
            rospy.loginfo('[alarm_sub_node] Alarm! Sending message to Weixin.')
            itchat.send('[{0}]Alarm! Check if emergency occurred.'.format(self.timestr()), toUserName='filehelper')
            print('-----------------------------------')
            self.rate.sleep()
        else:       
            rospy.sleep(0.5)
            pass

    def timestr(self):
        return time.strftime('%H:%M:%S', time.localtime(time.time()))

    def shutdown(self):
        itchat.send('[{0}]Session terminated.'.format(self.timestr()), toUserName='filehelper')

if __name__ == '__main__':
    rospy.init_node('alarm_sub_node', anonymous=True)
    listener()