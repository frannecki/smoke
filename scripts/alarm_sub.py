#!/usr/bin/env python
import rospy
from std_msgs.msg import Bool
import time
import itchat

class listener():
    def __init__(self):
        rospy.on_shutdown(self.shutdown)  # behavior on shutdown
        itchat.auto_login(hotReload=True)  # log in to Weixin
        
        itchat.send('[{0}]This is a trivial robot sending messages. Session started.'.format(self.timestr()), toUserName='filehelper')
        rospy.init_node('listener', anonymous=True)

        alarmSub = ''
        alarmSub_qs = 1
        alarmSub = rospy.get_param("subscribers/alarm_sub/topic")
        alarmSub_qs = rospy.get_param("subscribers/alarm_sub/queue_size")
        #rospy.Subscriber("/kinectdev/monitor/alarm", Bool, callback=self.alarmcallback, queue_size=1)
        rospy.Subscriber(alarmSub, Bool, callback=self.alarmcallback, queue_size=alarmSub_qs)
        rospy.spin()

    def alarmcallback(self, msg):
        rospy.loginfo('[{0}]Heard Alarm {1}'.format(self.timestr(), msg.data))
        if (msg.data == True):
            print('Alarm! Sending message to Weixin.')
            itchat.send('[{0}]Alarm! Check if emergency occurred.'.format(self.timestr()), toUserName='filehelper')
            print('-----------------------------------')
            rospy.sleep(5)
        else:       
            rospy.sleep(0.5)
            pass

    def timestr(self):
        return time.strftime('%H:%M:%S', time.localtime(time.time()))

    def shutdown(self):
        itchat.send('[{0}]Session terminated.'.format(self.timestr()), toUserName='filehelper')

if __name__ == '__main__':
    listener()