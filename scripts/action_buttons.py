#!/usr/bin/env python
import rospy
import roslib
from kobuki_msgs.msg import ButtonEvent
from kobuki_actTest import kobuki_actclient

class action_buttons():
    def __init__(self):
        rospy.on_shutdown(self.shutdown)
        self.ntime_dict = {"B0": 2., "B1": 5., "B2": 7.}
        rospy.Subscriber('/mobile_base/events/button', ButtonEvent, self.ButtonEventCallback)
        rospy.spin()
   
    def shutdown(self):
        rospy.loginfo('Stop action_buttons.')

    def ButtonEventCallback(self, data):
        if(data.state == ButtonEvent.RELEASED):
	    state = "released"
	else:
	    state = "pressed"
            if(data.button == ButtonEvent.Button0):
	        button = "B0"
	    elif(data.button == ButtonEvent.Button1):
	        button = "B1"
	    else:
	        button = "B2"
            rospy.loginfo("Button %s was %s."%(button, state))
            client = kobuki_actclient(self.ntime_dict[button])
            client.pub_action()

if __name__ == '__main__':
    rospy.init_node('kobuki_button')
    ab = action_buttons()
