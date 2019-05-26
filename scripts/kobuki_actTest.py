#!/usr/bin/env python
import rospy
import actionlib
import smoke.msg

if __name__ == '__main__':
    rospy.init_node('kobuki_action_test_node')
    rate = rospy.Rate(.1)
    client = actionlib.SimpleActionClient('/kinectdev/smoke/kobuki_alarm', smoke.msg.AlarmAction)
    goal = smoke.msg.AlarmGoal(5)
    for i in range(0, 5):
        client.send_goal(goal)
        client.wait_for_result(rospy.Duration.from_sec(5.0))
        rate.sleep()