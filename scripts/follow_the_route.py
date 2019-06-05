#!/usr/bin/env python
'''
Adaption by frankw 
'''

import rospy
import yaml
from go_to_specific_point_on_map import GoToPose

if __name__ == '__main__':

    # Read information from yaml file
    try:
        routedatafile = rospy.get_param('/smoke/nav/route')
    except KeyError:
        routedatafile = './../config/jun05.yaml'
    with open(routedatafile, 'r') as stream:
        dataMap = yaml.load(stream)
    
    rospy.init_node('follow_route', anonymous=False)
    navigator = GoToPose()
    pose_ind = 0
    while(not rospy.is_shutdown()):    
        name = dataMap[pose_ind]['filename']
        # Navigation
        rospy.loginfo("Go to %s pose", name[:-4])
        success = navigator.goto(dataMap[pose_ind]['position'], dataMap[pose_ind]['quaternion'])
        if not success:
            rospy.loginfo("Failed to reach %s pose", name[:-4])
            continue
        rospy.loginfo("Reached %s pose", name[:-4])
        rospy.sleep(1)
        pose_ind += 1
        if(pose_ind == len(dataMap)):
            pose_ind = 0
