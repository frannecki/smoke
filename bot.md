# Turtlebot Control
## Configuration
1. robot
   ```shell
   export ROS_MASTER_URI=http://${IP_ROBOT}:11311
   export ROS_HOSTNAME=${IP_ROBOT}
   export ROS_IP=${IP_WORKSTATION}  # optional
   export TURTLEBOT_3D_SENSOR=kinect
   ```

2. workstation
   ```shell
   export ROS_MASTER_URI=http://${IP_ROBOT}:11311
   export ROS_HOSTNAME=${IP_WORKSTATION}
   ```

   For local simulation with [rbx1](https://github.com/pirobot/rbx1), connection to real robots is not necessary.
   ```shell
   export ROS_MASTER_URI=http://localhost:11311
   export ROS_HOSTNAME=localhost
   ```

## Bringup
1. bringup
   ```shell
   roslaunch turtlebot_bringup minimal.launch  # for real robot
   roslaunch rbx1_bringup fake_turtlebot.launch  # for rbx1 simulator
   ```
2. teleop
   ```shell
   roslaunch turtlebot_teleop keyboard_teleop.launch  # for real robot
   roslaunch rbx1_nav keyboard_teleop.launch  # for rbx1 simulator
   ```

## SLAM
1. SLAM  
   ```shell
   # for real robot
   sudo sh -c "echo -1 > /sys/module/usbcore/parameters/autosuspend"  # avoid usb autosuspend
   roslaunch turtlebot_navigation gmapping_demo.launch
   
   # for rbx1 simulator
   roslaunch rbx1_nav gmapping.launch

   # view in rviz
   roslaunch turtlebot_rviz_launchers view_navigation.launch 
   ```

2. adaptive localization / autonomous driving
   ```shell
   roslaunch turtlebot_navigation amcl_demo map_file:=xx.yaml  # for real robot
   roslaunch rbx1_nav fake_amcl.launch map_file:=xx.yaml  # for rbx1 simulator
   ```

## Control with scripts
We can launch a python script to adjust the real time velocity of the robot.
1. Autonomous navigation
   
   Given a point on the map, the robot could plan a path and navigate itself to the destination.
   ```python
   ## amcl should be launched as the action server
   from move_base_msgs.msg import MoveBaseAction, MoveBaseGoal
   from geometry_msgs.msg import Pose, Point, Quaternion
   move_base = actionlib.SimpleActionClient("move_base", MoveBaseAction)  # topic: 'move_base'
   move_base.wait_for_server(rospy.Duration(5))
   goal = MoveBaseGoal()
   goal.target_pose.header.frame_id = 'map'
   goal.target_pose.header.stamp = rospy.Time.now()
   goal.target_pose.pose = Pose(Point(pos['x'], pos['y'], 0.000), Quaternion(quat['r1'], quat['r2'], quat['r3'], quat['r4']))

   move_base.send_goal(goal)
   success = move_base.wait_for_result(rospy.Duration(60))
   state = move_base.get_state()
   ```

2. Specify velocity (linear and angular)
   
   The cmd_vel client could preempt the navigation action goals.
   ```python
   from geometry_msgs.msg import Twist
   topic = 'cmd_vel_mux/input/navi'  # '/cmd_vel' for rbx1 simulator
   cmd_vel = rospy.Publisher(topic, Twist, queue_size=10)
   move_cmd = Twist()
   move_cmd.linear.x = 0.5
   move_cmd.angular.z = 0.5
   cmd_vel.publish(move_cmd)
   ```