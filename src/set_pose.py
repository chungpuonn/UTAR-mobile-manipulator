#! /usr/bin/env python

import time
import sys
import copy
import rospy
import moveit_commander
import moveit_msgs.msg
import geometry_msgs.msg
from std_msgs.msg import String
from moveit_commander.conversions import pose_to_list

moveit_commander.roscpp_initialize(sys.argv)
rospy.init_node('move_group_python_interface_tutorial', anonymous=True)

robot = moveit_commander.RobotCommander()
scene = moveit_commander.PlanningSceneInterface()    
group = moveit_commander.MoveGroupCommander("arm")
group.set_planner_id("PRMkConfigDefault")
#"PRMkConfigDefault"
group.set_planning_time(10)
group_name = "arm"
move_group = moveit_commander.MoveGroupCommander(group_name)
eef_link = move_group.get_end_effector_link()
display_trajectory_publisher = rospy.Publisher('/move_group/display_planned_path', moveit_msgs.msg.DisplayTrajectory, queue_size=1)



raw_input()

#IK - Update end-effector coordinates 
pose_target = geometry_msgs.msg.Pose()

#3
# pose_target.orientation.x = 0.000000324492066582
# pose_target.orientation.y = -0.0000456296259049
# pose_target.orientation.z = 0.00711125403583
# pose_target.orientation.w = 0.999974713672

# pose_target.position.x = 0.0731916945437
# pose_target.position.y = 0.00112457643663
# pose_target.position.z = 0.372459720267

#2
# pose_target.orientation.x = 0.00000905222926613
# pose_target.orientation.y = -0.0000944386080891
# pose_target.orientation.z = 0.0954157278857
# pose_target.orientation.w = 0.995437506763

# pose_target.position.x = 0.130149321316
# pose_target.position.y = 0.0263180733394
# pose_target.position.z = 0.560728398757

#1
# pose_target.orientation.x = 0.00676378671188
# pose_target.orientation.y = -0.0195477204167
# pose_target.orientation.z = 0.326922659537
# pose_target.orientation.w = 0.944824699347

# pose_target.position.x = 0.163145225387
# pose_target.position.y = 0.13287362684
# pose_target.position.z = 0.438792872732

# group.set_pose_target(pose_target)

# plan1 = group.plan()
target_position = [0.163145225387, 0.13287362684, 0.438792872732]
group.set_position_target(target_position)

start = time.time()
plan1 = group.plan()
end = time.time()
print(end-start)
rospy.sleep(5)
# group.go(wait=True)
# rospy.sleep(5)


moveit_commander.roscpp_shutdown()
