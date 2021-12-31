#! /usr/bin/env python

import sys
import copy
import rospy
import moveit_commander
import moveit_msgs.msg
import geometry_msgs.msg
from std_msgs.msg import String
from std_msgs.msg import Header
from moveit_commander.conversions import pose_to_list
from moveit_msgs.msg import RobotState, Constraints, OrientationConstraint
from add_collision_object_v0 import add_coll_obj
from attach_obj import attach_dettach

import tf


moveit_commander.roscpp_initialize(sys.argv)
rospy.init_node('manipulation', anonymous=True)

robot = moveit_commander.RobotCommander()
scene = moveit_commander.PlanningSceneInterface()    
group = moveit_commander.MoveGroupCommander("arm")
group_h = moveit_commander.MoveGroupCommander("hand")
group.set_planner_id("PRMkConfigDefault")
#"PRMkConfigDefault"
group.set_planning_time(10)
group_name = "arm"
move_group = moveit_commander.MoveGroupCommander(group_name)
eef_link = move_group.get_end_effector_link()
display_trajectory_publisher = rospy.Publisher('/move_group/display_planned_path', moveit_msgs.msg.DisplayTrajectory, queue_size=1)

co = add_coll_obj()
att_dett = attach_dettach()




#FK (Set Joint Value) - Close the gripper 
print "============ Press `Enter` to close the gripper and pick object..."
raw_input()
group_h_variable_values = group_h.get_current_joint_values()


plan2 = group_h.plan(group_h_variable_values)
rospy.sleep(1)
group_h.go(wait=True)
while plan2.joint_trajectory.points:  # True if trajectory contains points
    print "Close by decrementing step angle of left finger..."
    group_h.go(wait=True)
    rospy.sleep(1)
    if group_h_variable_values[0] > -0.3142:  #Within joint limit
        group_h_variable_values[0] -= 0.0174533   # approx -0.5 degrees
    else:
        break
    plan2 = group_h.plan(group_h_variable_values)

print "Left finger in contact with object!"
rospy.sleep(2)

group_h_variable_values[1] += 0.0174533   # approx +0.5 degrees
plan2 = group_h.plan(group_h_variable_values)
rospy.sleep(1)
while plan2.joint_trajectory.points:  # True if trajectory contains points
    print "Close by decrementing step angle of right finger..."
    group_h.go(wait=True)
    rospy.sleep(1)
    if group_h_variable_values[1] < 0.3142:  #Within joint limit
        group_h_variable_values[1] += 0.0174533   # approx -0.5 degreess
    else:
        break
    plan2 = group_h.plan(group_h_variable_values)

print "Right finger in contact with object!"
rospy.sleep(2)


#(Set Pre-defined Pose) - Place at right pocket
att_dett.attach_box()
group.set_path_constraints(None)
print "============ Press `Enter` to place the target object to right pocket..."
raw_input()
group.set_named_target("TOP")
plan1 = group.plan()
rospy.sleep(2)
group.go(wait=True)
rospy.sleep(2)
#FK (Set Joint Value)
group_variable_values = group.get_current_joint_values()
group_variable_values[0] = -1.45 #Rotate CW 90 degrees
group.set_joint_value_target(group_variable_values)
plan1 = group.plan()
group.go(wait=True)
group.set_named_target("RIGHT")
plan1 = group.plan()
rospy.sleep(2)
group.go(wait=True)
rospy.sleep(2)

#(Set Pre-defined Pose) - Open the gripper
att_dett.dettach_box()
print "============ Press `Enter` to open the gripper and release object..."
raw_input()
group_h.set_named_target("OPEN")
plan2 = group_h.plan()
rospy.sleep(2)
group_h.go(wait=True)
rospy.sleep(2)

#(Set Pre-defined Pose)
print "============ Press `Enter` to prepare next sensing for manipulation..."
raw_input()
group.set_named_target("BACK")
plan1 = group.plan()
rospy.sleep(2)
group.go(wait=True)
rospy.sleep(2)





moveit_commander.roscpp_shutdown()
