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


# print "============ Press `Enter` to detect the potential obstacles that are not able to be detected by 2D laser scan..."
# raw_input()
# group.set_named_target("RIGHT")
# plan1 = group.plan()
# rospy.sleep(2)
# group.go(wait=True)
# rospy.sleep(2)
# print "============ ..."
# raw_input()
# group.set_named_target("LEFT")
# plan1 = group.plan()
# rospy.sleep(2)
# group.go(wait=True)
# rospy.sleep(2)
# print "============ ..."
# raw_input()
# group.set_named_target("HOME")
# plan1 = group.plan()
# rospy.sleep(2)
# group.go(wait=True)
# rospy.sleep(2)

#(Set Pre-defined Pose)
print "============ Press `Enter` to prepare sensing for manipulation..."
raw_input()
group.set_named_target("SENSE")
plan1 = group.plan()
rospy.sleep(3)
group.go(wait=True)
rospy.sleep(3)

#FK (Set Joint Value)
print "============ Press `Enter` to rotate depth camera around to perceive a full environment of station..."
raw_input()
group_variable_values = group.get_current_joint_values()
group.set_named_target("SENSE")
plan1 = group.plan()
rospy.sleep(2)
group.go(wait=True)
rospy.sleep(2)

group_variable_values = group.get_current_joint_values()
group_variable_values[0] = -1.4 #Rotate CW 90 degrees
group.set_joint_value_target(group_variable_values)
plan1 = group.plan()
rospy.sleep(2)
group.go(wait=True)
rospy.sleep(2)
#FK (Set Joint Value)
group_variable_values = group.get_current_joint_values()
group_variable_values[0] = 1.4 #Rotate CW 90 degrees
group.set_joint_value_target(group_variable_values)
plan1 = group.plan()
rospy.sleep(2)
group.go(wait=True)
rospy.sleep(2)
#FK (Set Joint Value)
group_variable_values = group.get_current_joint_values()
group_variable_values[0] = 0 #Back to 0 degrees
group.set_joint_value_target(group_variable_values)
plan1 = group.plan()
rospy.sleep(2)
group.go(wait=True)
rospy.sleep(2)

group.set_named_target("SENSE")
plan1 = group.plan()
rospy.sleep(2)
group.go(wait=True)
rospy.sleep(2)



# Orientation constraint
group.goal_c = Constraints()
header = Header()
header.frame_id = "link_1"
orientation_c = OrientationConstraint()
orientation_c.header = header
orientation_c.link_name = "eff_link"
orientation_c.orientation.w = 1.0
orientation_c.absolute_x_axis_tolerance = 0.05
orientation_c.absolute_y_axis_tolerance = 0.05
orientation_c.absolute_z_axis_tolerance = 0.05
orientation_c.weight = 1.0
group.goal_c.orientation_constraints.append(orientation_c)
# group.set_path_constraints(group.goal_c)
# group.set_path_constraints(None)

# robot.hand.pick("Box_0")


# Subscribe to tf frame pose
listener = tf.TransformListener()

rect_flag = 0
cyl_flag = 0
sphe_flag = 0

while(rect_flag == 0 ):
    try:
        rect_flag = listener.frameExists("object_28")
    except (tf.LookupException, tf.ConnectivityException, tf.ExtrapolationException):
        print('Cant get frame of wooden block, retrying...')

print('Found wooden block!')
co.add_coll_obj()

listener.waitForTransform("/base_footprint", "/object_28", rospy.Time(), rospy.Duration(4.0))
(trans, rot) = listener.lookupTransform('base_footprint', 'object_28', rospy.Time())
goal_coord = [trans[0], trans[1], trans[2]] # x, y, z

#FK (Set Joint Value) - Rotate CW 20 degrees
group_variable_values = group.get_current_joint_values()
group_variable_values[0] = -0.3491
# group_variable_values[1] = 0.8028
# group_variable_values[2] = 0.4014
# group_variable_values[3] = -1.2043
group.set_joint_value_target(group_variable_values)
plan1 = group.plan()
rospy.sleep(1)
group.go(wait=True)
rospy.sleep(1)

while(cyl_flag == 0 ):
    try:
        cyl_flag = listener.frameExists("object_29")
    except (tf.LookupException, tf.ConnectivityException, tf.ExtrapolationException):
        print('Cant get frame of cylindrical shaft, retrying...')

print('Found cylindrical shaft!')
co.add_coll_obj()

listener.waitForTransform("/base_footprint", "/object_29", rospy.Time(), rospy.Duration(4.0))
(trans_2, rot_2) = listener.lookupTransform('base_footprint', 'object_29', rospy.Time())
goal_coord_2 = [trans_2[0], trans_2[1], trans_2[2]] # x, y, z



if rect_flag == 1:

    print "============ Press `Enter` for arm repositioning..."
    raw_input()
    #(Set Pre-defined Pose)
    group.set_named_target("BACK")
    plan1 = group.plan()
    rospy.sleep(2)
    group.go(wait=True)
    rospy.sleep(2)

    # IK (Set Position) - Update end-effector XYZ coordinates to target object
    group.set_path_constraints(group.goal_c)
    print "============ Press `Enter` to move end-effector to target object by IK..."
    raw_input()
    target_position = [trans[0]-0.15, trans[1], trans[2]+0.065]
    group.set_position_target(target_position)
    plan1 = group.plan()
    rospy.sleep(10)
    group.go(wait=True)
    rospy.sleep(10)

    #FK (Set Joint Value) - Close the gripper 
    print "============ Press `Enter` to close the gripper and pick object..."
    raw_input()
    group_h_variable_values = group_h.get_current_joint_values()

    group_h_variable_values[0] -= 0.0174533   # approx -1.0 degrees
    plan2 = group_h.plan(group_h_variable_values)
    fail_count = 0
    rospy.sleep(1)
    while plan2.joint_trajectory.points:  # True if trajectory contains points
        print "Close by decrementing step angle of left finger..."
        group_h.go(wait=True)
        rospy.sleep(1)
        if group_h_variable_values[0] > -0.3142:  #Within joint limit
            group_h_variable_values[0] -= 0.0174533   # approx -1.0 degrees
        else:
            break
        plan2 = group_h.plan(group_h_variable_values)

    print "Left finger in contact with object!"
    group_h_variable_values = group_h.get_current_joint_values()
    rospy.sleep(2)

    group_h_variable_values[1] += 0.0174533   # approx +1.0 degrees
    plan2 = group_h.plan(group_h_variable_values)
    rospy.sleep(1)
    while plan2.joint_trajectory.points:  # True if trajectory contains points
        print "Close by decrementing step angle of right finger..."
        group_h.go(wait=True)
        rospy.sleep(1)
        if group_h_variable_values[1] +0.0174533 < 0.3142:  #Within joint limit
            group_h_variable_values[1] += 0.0174533   # approx +1.0 degreess
        else:
            break
        plan2 = group_h.plan(group_h_variable_values)

    print "Right finger in contact with object!"
    rospy.sleep(2)
    group_h_variable_values = group_h.get_current_joint_values()


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
    print "============ ..."
    raw_input()
    #FK (Set Joint Value)
    group_variable_values = group.get_current_joint_values()
    group_variable_values[0] = -1.45 #Rotate CW 90 degrees
    group.set_joint_value_target(group_variable_values)
    plan1 = group.plan()
    group.go(wait=True)
    print "============ ..."
    raw_input()
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



if cyl_flag == 1:

#     #(Set Pre-defined Pose)
    print "============ Press `Enter` to prepare next sensing for manipulation..."
    raw_input()
    group.set_named_target("BACK")
    plan1 = group.plan()
    rospy.sleep(2)
    group.go(wait=True)
    rospy.sleep(2)

    # IK (Set Position) - Update end-effector XYZ coordinates to target object
    group.set_path_constraints(group.goal_c)
    print "============ Press `Enter` to move end-effector to target object by IK..."
    raw_input()
    target_position = [trans_2[0]-0.147, trans_2[1]+ 0.05, trans_2[2]+0.062]
    group.set_position_target(target_position)
    plan1 = group.plan()
    rospy.sleep(10)
    group.go(wait=True)
    rospy.sleep(10)


    #FK (Set Joint Value) - Close the gripper 
    print "============ Press `Enter` to close the gripper and pick object..."
    raw_input()
    group_h_variable_values = group_h.get_current_joint_values()

    group_h_variable_values[0] -= 0.0174533   # approx -1.0 degrees
    plan2 = group_h.plan(group_h_variable_values)
    rospy.sleep(1)

    while plan2.joint_trajectory.points:  # True if trajectory contains points
        print "Close by decrementing step angle of left finger..."
        group_h.go(wait=True)
        rospy.sleep(1)
        if group_h_variable_values[0] -0.0174533 > -0.3142:  #Within joint limit
            group_h_variable_values[0] -= 0.0174533   # approx -1.0 degrees
        else:
            break
        plan2 = group_h.plan(group_h_variable_values)

    print "Left finger in contact with object!"
    group_h_variable_values = group_h.get_current_joint_values()
    rospy.sleep(2)

    group_h_variable_values[1] += 0.0174533   # approx +1.0 degrees
    plan2 = group_h.plan(group_h_variable_values)
    rospy.sleep(1)
    while plan2.joint_trajectory.points:  # True if trajectory contains points
        print "Close by decrementing step angle of right finger..."
        group_h.go(wait=True)
        rospy.sleep(1)
        if group_h_variable_values[1] +0.0174533 < 0.3142:  #Within joint limit
            group_h_variable_values[1] += 0.0174533   # approx +1.0 degreess
        else:
            break
        plan2 = group_h.plan(group_h_variable_values)

    print "Right finger in contact with object!"
    group_h_variable_values = group_h.get_current_joint_values()
    rospy.sleep(2)


    att_dett.attach_cylin()

    #(Set Pre-defined Pose)
    group.set_path_constraints(None)
    print "============ Press `Enter` to adjust arm pose for navigation..."
    raw_input()
    group.set_named_target("BACK")
    plan1 = group.plan()
    rospy.sleep(2)
    group.go(wait=True)
    rospy.sleep(2)

    #FK (Set Joint Value) - Extend the arm toward table at Station 2 
    group.set_path_constraints(group.goal_c)
    print "============ Press `Enter` to extend the arm toward table at Station 2..."
    raw_input()
    group_variable_values = group.get_current_joint_values()
    group_variable_values[0] = -0.2094   # 0 degrees
    group_variable_values[1] = 0.4363   # 25 degrees
    group_variable_values[2] = -0.4363   # -25 degrees
    group_variable_values[3] = 0.0   # 0 degrees
    group.set_joint_value_target(group_variable_values)
    plan2 = group.plan()
    group.go(wait=True)
    rospy.sleep(2)

    att_dett.dettach_cylin()

    #(Set Pre-defined Pose) - Open the gripper
    print "============ Press `Enter` to open the gripper and release object..."
    raw_input()
    group_h.set_named_target("OPEN")
    plan2 = group_h.plan()
    rospy.sleep(2)
    group_h.go(wait=True)
    rospy.sleep(2)
    group.set_path_constraints(None)

    #(Set Pre-defined Pose)
    print "============ Press `Enter` to retract the arm..."
    raw_input()
    group.set_named_target("BACK")
    plan1 = group.plan()
    rospy.sleep(2)
    group.go(wait=True)
    rospy.sleep(2)

    #(Set Pre-defined Pose)
    print "============ Press `Enter` to pick-up object from right pocket..."
    raw_input()
    group.set_named_target("RIGHT")
    plan1 = group.plan()
    rospy.sleep(2)
    group.go(wait=True)
    rospy.sleep(2)

    #FK (Set Joint Value) - Close the gripper 
    print "============ Press `Enter` to close the gripper..."
    raw_input()
    group_h_variable_values = group_h.get_current_joint_values()

    group_h_variable_values[0] -= 0.0174533   # approx -1.0 degrees
    plan2 = group_h.plan(group_h_variable_values)
    fail_count = 0
    rospy.sleep(1)
    while plan2.joint_trajectory.points:  # True if trajectory contains points
        print "Close by decrementing step angle of left finger..."
        group_h.go(wait=True)
        rospy.sleep(1)
        if group_h_variable_values[0] > -0.3142:  #Within joint limit
            group_h_variable_values[0] -= 0.0174533   # approx -1.0 degrees
        else:
            break
        plan2 = group_h.plan(group_h_variable_values)

    print "Left finger in contact with object!"
    group_h_variable_values = group_h.get_current_joint_values()
    rospy.sleep(2)

    group_h_variable_values[1] += 0.0174533   # approx +1.0 degrees
    plan2 = group_h.plan(group_h_variable_values)
    rospy.sleep(1)
    while plan2.joint_trajectory.points:  # True if trajectory contains points
        print "Close by decrementing step angle of right finger..."
        group_h.go(wait=True)
        rospy.sleep(1)
        if group_h_variable_values[1] +0.0174533 < 0.3142:  #Within joint limit
            group_h_variable_values[1] += 0.0174533   # approx +1.0 degreess
        else:
            break
        plan2 = group_h.plan(group_h_variable_values)

    print "Right finger in contact with object!"
    rospy.sleep(2)
    group_h_variable_values = group_h.get_current_joint_values()


    #(Set Pre-defined Pose) - Place at right pocket
    att_dett.attach_box()

    #(Set Pre-defined Pose)
    group.set_path_constraints(None)
    print "============ Press `Enter` to adjust arm pose for next manipulation..."
    raw_input()
    group.set_named_target("BACK")
    plan1 = group.plan()
    rospy.sleep(2)
    group.go(wait=True)
    rospy.sleep(2)

    #FK (Set Joint Value) - Extend the arm toward table at Station 2 
    group.set_path_constraints(group.goal_c)
    print "============ Press `Enter` to extend the arm toward table at Station 3..."
    raw_input()
    group_variable_values = group.get_current_joint_values()
    group_variable_values[0] = -0.2094   # 0 degrees
    group_variable_values[1] = 0.4363   # 25 degrees
    group_variable_values[2] = -0.4363   # -25 degrees
    group_variable_values[3] = 0.0   # 0 degrees
    group.set_joint_value_target(group_variable_values)
    plan2 = group.plan()
    group.go(wait=True)
    rospy.sleep(2)

    att_dett.dettach_box()

    #(Set Pre-defined Pose) - Open the gripper
    print "============ Press `Enter` to open the gripper and release object..."
    raw_input()
    group_h.set_named_target("OPEN")
    plan2 = group_h.plan()
    rospy.sleep(2)
    group_h.go(wait=True)
    rospy.sleep(2)

    #(Set Pre-defined Pose)
    print "============ Press `Enter` to retract the arm..."
    raw_input()
    group.set_named_target("TOP")
    plan1 = group.plan()
    rospy.sleep(2)
    group.go(wait=True)
    rospy.sleep(2)

    att_dett.remove_box()

    #(Set Pre-defined Pose)
    print "============ Press `Enter` to move arm back to the HOME pose..."
    raw_input()
    group.set_named_target("HOME")
    plan1 = group.plan()
    rospy.sleep(2)
    group.go(wait=True)
    rospy.sleep(2)


moveit_commander.roscpp_shutdown()
