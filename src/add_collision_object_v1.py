#! /usr/bin/env python

#Add collision object into planning scene in react to the find_object pkg

import sys
import copy
import rospy
import tf
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

# print "============ Press `Enter` to update table to the planning scene ..."
# raw_input()

# # Add grasping object
# rospy.sleep(2)
# box_pose = geometry_msgs.msg.PoseStamped()
# box_pose.header.frame_id = "Table"
# box_pose.pose.orientation.w = 1.0
# box_pose.pose.position.x = 0.2873/2 
# box_pose.pose.position.y = 1.0/2 
# box_pose.pose.position.z = 0.3269/2 
# box_name = "box"
# scene.add_box(box_name, box_pose, size=(0.2873, 1.0, 0.3269))

# Add table
rospy.sleep(2)
table_pose = geometry_msgs.msg.PoseStamped()
table_pose.header.frame_id = "Base"
table_pose.pose.orientation.w = 1.0
table_pose.pose.position.x = 0.34 # slightly above the end effector
table_pose.pose.position.y = 0.0 # slightly above the end effector
table_pose.pose.position.z = 0.12 # slightly above the end effector
table_name = "table"
scene.add_box(table_name, table_pose, size=(0.3, 0.5, 0.08))

# Subscribe to tf frame pose
listener = tf.TransformListener()
rospy.sleep(2)

rect_flag = 0
cyl_flag = 0
sphe_flag = 0

rect_flag = listener.frameExists("object_28")
cyl_flag = listener.frameExists("object_29")
sphe_flag = listener.frameExists("object_30")

if rect_flag == 1:

    listener.waitForTransform("/base_footprint", "/object_28", rospy.Time(), rospy.Duration(4.0))
    (trans, rot) = listener.lookupTransform('base_footprint', 'object_28', rospy.Time())

    goal_coord = [trans[0], trans[1], trans[2]] # x, y, z

    print "============ Press `Enter` to add a wooden block to the planning scene ..."
    raw_input()

    # Add grasping object - BOX
    rospy.sleep(2)
    box_pose = geometry_msgs.msg.PoseStamped()
    box_pose.header.frame_id = "base_footprint"
    box_pose.pose.orientation.w = 1.0
    box_pose.pose.position.x = trans[0] -0.0274-0.01
    box_pose.pose.position.y = trans[1] 
    box_pose.pose.position.z = trans[2] +0.012

    print (box_pose)

    box_name = "box"
    scene.add_box(box_name, box_pose, size=(0.0274 *0.85, 0.0187*0.85, 0.0678))

elif cyl_flag == 1:

    listener.waitForTransform("/base_footprint", "/object_29", rospy.Time(), rospy.Duration(4.0))
    (trans, rot) = listener.lookupTransform('base_footprint', 'object_29', rospy.Time())

    goal_coord = [trans[0], trans[1], trans[2]] # x, y, z

    print "============ Press `Enter` to add a cylindrical shaft to the planning scene ..."
    raw_input()

    # # Add grasping object - CYLINDER
    rospy.sleep(2)
    cylinder_pose = geometry_msgs.msg.PoseStamped()
    cylinder_pose.header.frame_id = "base_footprint"
    cylinder_pose.pose.orientation.w = 1.0
    cylinder_pose.pose.position.x = trans[0] -0.02-0.01
    cylinder_pose.pose.position.y = trans[1] 
    cylinder_pose.pose.position.z = trans[2] +0.012
    cylinder_name = "cylinder"
    scene.add_cylinder(cylinder_name, cylinder_pose, height=0.065, radius=0.010)

elif sphe_flag == 1:

    listener.waitForTransform("/base_footprint", "/object_30", rospy.Time(), rospy.Duration(4.0))
    (trans, rot) = listener.lookupTransform('base_footprint', 'object_30', rospy.Time())

    goal_coord = [trans[0], trans[1], trans[2]] # x, y, z

    print "============ Press `Enter` to add a ball to the planning scene ..."
    raw_input()

    # Add grasping object - SPHERE
    rospy.sleep(2)
    sphere_pose = geometry_msgs.msg.PoseStamped()
    sphere_pose.header.frame_id = "base_link"
    sphere_pose.pose.orientation.w = 1.0
    sphere_pose.pose.position.x = trans[0] -0.03-0.01
    sphere_pose.pose.position.y = trans[1]
    sphere_pose.pose.position.z = trans[2] +0.012
    sphere_name = "sphere"
    scene.add_sphere(sphere_name, sphere_pose, radius=0.015)
    #scene.set_object_color(sphere_name, 200, 0, 0, a=0.9)


# print "============ Press `Enter` to attach grasping object to the eff ..."
# raw_input()

# #Attach grasping object
# grasping_group = 'arm'
# touch_links = robot.get_link_names(group=grasping_group)
# scene.attach_box(eef_link, "cylinder", touch_links=touch_links)
# rospy.sleep(5)

# #Detach grasping object
# scene.remove_attached_object(eef_link, "cylinder")
# rospy.sleep(5)


moveit_commander.roscpp_shutdown()




