#!/usr/bin/env python
from threading import current_thread
import rospy
import tf
import tf2_ros
import math
import actionlib
from actionlib_msgs.msg import *
from geometry_msgs.msg import Pose, PoseWithCovarianceStamped, Point, Quaternion, Twist
from move_base_msgs.msg import MoveBaseAction, MoveBaseGoal
from tf.transformations import quaternion_from_euler, euler_from_quaternion
import tf2_msgs.msg
import geometry_msgs.msg
import std_msgs.msg 
from std_srvs.srv import Empty

class move_base:

    def __init__(self):
        self.move_base = actionlib.SimpleActionClient("move_base", MoveBaseAction)
        rospy.loginfo("Waiting for move_base action server...")
        self.move_base.wait_for_server(rospy.Duration(120))
        self.listener = tf.TransformListener()
        rospy.loginfo("Connected to move base server")
        self.locations = dict()
        self.r = rospy.Rate(10)
        self.step = 0 
        self.mv_state = False
        self.parent_frame = 'map'

    def sendGoal(self, trans, rot):
        
        self.locations['goal'] = Pose(Point(trans[0],trans[1],0), Quaternion(rot[0], rot[1], rot[2], rot[3]))
        self.goal = MoveBaseGoal()
        self.goal.target_pose.header.frame_id = self.parent_frame
        self.goal.target_pose.header.stamp = rospy.Time.now()
        self.goal.target_pose.pose = self.locations['goal']
        self.move_base.send_goal(self.goal)
        self.step = self.step + 1

    def checkGoal(self):

        rospy.wait_for_service('/clear_octomap') #this will stop your code until the clear octomap service starts running
        clear_octomap = rospy.ServiceProxy('/clear_octomap', Empty)

        rospy.sleep(1.2)

        try:
            clear_octomap()
        except rospy.ServiceException as exc:
            print("Service did not process request: " + str(exc))

        if self.move_base.get_state() == actionlib.GoalStatus.SUCCEEDED:
            rospy.loginfo("reached goal")
            return True
        else:
            return False

    def move_to_goal(self, trans, rot):
        while not rospy.is_shutdown():
            try:
                if self.step == 0:
                    self.sendGoal(trans, rot)
                elif self.step ==1:
                    self.state = self.checkGoal()
                    if self.state:
                        self.step = 0
                        reached = True
                        break
                    elif not self.state:
                        continue
            except:
                pass
            self.r.sleep()
        return True
#Get the tf of a given frame            
    def get_tf(self, frame_id):
        try:
            (trans,rot) = self.listener.lookupTransform(self.parent_frame, frame_id, rospy.Time(0))
        except (tf.LookupException, tf.ConnectivityException, tf.ExtrapolationException):
            print('cant get frame')
        return (trans,rot)
#Get the x, y, w of a given frame wrt map frame            
    def get_pose(self, frame_id):
        try:
            (trans,rot) = self.listener.lookupTransform(self.parent_frame, frame_id, rospy.Time(0))
            angles = euler_from_quaternion(rot)
        except (tf.LookupException, tf.ConnectivityException, tf.ExtrapolationException):
            print('cant get frame')
        return (trans[0],trans[1],angles[2])

#Navigate by frame wrt map frame   
    def move_to_frame(self, frame_id):
        self.mv_state = False
        self.parent_frame = 'map'
        (trans, rot) = self.get_tf(frame_id)
        self.move_to_goal(trans, rot)
#Navigate by point wrt. map frame
    def move_to_point(self, x, y, w):
        self.mv_state = False
        self.trans = (x, y, 0)
        self.rot = quaternion_from_euler(0.0, 0.0, w)
        self.move_to_goal(self.trans, self.rot)
        
#Convert the referance frame of a tf
    def convert_TF(self,parent_frame,child_frame):
        while not rospy.is_shutdown():
            try:
                (trans, rot) = self.listener.lookupTransform(parent_frame,child_frame,rospy.Time(0))
                (roll, pitch, yaw) = euler_from_quaternion(rot)
                break
            except (tf.LookupException,tf.ConnectivityException, tf.ExtrapolationException) as e :
                rospy.logerr(e)
                continue
        return {'x':trans[0],'y':trans[1],'z':0,'roll':0,'pitch':0,'yaw':yaw}   

#Navigate by frame wrt parent frame   
    def move_to_frame_front(self, parent_frame, child_frame,dist):
        self.mv_state = False
        self.parent_frame = parent_frame
        Transform = []
        Transform = self.convert_TF(parent_frame, child_frame)
        self.move_to_point(Transform['x']+dist,Transform['y'],0)
        self.parent_frame = 'map'  

#simple move base without navigation planners   
    def no_planner(self, dist_x, vel_x, angle, rot_speed):
        vel_publisher = rospy.Publisher('/diff_drive_controller/cmd_vel', Twist, queue_size=10)

        #Translation
        vel_msg = Twist()
        vel_msg.linear.x = vel_x
        vel_msg.linear.y = 0
        vel_msg.linear.z = 0 
        vel_msg.angular.x = 0 
        vel_msg.angular.y = 0 
        vel_msg.angular.z = 0

        t0 = rospy.Time.now().to_sec()
        current_distance = 0
        distance = dist_x

        if distance != 0:
            while(current_distance < distance):
                vel_publisher.publish(vel_msg)
                t1 =rospy.Time.now().to_sec()
                current_distance = abs ( vel_msg.linear.x * (t1-t0) )
            
            # After the loop, force the robot to stop 
            vel_msg.linear.x = 0
            vel_publisher.publish(vel_msg)


        #Rotation
        vel_msg = Twist()
        vel_msg.linear.x = 0
        vel_msg.linear.y = 0
        vel_msg.linear.z = 0 
        vel_msg.angular.x = 0 
        vel_msg.angular.y = 0 
        vel_msg.angular.z = rot_speed

        t0 = rospy.Time.now().to_sec()
        current_angle = 0

        if angle != 0:
            while(current_angle < angle):
                vel_publisher.publish(vel_msg)
                t1 =rospy.Time.now().to_sec()
                current_angle = abs ( rot_speed * (t1-t0) )

            vel_msg.angular.z = 0
            vel_publisher.publish(vel_msg)

  

