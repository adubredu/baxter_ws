#! /usr/bin/env python
import rospy
import sys
import baxter_interface
import geometry_msgs.msg
from geometry_msgs.msg import PoseStamped
from positionControl import *
import time
import tf2_ros
import tf2_geometry_msgs
import tf
from geometry_msgs.msg import Twist
from std_msgs.msg import Float32

class pick_object:

	def orientation_callback(self,angle):
		self.object_orientation = angle.data 


	def pose_callback(self, pose):
		self.object_pose = pose
		#self.grab_object()


	def transform_object_pose_to_robot_rf(self):
		#kinect camera axis not the same as the robot axis so we could have
		#to perform the necessary transforms first to get both axes aligned
		#and then to transform camera rf to robot's rf
		#goal_pose is the final pose of the marker wrt the robot's rf

		tf_buffer = tf2_ros.Buffer(rospy.Duration(1200.0))
		tf_listener = tf2_ros.TransformListener(tf_buffer)

		transform = tf_buffer.lookup_transform('base', 'camera_link',rospy.Time(0),
			rospy.Duration(1.0))
		trans_pose = tf2_geometry_msgs.do_transform_pose(self.object_pose, transform)

		return trans_pose



	def grab_object(self):
		pose = self.transform_object_pose_to_robot_rf()

		gpose = get_pick_from_floor_pose(pose)
		self.orient_gripper()
		time.sleep(2)
		current_pose = self.lLimb.endpoint_pose()
		# print current_pose
		gpose.pose.orientation.x = current_pose['orientation'][0]
		gpose.pose.orientation.y = current_pose['orientation'][1]
		gpose.pose.orientation.z = current_pose['orientation'][2]
		gpose.pose.orientation.w = current_pose['orientation'][3]
		#
		# print pose
		#print gpose
		move_to_goal_pose(self.lLimb, gpose, self.pause_event)


	def orient_gripper(self):
		angle = self.map_angle_to_wrist()
		print('angle is',angle)
		playPositionFile('waypoints/picking_mode.wp', self.lLimb, self.rLimb, self.pause_event)
		ang = self.lLimb.joint_angles()
		ang['left_w2'] = angle
		move_to_goal_joint_angle(self.lLimb, ang, self.pause_event)


	def move_to_init(self):
		both_arms_follow_trajectory('waypoints/left_move_to_init.wp', 'waypoints/right_move_to_init.wp', 
			self.lLimb, self.rLimb, self.pause_event)
		# right_arm_follow_trajectory( 'waypoints/right_move_to_init.wp', self.rLimb, self.pause_event)
		time.sleep(5)
		self.move_to_sleep()


	def move_to_sleep(self):
		both_arms_follow_trajectory('waypoints/left_move_to_sleep.wp', 'waypoints/right_move_to_sleep.wp', 
			self.lLimb, self.rLimb, self.pause_event)
		# right_arm_follow_trajectory( 'waypoints/right_move_to_sleep.wp', self.rLimb, self.pause_event)


	


	def __init__(self):
		rospy.init_node('pick_object', disable_signals=True)
		self.place = 0
		self.baxter_enabler = baxter.RobotEnable(versioned=True)
		self.baxter_enabler.enable()

		self.lLimb = baxter.Limb('left')
		self.rLimb = baxter.Limb('right')
		self.lGripper = baxter.Gripper('left')
		self.rGripper = baxter.Gripper('right')
		# print self.lLimb.endpoint_pose()

		#calibrating gripper
		if not self.lGripper.calibrate():
		    print("left gripper did not calibrate")
		    sys.exit()
		if not self.rGripper.calibrate():
		    print("right gripper did not calibrate")
		    sys.exit()

		self.lGripper.set_holding_force(100)
		self.lGripper.set_moving_force(100)

		self.rGripper.set_holding_force(100)
		self.rGripper.set_moving_force(100)
		# self.lGripper.open()
		#ang = self.lLimb.joint_angles()
		# print self.lLimb.endpoint_pose()
		#ang['left_w2'] = -0.75
		#move_to_goal_joint_angle(self.lLimb, ang, self.pause_event)

		

		self.head = baxter.Head()
		#self.head.set_pan(1.57)

		self.object_orientation = None
		self.object_pose = None
		self.pause_event = Event()

		#move_to_goal_joint_angle(self.lLimb, self.origin, self.pause_event)
		'''
		rospy.Subscriber('/floor_object/pose', PoseStamped, self.pose_callback)
		rospy.Subscriber('/floor_object/angle', Float32, self.orientation_callback)
		'''
		self.move_to_sleep()
		
		

		#print self.lLimb.endpoint_pose()
		#print self.lLimb.joint_angles()
		
		
		rospy.spin()
		

if __name__=="__main__":
	
	try:
		go = pick_object()
		# go.move_to_init()


	except rospy.ROSInterruptException: pass
	# if rospy.is_shutdown():
	# 		go.move_to_sleep()