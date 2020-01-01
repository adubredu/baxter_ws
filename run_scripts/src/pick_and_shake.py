#! /usr/bin/env python
import rospy
import sys
import baxter_interface
import geometry_msgs.msg
from geometry_msgs.msg import PoseStamped, PoseArray
from positionControl import *
import time
import tf2_ros
import tf2_geometry_msgs
import tf
from geometry_msgs.msg import Twist
from std_msgs.msg import Float32
import numpy as np



class Pick_and_shake:

	def object_poses_callback(self, data):
		if len(data.poses) > 0:
			self.object_poses = data.poses 

	# def joy_callback(self,data):

	def transform_object_pose_to_robot_rf(self, object_pose):
		#kinect camera axis not the same as the robot axis so we could have
		#to perform the necessary transforms first to get both axes aligned
		#and then to transform camera rf to robot's rf
		#goal_pose is the final pose of the marker wrt the robot's rf

		tf_buffer = tf2_ros.Buffer(rospy.Duration(1200.0))
		tf_listener = tf2_ros.TransformListener(tf_buffer)

		transform = tf_buffer.lookup_transform('base', 'camera_link',rospy.Time(0),
			rospy.Duration(1.0))
		trans_pose = tf2_geometry_msgs.do_transform_pose(object_pose, transform)

		return trans_pose


	def grasp_object(self, arm):
		if len(self.object_poses) > 0:
			ind = np.random.randint(low=0, high=len(self.object_poses))
			to_pick = self.object_poses[ind]
			p_stamp = PoseStamped()
			p_stamp.pose = to_pick
			pick_pose = self.transform_object_pose_to_robot_rf(p_stamp)

			#gripper orientation
			current_pose = self.lLimb.endpoint_pose()
			pick_pose.pose.orientation.x = current_pose['orientation'][0]
			pick_pose.pose.orientation.y = current_pose['orientation'][1]
			pick_pose.pose.orientation.z = current_pose['orientation'][2]
			pick_pose.pose.orientation.w = current_pose['orientation'][3]

			if arm == 'left':
				move_to_goal_pose(self.left_arm, pick_pose, self.pause_event)

			elif arm == 'right':
				move_to_goal_pose(self.right_arm, pick_pose, self.pause_event)



	def move_to_init(self):
		both_arms_follow_trajectory('waypoints/left_move_to_init.wp', 'waypoints/right_move_to_init.wp', 
			self.left_arm, self.right_arm, self.pause_event)



	def move_to_sleep(self):
		both_arms_follow_trajectory('waypoints/left_move_to_sleep.wp', 'waypoints/right_move_to_sleep.wp', 
			self.left_arm, self.right_arm, self.pause_event)


	def perform_grasp(self, arm):
		self.grasp_object(arm)
		# self.shake(arm)
		# self.place_back(arm)



	def __init__(self):
		rospy.init_node('pick_n_shake', disable_signals=True)
		self.place = 0
		self.baxter_enabler = baxter.RobotEnable(versioned=True)
		self.baxter_enabler.enable()

		self.left_arm = baxter.Limb('left')
		self.right_arm = baxter.Limb('right')
		self.lGripper = baxter.Gripper('left')
		self.rGripper = baxter.Gripper('right')

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

		rospy.Subscriber('/object_poses', PoseArray, self.object_poses_callback)

		self.head = baxter.Head()
		#self.head.set_pan(1.57)

		self.pause_event = Event()
		self.object_poses = None

		self.move_to_init()
		self.perform_grasp('left')
		
		

		#print self.lLimb.endpoint_pose()
		#print self.lLimb.joint_angles()
		
		
		rospy.spin()
				

if __name__=="__main__":
	
	try:
		go = Pick_and_shake()


	except rospy.ROSInterruptException: pass

