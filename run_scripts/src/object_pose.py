
#! /usr/bin/env python
import rospy
from geometry_msgs.msg import Pose, PointStamped, PoseArray
from visualization_msgs.msg import MarkerArray
from math import sqrt


class object_pose:

	def compute_center(self, point_array, header):
		mean_x = 0.0; mean_y = 0.0; mean_z = 0.0; 
		length = len(point_array)
		for p in point_array:
			mean_x += p.x 
			mean_y += p.y 
			mean_z += p.z
		mean_x /= length
		mean_y /= length
		mean_z /= length

		if (sqrt((mean_x**2) + (mean_y**2) + (mean_z**2))) < 1.3:
			po = PointStamped()
			po.header = header
			po.point.x = mean_x
			po.point.y = mean_y
			po.point.z = mean_z

			self.point_pub.publish(po)
			return po



	def viz_array_callback(self, data):
		if len(data.markers) > 0:
			poses = []
			for marker in data.markers:
				po = self.compute_center(marker.points, marker.header)
				if po is not None:
					pose = Pose()
					pose.position = po.point
					poses.append(pose)
			if len(poses) != 0:
				posearray = PoseArray()
				for p in poses:
					posearray.poses.append(p)
				self.posearray_pub.publish(posearray)
				





	def __init__(self):
		rospy.init_node('get_marker_pose')
		rospy.Subscriber('/tabletop/clusters', MarkerArray, self.viz_array_callback)
		self.point_pub = rospy.Publisher('/obs', PointStamped, queue_size=20)
		self.posearray_pub = rospy.Publisher('/object_poses', PoseArray, queue_size=20)
		rospy.spin()


if __name__=='__main__':
	try:
		go = object_pose()
	except:
		pass