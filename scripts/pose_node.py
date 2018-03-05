#!/usr/bin/env python

import rospy
from ar_track_alvar_msgs.msg import AlvarMarkers, AlvarMarker
from geometry_msgs.msg import PoseStamped, Pose, Pose2D, Point, Quaternion
from std_msgs.msg import Header
from math import atan2, cos, sin


class PoseNode:
	"""Class publishes positions of marker"""
	
	def pose_callback(self, data):

		"""
		:param markers : AlvarMarkers 
		:param marker.pose : PoseStamped
		:param:marker.pose.pose : Pose  
		"""
		
		ar_markers = data.markers
		for marker in ar_markers: 
			ID = marker.id					
			pose = marker.pose.pose    		
			
			point2D_camkoord = Pose2D(pose.position.x, pose.position.y, self.quat_to_angle(pose) )
			point2D = self.tf( point2D_camkoord )
			
			if(ID == 0):
				self.pub0.publish(point2D)
				self.pub0_3D.publish(pose)

			if(ID == 1):
				self.pose0 = point2D_camkoord
				self.pub1.publish(point2D)
				self.pub1_3D.publish(pose)

			if(ID == 2):
				self.pub2.publish(point2D)
				self.pub2_3D.publish(pose)

			if(ID == 15):
				self.pub15.publish(point2D)
				self.pub15_3D.publish(pose)

			if(ID == 4):
				self.pub4.publish(point2D)
				self.pub4_3D.publish(pose)
		
	def __init__(self):
		rospy.Subscriber("/ar_pose_marker", AlvarMarkers, self.pose_callback)
		self.pub0 = rospy.Publisher('/pose_node/pose_id0', Pose2D, queue_size = 1)
		self.pub1 = rospy.Publisher('/pose_node/pose_id1', Pose2D, queue_size = 1)
		self.pub2 = rospy.Publisher('/pose_node/pose_id2', Pose2D, queue_size = 1)
		self.pub15 = rospy.Publisher('/pose_node/pose_id15', Pose2D, queue_size = 1)
		self.pub4 = rospy.Publisher('/pose_node/pose_id4', Pose2D, queue_size = 1)
		
		self.pub0_3D = rospy.Publisher('/pose_node/pose3D_id0', Pose, queue_size = 1)
		self.pub1_3D = rospy.Publisher('/pose_node/pose3D_id1', Pose, queue_size = 1)
		self.pub2_3D = rospy.Publisher('/pose_node/pose3D_id2', Pose, queue_size = 1)
		self.pub15_3D = rospy.Publisher('/pose_node/pose3D_id15', Pose, queue_size = 1)
		self.pub4_3D = rospy.Publisher('/pose_node/pose3D_id4', Pose, queue_size = 1)
		
		self.pose0 = Pose2D(0, 0, 0)
			
		
	def quat_to_angle(self, pose):
		q = pose.orientation
		angle = atan2( 2* (q.w*q.z + q.x*q.y ), 1 - 2* (q.y*q.y + q.z*q.z) )
		return angle * 360 / (2*3.14159)
	
	def tf(self, cam):
		fi = self.pose0.theta * (-1) * 2*3.14159 / 360
		camx = cam.x - self.pose0.x
		camy = cam.y - self.pose0.y
		x = camx * cos(fi) - camy * sin(fi)
		y = camx * sin(fi) + camy * cos(fi)
		theta = cam.theta - self.pose0.theta
		return Pose2D(x, y, theta)


if __name__ == '__main__':
	rospy.init_node('PoseNode')
	try:
		pose_node = PoseNode()
		rospy.spin()
	except rospy.ROSInterruptException: pass

