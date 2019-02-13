#!/usr/bin/env python
import rospy
import numpy as np
from scipy.cluster.vq import vq, kmeans, whiten
#import matplotlib.pyplot as plt
from geometry_msgs.msg import PoseArray, Pose, Point, PoseStamped
from sensor_msgs.msg import LaserScan
from nav_msgs.msg import OccupancyGrid
#from semaforr.msg import CrowdModel
from visualization_msgs.msg import Marker
from visualization_msgs.msg import MarkerArray
from std_msgs.msg import String, Header
import itertools
import tf
from math import floor, sin, cos, atan2
import sys
#from cusum import Cusum

class SituationModel:
	def __init__(self, num):
		# set up this python class as ros node and initialize publisher and subscriber
		rospy.init_node('situation_model')
		rospy.Subscriber("decision_laser", LaserScan, self.laser_data)
		rospy.Subscriber("decision_pose", PoseStamped, self.pose_data)
		self.pub_situations = rospy.Publisher('situations', String, queue_size=1)

		# robot current pose is x,y,theta
		self.robot_pose = [0,0,0]
		self.laser_scan_endpoints = []
		self.laser_scan = []
		self.num_clusters = num

	# calls the callback for each of the subscriber
	def listen(self):
		rospy.spin()

	# receive the laser data in standard format and save as a sequence of endpoints
	def laser_data(self, data):
		print "receiving laser_scan data message"
		self.laser_scan.append(data.ranges)
		endpoints = []
		angle = data.angle_min
		increment = data.angle_increment
		for laser_distance in data.ranges:
			x = self.robot_pose[0] + laser_distance * cos(self.robot_pose[2] + angle)
			y = self.robot_pose[1] + laser_distance * sin(self.robot_pose[2] + angle)
			angle += increment
			endpoints.append([x,y])
		self.laser_scan_endpoints.append(endpoints)
		self.publish_situations()

	# receive and save the robot pose data
	def pose_data(self, data):
		print "receiving robot pose data message"
		quaternion = (data.pose.orientation.x, data.pose.orientation.y, data.pose.orientation.z, data.pose.orientation.w)
		euler = tf.transformations.euler_from_quaternion(quaternion)
		roll = euler[0]
		pitch = euler[1]
		yaw = euler[2]
		self.robot_pose[0] = data.pose.position.x
		self.robot_pose[1] = data.pose.position.y
		self.robot_pose[2] = yaw

	def publish_situations(self):
		if len(self.laser_scan) > self.num_clusters:
			features = np.array(self.laser_scan)
			whitened = whiten(features)
			centroids, distortion = kmeans(whitened,self.num_clusters)
			print centroids
			print distortion
			clx,_ = vq(whitened,centroids)
			print clx
			situation = str(centroids)
			self.pub_situations.publish(situation)

situation_model = SituationModel(5)
situation_model.listen()
