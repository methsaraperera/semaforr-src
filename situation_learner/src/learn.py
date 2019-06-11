#!/usr/bin/env python
import rospy
import numpy as np
np.set_printoptions(threshold=np.inf)
import scipy.spatial as scp
import networkx as nx
from sklearn.cluster import KMeans
from sklearn.preprocessing import StandardScaler
from matplotlib import pyplot as plt
from geometry_msgs.msg import PoseArray, Pose, Point, PoseStamped
from sensor_msgs.msg import LaserScan
from visualization_msgs.msg import Marker
from visualization_msgs.msg import MarkerArray
from std_msgs.msg import String, Header
import re
import tf
from math import sqrt, floor, sin, cos, atan2

def dfun(u, v):
	dist = sqrt((u[1] - v[1])*(u[1] - v[1]))
	# print u[1], v[1], dist
	anglediff = u[0] - v[0]
	if anglediff > 3.141592654:
		anglediff = anglediff - (2 * 3.141592654)
	if anglediff < -3.141592654:
		anglediff = anglediff + (2 * 3.141592654)
	# print u[0], v[0], anglediff
	# return dist + abs(anglediff)
	total = 2
	if abs(anglediff) < 0.0872665:
		total = total - 1
	if dist < 0.5:
		total = total - 1
	return total

class SituationModel:
	def __init__(self, num):
		# set up this python class as ros node and initialize publisher and subscriber
		rospy.init_node('situation_model')
		rospy.Subscriber("decision_pose", PoseStamped, self.pose_data, queue_size=10000)
		rospy.Subscriber("decision_log", String, self.decision_data, queue_size=10000)
		rospy.Subscriber("decision_laser", LaserScan, self.laser_data, queue_size=10000)
		self.pub_situations = rospy.Publisher('situations', String, queue_size=10000)
		self.robot_pose = []
		self.laser_scan = []
		self.decisions = []
		self.trails = []
		self.laser_end_points = []
		self.data_num = 0
		self.num_clusters = num

	# calls the callback for each of the subscriber
	def listen(self):
		rospy.spin()

	# receive the laser data in standard format and save as a sequence of endpoints
	def laser_data(self, data):
		self.laser_scan.append(np.array(data.ranges).astype('float'))
		print "receiving laser_scan data message", len(self.laser_scan)
		# if len(self.laser_scan) % 50 == 0 and len(self.laser_scan) == len(self.robot_pose) and len(self.laser_scan) >= 50:
		# 	self.publish_situations()

	# receive and save the robot pose data
	def pose_data(self, data):
		quaternion = (data.pose.orientation.x, data.pose.orientation.y, data.pose.orientation.z, data.pose.orientation.w)
		euler = tf.transformations.euler_from_quaternion(quaternion)
		roll = euler[0]
		pitch = euler[1]
		yaw = euler[2]
		self.robot_pose.append([data.pose.position.x, data.pose.position.y, yaw])
		print "receiving robot pose data message", len(self.robot_pose)

	# receive and save the robot's decision log
	def decision_data(self, data):
		new_data = re.split(r'\t',data.data)
		new_data_values = np.array(new_data[0:11]).astype('float')
		self.decisions.append(new_data_values)
		new_data_trails = re.split(r';',new_data[19])
		# print new_data_trails
		new_data_trails = filter(None, new_data_trails)
		self.trails = []
		for trail in new_data_trails:
			# print trail
			trail_data = re.split(r' ',trail)
			trail_data = filter(None, trail_data)
			trail_data = np.array(trail_data).astype('float')
			parsed_trail_data = []
			for i in range(0,len(trail_data),2):
				parsed_trail_data.append([trail_data[i], trail_data[i+1]])
			self.trails.append(parsed_trail_data)
		# print self.trails
		new_data_lep = re.split(r';',new_data[26])
		# print new_data_lep
		new_data_lep = filter(None, new_data_lep)
		lep = []
		for pair in new_data_lep:
			lep.append(np.array(re.split(r',',pair)).astype('float'))
		self.laser_end_points.append(lep)
		# print lep
		print "receiving decision log data message", len(self.decisions)
		if len(self.decisions) > 1:
			if len(self.laser_scan) == len(self.robot_pose) and len(self.laser_scan) == len(self.decisions) and len(self.robot_pose) == len(self.decisions) and self.decisions[-1][0] != self.decisions[-2][0]:
				print len(self.laser_scan), len(self.robot_pose), len(self.decisions)
				self.data_num = len(self.laser_scan)-3
				self.publish_situations()

	def publish_situations(self):
		ActionsComp = []
		Actions = []
		# Moves = [0.1,0.2,0.3,0.4,0.5,0.6,0.7,0.8,0.9,1,1.1,1.2,1.3,1.4,1.5,1.6,1.7,1.8,1.9,2,2.1,2.2,2.3,2.4,2.5,2.6,2.7,2.8,2.9,3,3.1,3.2,3.3,3.4,3.5,3.6,3.7,3.8,3.9,4,4.1,4.2,4.3,4.4,4.5,4.6,4.7,4.8,4.9,5]
		# Moves = [0.25,0.5,0.75,1,1.25,1.5,1.75,2,2.25,2.5,2.75,3,3.25,3.5,3.75,4,4.25,4.5,4.75,5,5.25,5.5,5.75,6,6.25,6.5,6.75,7,7.25,7.5,7.75,8,8.25,8.5,8.75,9,9.25,9.5,9.75,10]
		# Moves = [0.1,0.2,0.3,0.4,0.5,0.6,0.7,0.8,0.9,1,1.1,1.2,1.3,1.4,1.5,1.6,1.7,1.8,1.9,2,2.1,2.2,2.3,2.4,2.5,2.6,2.7,2.8,2.9,3,3.1,3.2,3.3,3.4,3.5,3.6,3.7,3.8,3.9,4,4.1,4.2,4.3,4.4,4.5,4.6,4.7,4.8,4.9,5,5.1,5.2,5.3,5.4,5.5,5.6,5.7,5.8,5.9,6,6.1,6.2,6.3,6.4,6.5,6.6,6.7,6.8,6.9,7,7.1,7.2,7.3,7.4,7.5,7.6,7.7,7.8,7.9,8,8.1,8.2,8.3,8.4,8.5,8.6,8.7,8.8,8.9,9,9.1,9.2,9.3,9.4,9.5,9.6,9.7,9.8,9.9,10,10.1,10.2,10.3,10.4,10.5,10.6,10.7,10.8,10.9,11,11.1,11.2,11.3,11.4,11.5,11.6,11.7,11.8,11.9,12,12.1,12.2,12.3,12.4,12.5,12.6,12.7,12.8,12.9,13,13.1,13.2,13.3,13.4,13.5,13.6,13.7,13.8,13.9,14,14.1,14.2,14.3,14.4,14.5,14.6,14.7,14.8,14.9,15,15.1,15.2,15.3,15.4,15.5,15.6,15.7,15.8,15.9,16,16.1,16.2,16.3,16.4,16.5,16.6,16.7,16.8,16.9,17,17.1,17.2,17.3,17.4,17.5,17.6,17.7,17.8,17.9,18,18.1,18.2,18.3,18.4,18.5,18.6,18.7,18.8,18.9,19,19.1,19.2,19.3,19.4,19.5,19.6,19.7,19.8,19.9,20,20.1,20.2,20.3,20.4,20.5,20.6,20.7,20.8,20.9,21,21.1,21.2,21.3,21.4,21.5,21.6,21.7,21.8,21.9,22,22.1,22.2,22.3,22.4,22.5,22.6,22.7,22.8,22.9,23,23.1,23.2,23.3,23.4,23.5,23.6,23.7,23.8,23.9,24,24.1,24.2,24.3,24.4,24.5,24.6,24.7,24.8,24.9,25]
		# Moves = [0.1,0.2,0.3,0.4,0.5,0.6,0.7,0.8,0.9,1,1.1,1.2,1.3,1.4,1.5,1.6,1.7,1.8,1.9,2,2.1,2.2,2.3,2.4,2.5,2.6,2.7,2.8,2.9,3,3.1,3.2,3.3,3.4,3.5,3.6,3.7,3.8,3.9,4,4.1,4.2,4.3,4.4,4.5,4.6,4.7,4.8,4.9,5,5.1,5.2,5.3,5.4,5.5,5.6,5.7,5.8,5.9,6,6.1,6.2,6.3,6.4,6.5,6.6,6.7,6.8,6.9,7,7.1,7.2,7.3,7.4,7.5,7.6,7.7,7.8,7.9,8,8.1,8.2,8.3,8.4,8.5,8.6,8.7,8.8,8.9,9,9.1,9.2,9.3,9.4,9.5,9.6,9.7,9.8,9.9,10]
		# Moves = [0.5,1.0,1.5,2.0,2.5,3.0,3.5,4.0,4.5,5.0]
		Moves = [0.45, 1.2, 3.6, 7.6]
		LaserScans = np.array(self.laser_scan[0:self.data_num])
		for val in Moves:
			ActionsComp.append(LaserScans >= val)
		ActionsComp = np.array(ActionsComp)
		for i in range(0,len(LaserScans)):
			Actions.append(ActionsComp[:,i].flatten().astype('int'))
		# for scan in range(0,self.data_num,1):
		# 	Values = []
		# 	for dist in range(0,len(self.laser_scan[scan]),1):
		# 		for val in Moves:
		# 			if self.laser_scan[scan][dist] >= val:
		# 				Values.append(1)
		# 			else:
		# 				Values.append(0)
		# 	Actions.append(Values)
		# print Actions
		CityBlockDistances = scp.distance.pdist(Actions, 'cityblock')
		SimilarityMatrix = scp.distance.squareform(CityBlockDistances)
		Threshold = (float(len(Moves)) * float(len(self.laser_scan[0])))*0.03
		print "Threshold", Threshold
		SimilarityMatrixInt = (np.array(SimilarityMatrix) <= Threshold).astype(int)
		np.fill_diagonal(SimilarityMatrixInt, 0)
		adjacency_matrix = np.array(SimilarityMatrixInt)
		rows, cols = np.where(adjacency_matrix == 1)
		edges = zip(rows.tolist(), cols.tolist())
		gr = nx.Graph()
		gr.add_edges_from(edges)
		ConnectedComponents = sorted(nx.connected_components(gr), key = len, reverse=True)
		print "ConnectedComponents", ConnectedComponents
		FilteredComponents = []
		for component in ConnectedComponents:
			if len(component) >= 2:
				FilteredComponents.append(np.array(list(component)))
		ActionClusters = []
		for i in range(0,len(Actions)):
			cluster_found = 0
			for j in range(0,len(FilteredComponents)):
				if i in FilteredComponents[j]:
					ActionClusters.append(j)
					cluster_found = 1
			if cluster_found == 0:
				ActionClusters.append(-1)
		situation = 'ActionClusters_'+str(self.data_num)+' '+str(ActionClusters)
		print situation
		self.pub_situations.publish(situation)

		Targets = []
		for val in range(0, self.data_num,1):
			merger = []
			targetX = self.decisions[val][4]
			targetY = self.decisions[val][5]
			distToTarget = sqrt((self.robot_pose[val][0] - targetX) * (self.robot_pose[val][0] - targetX) + (self.robot_pose[val][1] - targetY) * (self.robot_pose[val][1] - targetY))
			merger.append(distToTarget)
			angleToTarget = atan2((self.robot_pose[val][1] - targetY), (self.robot_pose[val][0] - targetX))
			RequiredRotation = angleToTarget - self.robot_pose[val][2]
			if RequiredRotation > 3.141592654:
				RequiredRotation = RequiredRotation - (2 * 3.141592654)
			if RequiredRotation < -3.141592654:
				RequiredRotation = RequiredRotation + (2 * 3.141592654)
			merger.append(RequiredRotation)
			Targets.append(merger)
		Targets = np.array(Targets)
		# scaler = StandardScaler()
		# scaler.fit(Targets)
		# TargetsScaled = scaler.transform(Targets)
		# NumClusters = 20
		# if self.data_num<20:
		# 	NumClusters = self.data_num - 1
		# TargetClusters = KMeans(n_clusters=NumClusters, n_init=50, max_iter=1000).fit(TargetsScaled)
		# situation = 'TargetClusters_'+str(self.data_num)+' '+str(TargetClusters.labels_)
		CustomDistances = scp.distance.pdist(Targets, dfun)
		SimilarityMatrix = scp.distance.squareform(CustomDistances)
		SimilarityMatrixInt = (np.array(SimilarityMatrix) == 0).astype(int)
		np.fill_diagonal(SimilarityMatrixInt, 0)
		adjacency_matrix = np.array(SimilarityMatrixInt)
		rows, cols = np.where(adjacency_matrix == 1)
		edges = zip(rows.tolist(), cols.tolist())
		gr = nx.Graph()
		gr.add_edges_from(edges)
		cliques = sorted(list(nx.find_cliques(gr)), key = len, reverse=True)
		filtered_cliques = []
		while len(cliques) > 0:
			filtered_cliques.append(cliques[0])
			cliques.remove(cliques[0])
			cliques_to_remove = []
			for val in cliques:
				item_in_clique = False
				for item in filtered_cliques[-1]:
					if item in val:
						item_in_clique = True
				if item_in_clique == True:
					cliques_to_remove.append(val)
			for item in cliques_to_remove:
				cliques.remove(item)
		filtered_cliques = np.array(filtered_cliques)
		cluster_values = []
		for i in range(0,len(Targets)):
			cluster_found = 0
			for j in range(0,len(filtered_cliques)):
				if i in filtered_cliques[j]:
					cluster_values.append(j)
					cluster_found = 1
			if cluster_found == 0:
				cluster_values.append(-1)
		situation = 'TargetClusters_'+str(self.data_num)+' '+str(cluster_values)
		print situation
		self.pub_situations.publish(situation)

		TargetTrailMarker = []
		for i in range(0, self.data_num,1):
			# print len(self.trails), self.decisions[i][0]
			CurrentTrail = self.trails[int(self.decisions[i][0])]
			CurrentPose = self.robot_pose[i]
			CurrentLaser = self.laser_end_points[i]
			found = False
			for j in range(len(CurrentTrail)-1, -1, -1):
				if self.can_access_point(CurrentLaser, CurrentPose, CurrentTrail[j], 20):
					TargetTrailMarker.append(CurrentTrail[j])
					found = True
					# print i
					break
			if found == False:
				TargetTrailMarker.append([self.decisions[i][4], self.decisions[i][5]])
			# print i
		BestActions = []
		# print len(TargetTrailMarker), self.data_num
		for i in range(0, self.data_num,1):
			CurrentPose = self.robot_pose[i]
			# print i
			CurrentTrailMarker = TargetTrailMarker[i]
			distance_from_tm = sqrt((CurrentPose[0]-CurrentTrailMarker[0])*(CurrentPose[0]-CurrentTrailMarker[0])+(CurrentPose[1]-CurrentTrailMarker[1])*(CurrentPose[1]-CurrentTrailMarker[1]))
			distance_from_obstacle = 100000
			error_margin = 0.05
			start_view = int(-0.35/0.005817)+330
			end_view = int(0.35/0.005817)+330
			for angle in range(start_view, end_view+1):
				if self.laser_scan[i][angle] < distance_from_obstacle:
					distance_from_obstacle = self.laser_scan[i][angle]
			distance_from_obstacle = distance_from_obstacle - error_margin
			robot_direction = CurrentPose[2]
			goal_direction = atan2((CurrentTrailMarker[1] - CurrentPose[1]), (CurrentTrailMarker[0] - CurrentPose[0]))
			RequiredRotation = goal_direction - robot_direction
			if RequiredRotation > 3.141592654:
				RequiredRotation = RequiredRotation - (2 * 3.141592654)
			if RequiredRotation < -3.141592654:
				RequiredRotation = RequiredRotation + (2 * 3.141592654)
			ActionValues = []
			move = [0, 0.2, 0.4, 0.8, 1.6, 3.2]
			rotate = [0, 0.25, 0.5, 1, 2]
			numMoves = 6
			numRotates = 5
			rotIntensity = 0
			while (rotIntensity < numRotates and abs(RequiredRotation) > rotate[rotIntensity]):
				rotIntensity = rotIntensity + 1
			if rotIntensity > 1:
				if RequiredRotation < 0:
					ActionValues.append([1,rotIntensity-1])
				else:
					ActionValues.append([2,rotIntensity-1])
			else:
				ActionValues.append([0,0])
			intensity = 0
			while (intensity < numMoves and distance_from_tm > move[intensity]):
				intensity = intensity + 1
			obstacle_intensity = 0
			while (obstacle_intensity < numMoves and distance_from_obstacle > move[obstacle_intensity]):
				obstacle_intensity = obstacle_intensity + 1
			if intensity > obstacle_intensity:
				intensity = obstacle_intensity
			ActionValues.append([0,intensity-1])
			BestActions.append(ActionValues)
		situation = 'BestActions_'+str(self.data_num)+' '+str(BestActions)
		print situation
		self.pub_situations.publish(situation)

	def can_access_point(self, laser, pose, point, distance):
		canAccessPoint = False;
		distLaserPosToPoint = sqrt((pose[0]-point[0])*(pose[0]-point[0])+(pose[1]-point[1])*(pose[1]-point[1]))
		if distLaserPosToPoint > distance:
			return False
		point_direction = atan2((point[1] - pose[1]), (point[0] - pose[0]))
		index = 0
		min_angle = 100000

		for i in range(0, len(laser)):
			laser_direction = atan2((laser[i][1] - pose[1]), (laser[i][0] - pose[0]))
			if abs(laser_direction - point_direction) < min_angle:
				min_angle = abs(laser_direction - point_direction)
				index = i
		# print "index", index, len(laser)
		while index-2 < 0:
			index = index +1
		while index+2 > len(laser)-1:
			index = index -1
		# print "index", index, len(laser)
		num_free = 0
		for i in range(-2, 3):
			# print "index+i", index+i, len(laser)
			distLaserEndPointToLaserPos = sqrt((pose[0]-laser[index+i][0])*(pose[0]-laser[index+i][0])+(pose[1]-laser[index+i][1])*(pose[1]-laser[index+i][1]))
			# print laser[index+i]
			if distLaserEndPointToLaserPos > distLaserPosToPoint:
				num_free = num_free + 1

		if num_free > 4:
			canAccessPoint = True

		epsilon = 0.005
		canSeePoint = False
		ab = distLaserPosToPoint
		for i in range(0,len(laser)):
			ac = sqrt((pose[0]-laser[i][0])*(pose[0]-laser[i][0])+(pose[1]-laser[i][1])*(pose[1]-laser[i][1]))
			bc = sqrt((point[0]-laser[i][0])*(point[0]-laser[i][0])+(point[1]-laser[i][1])*(point[1]-laser[i][1]))
			if ((ab + bc) - ac) < epsilon:
				canSeePoint = True
				break
		if canSeePoint or canAccessPoint:
			return True
		else:
			return False

situation_model = SituationModel(10)
situation_model.listen()