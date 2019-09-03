#!/usr/bin/env python
import rospy
import numpy as np
np.set_printoptions(threshold=np.inf)
# import scipy.spatial as scp
# import networkx as nx
# from sklearn.cluster import KMeans
# from sklearn.preprocessing import StandardScaler
# from matplotlib import pyplot as plt
from geometry_msgs.msg import PoseArray, Pose, Point, PoseStamped
from sensor_msgs.msg import LaserScan
from visualization_msgs.msg import Marker
from visualization_msgs.msg import MarkerArray
from std_msgs.msg import String, Header
import re
import tf
from math import sqrt, floor, sin, cos, atan2, ceil, radians
from sklearn.cluster import SpectralClustering
from sklearn.neural_network import MLPClassifier

def clusterActions(n_clus, n_neigh, min_clus_size, data):
	SC = SpectralClustering(n_clusters = n_clus, affinity='nearest_neighbors', assign_labels='kmeans', n_neighbors=n_neigh).fit(data)
	clusters = np.array(SC.labels_)
	cluster_vals = []
	for i in range(0,n_clus):
		if np.count_nonzero(clusters == i) < min_clus_size:
			cluster_vals.append(i)
	print cluster_vals
	cluster_labels = []
	for val in clusters:
		if val in cluster_vals:
			cluster_labels.append(-1)
		else:
			cluster_labels.append(val)
	return np.array(cluster_labels)

def classifyActions(data, clusters, newdata, prob_cutoff, min_clus_size):
	X = data[clusters >= 0]
	Y = clusters[clusters >= 0]
	mlp = MLPClassifier(max_iter = 2000, verbose=False).fit(X, Y)
	prob = mlp.predict_proba(newdata)
	pred = mlp.predict(newdata)
	cluster_labels = []
	for i in range(0,len(prob)):
		if max(prob[i]) < prob_cutoff:
			cluster_labels.append(-1)
		else:
			cluster_labels.append(pred[i])
	cluster_vals = []
	for i in range(0,len(set(cluster_labels))):
		if np.count_nonzero(clusters == i) < min_clus_size:
			cluster_vals.append(i)
	print cluster_vals
	cluster_labels_new = []
	for val in cluster_labels:
		if val in cluster_vals:
			cluster_labels_new.append(-1)
		else:
			cluster_labels_new.append(val)
	return np.array(cluster_labels_new)

class SituationModel:
	def __init__(self, num_clus, min_clus, prob_cut):
		# set up this python class as ros node and initialize publisher and subscriber
		rospy.init_node('situation_model')
		rospy.Subscriber("decision_pose", PoseStamped, self.pose_data, queue_size=10000)
		rospy.Subscriber("decision_log", String, self.decision_data, queue_size=10000)
		rospy.Subscriber("decision_laser", LaserScan, self.laser_data, queue_size=10000)
		self.pub_situations = rospy.Publisher('situations', String, queue_size=10000)
		self.action_grid_data = []
		self.robot_pose = []
		self.laser_scan = []
		self.decisions = []
		self.trails = []
		self.laser_end_points = []
		self.data_num = 0
		self.num_clusters = num_clus
		self.min_cluster_size = min_clus
		self.probability_cutoff = prob_cut

	# calls the callback for each of the subscriber
	def listen(self):
		rospy.spin()

	# receive the laser data in standard format and save as a sequence of endpoints
	def laser_data(self, data):
		self.laser_scan.append(np.array(data.ranges).astype('float'))
		print "receiving laser_scan data message", len(self.laser_scan)
		self.laserToActionGrid(data)
		# if np.amax(self.laser_scan[-1]) > 20:
			# print "max range detected"
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
			# if len(self.laser_scan) == len(self.robot_pose) and len(self.laser_scan) == len(self.decisions) and len(self.robot_pose) == len(self.decisions) and self.decisions[-1][0] != self.decisions[-2][0]:
			if len(self.action_grid_data) > 500 and self.decisions[-1][0] != self.decisions[-2][0]:
				print len(self.laser_scan), len(self.robot_pose), len(self.decisions)
				self.data_num = len(self.laser_scan)-3
				self.publish_situations()

	def publish_situations(self):
		Actions = np.array(self.action_grid_data[0:self.data_num])
		clustersSC = clusterActions(self.num_clusters, 15, self.min_cluster_size, Actions)
		clusters = classifyActions(Actions, clustersSC, Actions, self.probability_cutoff, self.min_cluster_size)
		print clusters
		Mean_Values = []
		Mean_Counts = []
		for i in range(0, self.num_clusters):
			if len(Actions[np.array(clusters) == i]) > 0:
				Mean_Values.append(np.mean(Actions[np.array(clusters) == i], axis=0))
				Mean_Counts.append(len(Actions[np.array(clusters) == i]))
		situations = ""
		for i in range(0, len(Mean_Counts)):
			values = ""
			for j in range(0, len(Mean_Values[i])):
				values = values + str(Mean_Values[i][j]) + " "
			situations = situations+"\n"+str(Mean_Counts[i])+" "+values[:-1]

		print situations
		self.pub_situations.publish(situations)

		# Targets = []
		# for val in range(0, self.data_num,1):
		# 	merger = []
		# 	targetX = self.decisions[val][4]
		# 	targetY = self.decisions[val][5]
		# 	distToTarget = sqrt((self.robot_pose[val][0] - targetX) * (self.robot_pose[val][0] - targetX) + (self.robot_pose[val][1] - targetY) * (self.robot_pose[val][1] - targetY))
		# 	merger.append(distToTarget)
		# 	angleToTarget = atan2((self.robot_pose[val][1] - targetY), (self.robot_pose[val][0] - targetX))
		# 	RequiredRotation = angleToTarget - self.robot_pose[val][2]
		# 	if RequiredRotation > 3.141592654:
		# 		RequiredRotation = RequiredRotation - (2 * 3.141592654)
		# 	if RequiredRotation < -3.141592654:
		# 		RequiredRotation = RequiredRotation + (2 * 3.141592654)
		# 	merger.append(RequiredRotation)
		# 	Targets.append(merger)
		# Targets = np.array(Targets)
		# # scaler = StandardScaler()
		# # scaler.fit(Targets)
		# # TargetsScaled = scaler.transform(Targets)
		# # NumClusters = 20
		# # if self.data_num<20:
		# # 	NumClusters = self.data_num - 1
		# # TargetClusters = KMeans(n_clusters=NumClusters, n_init=50, max_iter=1000).fit(TargetsScaled)
		# # situation = 'TargetClusters_'+str(self.data_num)+' '+str(TargetClusters.labels_)
		# CustomDistances = scp.distance.pdist(Targets, dfun)
		# SimilarityMatrix = scp.distance.squareform(CustomDistances)
		# SimilarityMatrixInt = (np.array(SimilarityMatrix) == 0).astype(int)
		# np.fill_diagonal(SimilarityMatrixInt, 0)
		# adjacency_matrix = np.array(SimilarityMatrixInt)
		# rows, cols = np.where(adjacency_matrix == 1)
		# edges = zip(rows.tolist(), cols.tolist())
		# gr = nx.Graph()
		# gr.add_edges_from(edges)
		# cliques = sorted(list(nx.find_cliques(gr)), key = len, reverse=True)
		# filtered_cliques = []
		# while len(cliques) > 0:
		# 	filtered_cliques.append(cliques[0])
		# 	cliques.remove(cliques[0])
		# 	cliques_to_remove = []
		# 	for val in cliques:
		# 		item_in_clique = False
		# 		for item in filtered_cliques[-1]:
		# 			if item in val:
		# 				item_in_clique = True
		# 		if item_in_clique == True:
		# 			cliques_to_remove.append(val)
		# 	for item in cliques_to_remove:
		# 		cliques.remove(item)
		# filtered_cliques = np.array(filtered_cliques)
		# cluster_values = []
		# for i in range(0,len(Targets)):
		# 	cluster_found = 0
		# 	for j in range(0,len(filtered_cliques)):
		# 		if i in filtered_cliques[j]:
		# 			cluster_values.append(j)
		# 			cluster_found = 1
		# 	if cluster_found == 0:
		# 		cluster_values.append(-1)
		# situation = 'TargetClusters_'+str(self.data_num)+' '+str(cluster_values)
		# print situation
		# self.pub_situations.publish(situation)

		# TargetTrailMarker = []
		# for i in range(0, self.data_num,1):
		# 	# print len(self.trails), self.decisions[i][0]
		# 	CurrentTrail = self.trails[int(self.decisions[i][0])]
		# 	CurrentPose = self.robot_pose[i]
		# 	CurrentLaser = self.laser_end_points[i]
		# 	found = False
		# 	for j in range(len(CurrentTrail)-1, -1, -1):
		# 		if self.can_access_point(CurrentLaser, CurrentPose, CurrentTrail[j], 20):
		# 			TargetTrailMarker.append(CurrentTrail[j])
		# 			found = True
		# 			# print i
		# 			break
		# 	if found == False:
		# 		TargetTrailMarker.append([self.decisions[i][4], self.decisions[i][5]])
		# 	# print i
		# BestActions = []
		# # print len(TargetTrailMarker), self.data_num
		# for i in range(0, self.data_num,1):
		# 	CurrentPose = self.robot_pose[i]
		# 	# print i
		# 	CurrentTrailMarker = TargetTrailMarker[i]
		# 	distance_from_tm = sqrt((CurrentPose[0]-CurrentTrailMarker[0])*(CurrentPose[0]-CurrentTrailMarker[0])+(CurrentPose[1]-CurrentTrailMarker[1])*(CurrentPose[1]-CurrentTrailMarker[1]))
		# 	distance_from_obstacle = 100000
		# 	error_margin = 0.05
		# 	start_view = int(-0.35/0.005817)+330
		# 	end_view = int(0.35/0.005817)+330
		# 	for angle in range(start_view, end_view+1):
		# 		if self.laser_scan[i][angle] < distance_from_obstacle:
		# 			distance_from_obstacle = self.laser_scan[i][angle]
		# 	distance_from_obstacle = distance_from_obstacle - error_margin
		# 	robot_direction = CurrentPose[2]
		# 	goal_direction = atan2((CurrentTrailMarker[1] - CurrentPose[1]), (CurrentTrailMarker[0] - CurrentPose[0]))
		# 	RequiredRotation = goal_direction - robot_direction
		# 	if RequiredRotation > 3.141592654:
		# 		RequiredRotation = RequiredRotation - (2 * 3.141592654)
		# 	if RequiredRotation < -3.141592654:
		# 		RequiredRotation = RequiredRotation + (2 * 3.141592654)
		# 	ActionValues = []
		# 	move = [0, 0.2, 0.4, 0.8, 1.6, 3.2]
		# 	rotate = [0, 0.25, 0.5, 1, 2]
		# 	numMoves = 6
		# 	numRotates = 5
		# 	rotIntensity = 0
		# 	while (rotIntensity < numRotates and abs(RequiredRotation) > rotate[rotIntensity]):
		# 		rotIntensity = rotIntensity + 1
		# 	if rotIntensity > 1:
		# 		if RequiredRotation < 0:
		# 			ActionValues.append([1,rotIntensity-1])
		# 		else:
		# 			ActionValues.append([2,rotIntensity-1])
		# 	else:
		# 		ActionValues.append([0,0])
		# 	intensity = 0
		# 	while (intensity < numMoves and distance_from_tm > move[intensity]):
		# 		intensity = intensity + 1
		# 	obstacle_intensity = 0
		# 	while (obstacle_intensity < numMoves and distance_from_obstacle > move[obstacle_intensity]):
		# 		obstacle_intensity = obstacle_intensity + 1
		# 	if intensity > obstacle_intensity:
		# 		intensity = obstacle_intensity
		# 	ActionValues.append([0,intensity-1])
		# 	BestActions.append(ActionValues)
		# situation = 'BestActions_'+str(self.data_num)+' '+str(BestActions)
		# print situation
		# self.pub_situations.publish(situation)

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

	def laserToActionGrid(self, data):
		scan = np.array(data.ranges).astype('float')
		angle = data.angle_min
		increment = data.angle_increment
		values = np.zeros((51,51))
		for dist in scan:
			for i in np.arange(0.0,dist,0.9):
				x = int(round(i * cos(angle)))+25
				y = int(round(i * sin(angle)))+25
				values[x][y] = 1
				x = int(i * cos(angle))+25
				y = int(i * sin(angle))+25
				values[x][y] = 1
				x = int(floor(i * cos(angle)))+25
				y = int(floor(i * sin(angle)))+25
				values[x][y] = 1
				x = int(ceil(i * cos(angle)))+25
				y = int(ceil(i * sin(angle)))+25
				values[x][y] = 1
			angle = angle + increment
		self.action_grid_data.append(values[16:].flatten())

situation_model = SituationModel(15, 10, 0.95)
situation_model.listen()