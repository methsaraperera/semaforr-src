#!/usr/bin/env python
import rospy
import numpy as np
np.set_printoptions(threshold=np.inf)
from scipy.cluster.vq import vq, kmeans, whiten
from scipy.cluster.hierarchy import fclusterdata
from sklearn.cluster import DBSCAN
from sklearn.cluster import AffinityPropagation
from sklearn import metrics
from sklearn.cluster import KMeans
from sklearn.cluster import MiniBatchKMeans
from sklearn.cluster import MeanShift
from sklearn.cluster import SpectralClustering
from sklearn.cluster import AgglomerativeClustering
from matplotlib import pyplot as plt
from scipy.cluster.hierarchy import dendrogram
from sklearn.decomposition import PCA
from sklearn.neighbors.nearest_centroid import NearestCentroid
from sklearn.preprocessing import StandardScaler
#import matplotlib.pyplot as plt
from geometry_msgs.msg import PoseArray, Pose, Point, PoseStamped
from sensor_msgs.msg import LaserScan
#from nav_msgs.msg import OccupancyGrid
#from semaforr.msg import CrowdModel
from visualization_msgs.msg import Marker
from visualization_msgs.msg import MarkerArray
from std_msgs.msg import String, Header
import re
#import itertools
import tf
from math import sqrt, floor, sin, cos, atan2
#import sys
#from cusum import Cusum

class SituationModel:
	def __init__(self, num):
		# set up this python class as ros node and initialize publisher and subscriber
		rospy.init_node('situation_model')
		rospy.Subscriber("decision_pose", PoseStamped, self.pose_data, queue_size=1000)
		rospy.Subscriber("decision_laser", LaserScan, self.laser_data, queue_size=1000)
		rospy.Subscriber("decision_log", String, self.decision_data, queue_size=1000)
		self.pub_situations = rospy.Publisher('situations', String, queue_size=1000)
		self.robot_pose = []
		self.laser_scan = []
		self.decisions = []
		self.num_clusters = num

	# calls the callback for each of the subscriber
	def listen(self):
		rospy.spin()

	# receive the laser data in standard format and save as a sequence of endpoints
	def laser_data(self, data):
		self.laser_scan.append(np.array(data.ranges).astype('float'))
		print "receiving laser_scan data message", len(self.laser_scan)
		if len(self.laser_scan) % 100 == 0 and len(self.laser_scan) == len(self.robot_pose) and len(self.laser_scan) >= 100:
			self.publish_situations()

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
		new_data = np.array(new_data[0:11]).astype('float')
		self.decisions.append(new_data)
		print "receiving decision log data message", len(self.decisions)

	def publish_situations(self):
		#numClusters = [2,3,4,5,6,7,8,9,10,12,14,16,18,20,25,30,35,40,45,50,60,70,80,90,100]
		numClusters = [2,3,4,5,6,7,8,9,10,12,14,16,18,20,25,30,35,40,45,50]
		features = np.array(self.laser_scan)
		# features2 = []
		# features11 = []
		# features22 = []
		# #features44 = []
		# #features110 = []
		# for item in features:
		# 	features2.append([np.mean(item[0:330]),np.mean(item[330:])])
		# 	features11.append([np.mean(item[0:60]),np.mean(item[60:120]),np.mean(item[120:180]),np.mean(item[180:240]),np.mean(item[240:300]),np.mean(item[300:360]),np.mean(item[360:420]),np.mean(item[420:480]),np.mean(item[480:540]),np.mean(item[540:600]),np.mean(item[600:])])
		# 	features22.append([np.mean(item[0:30]),np.mean(item[30:60]),np.mean(item[60:90]),np.mean(item[90:120]),np.mean(item[120:150]),np.mean(item[150:180]),np.mean(item[180:210]),np.mean(item[210:240]),np.mean(item[240:270]),np.mean(item[270:300]),np.mean(item[300:330]),np.mean(item[330:360]),np.mean(item[360:390]),np.mean(item[390:420]),np.mean(item[420:450]),np.mean(item[450:480]),np.mean(item[480:510]),np.mean(item[510:540]),np.mean(item[540:570]),np.mean(item[570:600]),np.mean(item[600:630]),np.mean(item[630:])])
		# features2 = np.array(features2)
		# features11 = np.array(features11)
		# features22 = np.array(features22)
		pca5 = PCA(n_components=5)
		pca10 = PCA(n_components=10)
		# pca20 = PCA(n_components=20)
		# pca100 = PCA(n_components=100)
		pca5.fit(features)
		pca10.fit(features)
		# pca20.fit(features)
		# pca100.fit(features)
		print pca5.components_
		print pca10.components_
		# #print pca20.explained_variance_
		print pca5.explained_variance_ratio_
		print pca10.explained_variance_ratio_
		# print pca20.explained_variance_ratio_
		# print pca100.explained_variance_ratio_
		features_pca5 = pca5.transform(features)
		features_pca10 = pca10.transform(features)
		# features_pca20 = pca20.transform(features)
		# features_pca100 = pca100.transform(features)
		# print features.shape, features_pca5.shape, features_pca10.shape, features_pca20.shape, features_pca100.shape
		#print pca20.singular_values_
		print len(self.laser_scan), len(self.robot_pose)
		#whitened = whiten(features)
		#stdev = np.std(features, axis = 0)
		print self.robot_pose
		new_features_5 = []
		new_features_10 = []
		for val in range(0, len(features_pca5)):
			merger = []
			for num in features_pca5[val]:
				merger.append(num)
			# for num in self.robot_pose[val]:
			# 	merger.append(num)
			targetX = self.decisions[val][4]
			targetY = self.decisions[val][5]
			distToTarget = sqrt((self.robot_pose[val][0] - targetX) * (self.robot_pose[val][0] - targetX) + (self.robot_pose[val][1] - targetY) * (self.robot_pose[val][1] - targetY))
			merger.append(distToTarget)
			angleToTarget = atan2(self.robot_pose[val][1] - targetY, self.robot_pose[val][0] - targetX)
			angleDiff = abs(atan2(sin(angleToTarget - self.robot_pose[val][2]), cos(angleToTarget - self.robot_pose[val][2])))
			# print self.robot_pose[val][0], self.robot_pose[val][1], self.robot_pose[val][2], targetX, targetY, angleToTarget, angleDiff
			merger.append(angleDiff)
			# merger.append(self.decisions[val][9])
			# merger.append(self.decisions[val][10])
			new_features_5.append(merger)
			merger = []
			for num in features_pca10[val]:
				merger.append(num)
			merger.append(distToTarget)
			merger.append(angleDiff)
			new_features_10.append(merger)
		new_features_5 = np.array(new_features_5)
		print new_features_5
		scaler5 = StandardScaler()
		scaler5.fit(new_features_5)
		new_features_scaled_5 = scaler5.transform(new_features_5)
		new_features_10 = np.array(new_features_10)
		print new_features_10
		scaler10 = StandardScaler()
		scaler10.fit(new_features_10)
		new_features_scaled_10 = scaler10.transform(new_features_10)
		# clusteringdb65 = DBSCAN(eps=65, min_samples=2, algorithm='brute').fit(features)
		# clusteringdb70 = DBSCAN(eps=70, min_samples=2, algorithm='brute').fit(features)
		# clusteringdb75 = DBSCAN(eps=75, min_samples=2, algorithm='brute').fit(features)
		# clusteringdbm65 = DBSCAN(eps=65, min_samples=2, metric='manhattan').fit(features)
		# clusteringdbm70 = DBSCAN(eps=70, min_samples=2, metric='manhattan').fit(features)
		# clusteringdbm75 = DBSCAN(eps=75, min_samples=2, metric='manhattan').fit(features)
		# clusteringdbpca5 = DBSCAN().fit(features_pca5)
		# clusteringdbpca10 = DBSCAN().fit(features_pca10)
		# clusteringdbpca20 = DBSCAN().fit(features_pca20)
		# clusteringdbpca100 = DBSCAN().fit(features_pca100)
		# clusteringaf = AffinityPropagation(max_iter=500).fit(features)
		# clusteringafpca5 = AffinityPropagation(max_iter=500).fit(features_pca5)
		# clusteringafnew = AffinityPropagation(max_iter=500).fit(new_features)
		# clusteringafnewsc = AffinityPropagation(max_iter=500).fit(new_features_scaled)
		# clusteringafpca10 = AffinityPropagation(max_iter=500).fit(features_pca10)
		# clusteringafpca20 = AffinityPropagation(max_iter=500).fit(features_pca20)
		# clusteringafpca100 = AffinityPropagation(max_iter=500).fit(features_pca100)
		# print 'clusteringdb65', clusteringdb65.labels_
		# print 'clusteringdb70', clusteringdb70.labels_
		# print 'clusteringdb75', clusteringdb75.labels_
		# print 'clusteringdbm65', clusteringdbm65.labels_
		# print 'clusteringdbm70', clusteringdbm70.labels_
		# print 'clusteringdbm75', clusteringdbm75.labels_
		# print 'clusteringdbpca5', clusteringdbpca5.labels_
		# print 'clusteringdbpca10', clusteringdbpca10.labels_
		# print 'clusteringdbpca20', clusteringdbpca20.labels_
		# print 'clusteringdbpca100', clusteringdbpca100.labels_
		# print 'clusteringaf', clusteringaf.labels_
		# print 'clusteringafpca5', clusteringafpca5.labels_
		# print 'clusteringafnew', clusteringafnew.labels_
		# print 'clusteringafnewsc', clusteringafnewsc.labels_
		# print 'clusteringafpca10', clusteringafpca10.labels_
		# print 'clusteringafpca20', clusteringafpca20.labels_
		# print 'clusteringafpca100', clusteringafpca100.labels_
		# situation = 'clusteringdb65_'+str(len(self.laser_scan))+' '+str(clusteringdb65.components_)
		# self.pub_situations.publish(situation)
		# situation = 'clusteringdb70_'+str(len(self.laser_scan))+' '+str(clusteringdb70.components_)
		# self.pub_situations.publish(situation)
		# situation = 'clusteringdb75_'+str(len(self.laser_scan))+' '+str(clusteringdb75.components_)
		# self.pub_situations.publish(situation)
		# situation = 'clusteringdbm65_'+str(len(self.laser_scan))+' '+str(clusteringdbm65.components_)
		# self.pub_situations.publish(situation)
		# situation = 'clusteringdbm70_'+str(len(self.laser_scan))+' '+str(clusteringdbm70.components_)
		# self.pub_situations.publish(situation)
		# situation = 'clusteringdbm75_'+str(len(self.laser_scan))+' '+str(clusteringdbm75.components_)
		# self.pub_situations.publish(situation)
		# situation = 'clusteringdbpca5_'+str(len(self.laser_scan))+' '+str(clusteringdbpca5.components_)
		# self.pub_situations.publish(situation)
		# situation = 'clusteringdbpca10_'+str(len(self.laser_scan))+' '+str(clusteringdbpca10.components_)
		# self.pub_situations.publish(situation)
		# situation = 'clusteringdbpca20_'+str(len(self.laser_scan))+' '+str(clusteringdbpca20.components_)
		# self.pub_situations.publish(situation)
		# situation = 'clusteringdbpca100_'+str(len(self.laser_scan))+' '+str(clusteringdbpca100.components_)
		# self.pub_situations.publish(situation)
		# situation = 'clusteringaf_'+str(len(self.laser_scan))+' '+str(clusteringaf.cluster_centers_)
		# self.pub_situations.publish(situation)
		# situation = 'clusteringafpca5_'+str(len(self.laser_scan))+' '+str(clusteringafpca5.cluster_centers_)
		# self.pub_situations.publish(situation)
		# situation = 'clusteringafnew_'+str(len(self.laser_scan))+' '+str(clusteringafnew.cluster_centers_)
		# self.pub_situations.publish(situation)
		# situation = 'clusteringafnewsc_'+str(len(self.laser_scan))+' '+str(scaler.inverse_transform(clusteringafnewsc.cluster_centers_))
		# self.pub_situations.publish(situation)
		# situation = 'clusteringafpca10_'+str(len(self.laser_scan))+' '+str(clusteringafpca10.cluster_centers_)
		# self.pub_situations.publish(situation)
		# situation = 'clusteringafpca20_'+str(len(self.laser_scan))+' '+str(clusteringafpca20.cluster_centers_)
		# self.pub_situations.publish(situation)
		# situation = 'clusteringafpca100_'+str(len(self.laser_scan))+' '+str(clusteringafpca100.cluster_centers_)
		# self.pub_situations.publish(situation)
		# max_score1 = -2.0
		# best_value1 = -1
		# max_score2 = -2.0
		# best_value2 = -1
		# for value in [2,50]:
		# 	clusteringacew = AgglomerativeClustering(n_clusters=value, affinity='euclidean', linkage='ward', compute_full_tree= True).fit(new_features)
		# 	clusteringacewsc = AgglomerativeClustering(n_clusters=value, affinity='euclidean', linkage='ward', compute_full_tree= True).fit(new_features_scaled)
		# 	value_score1 = metrics.silhouette_score(new_features, clusteringacew.labels_, metric='euclidean')
		# 	value_score2 = metrics.silhouette_score(new_features_scaled, clusteringacewsc.labels_, metric='euclidean')
		# 	print value, value_score1, value_score2
		# 	if value_score1 > max_score1:
		# 		max_score1 = value_score1
		# 		best_value1 = value
		# 	if value_score2 > max_score2:
		# 		max_score2 = value_score2
		# 		best_value2 = value
		# clusteringacew = AgglomerativeClustering(n_clusters=best_value1, affinity='euclidean', linkage='ward', compute_full_tree= True).fit(new_features)
		# children = clusteringacew.children_
		# distance = np.arange(children.shape[0])
		# no_of_observations = np.arange(2, children.shape[0]+2)
		# linkage_matrix = np.column_stack([children, distance, no_of_observations]).astype(float)
		# plt.title('Hierarchical Clustering Dendrogram '+str(len(self.laser_scan))+'_'+str(best_value1))
		# dendrogram(linkage_matrix, labels=clusteringacew.labels_, leaf_rotation=360)
		# figure = plt.gcf()
		# figure.set_size_inches(60, 40)
		# figure.savefig('Dendrogram_best_'+str(len(self.laser_scan))+'_'+str(best_value1)+'.svg', dpi = 400)
		# plt.close()
		# print 'clusteringacew_best_value_'+str(best_value1), max_score1, clusteringacew.labels_
		# acewcluster_centers = NearestCentroid()
		# acewcluster_centers.fit(new_features,clusteringacew.labels_)
		# situation = 'clusteringacew_best_value_'+str(len(self.laser_scan))+'_'+str(best_value1)+' '+str(acewcluster_centers.centroids_)
		# self.pub_situations.publish(situation)
		# clusteringacewsc = AgglomerativeClustering(n_clusters=best_value2, affinity='euclidean', linkage='ward', compute_full_tree= True).fit(new_features_scaled)
		# children = clusteringacewsc.children_
		# distance = np.arange(children.shape[0])
		# no_of_observations = np.arange(2, children.shape[0]+2)
		# linkage_matrix = np.column_stack([children, distance, no_of_observations]).astype(float)
		# plt.title('Hierarchical Clustering Dendrogram SC '+str(len(self.laser_scan))+'_'+str(best_value2))
		# dendrogram(linkage_matrix, labels=clusteringacewsc.labels_, leaf_rotation=360)
		# figure = plt.gcf()
		# figure.set_size_inches(60, 40)
		# figure.savefig('Dendrogram_bestsc_'+str(len(self.laser_scan))+'_'+str(best_value2)+'.svg', dpi = 400)
		# plt.close()
		# print 'clusteringacewsc_best_value_'+str(best_value2), max_score2, clusteringacewsc.labels_
		# acewcluster_centers = NearestCentroid()
		# acewcluster_centers.fit(new_features,clusteringacewsc.labels_)
		# situation = 'clusteringacewsc_best_value_'+str(len(self.laser_scan))+'_'+str(best_value2)+' '+str(scaler.inverse_transform(acewcluster_centers.centroids_))
		# self.pub_situations.publish(situation)
		for value in numClusters:
			if len(self.laser_scan) > value:
				#centroids, distortion = kmeans(whitened,value)
				#clx, disttocent = vq(whitened,centroids)
				#hierclustlabel = fclusterdata(whitened, t = 20, criterion='distance')
				clusteringkm = KMeans(n_clusters=value, n_init=50, max_iter=1000).fit(new_features_scaled_5)
				clusteringkmsc = KMeans(n_clusters=value, n_init=50, max_iter=1000).fit(new_features_scaled_10)
				# clusteringkm2 = KMeans(n_clusters=value, n_init=50, max_iter=1000).fit(features2)
				# clusteringkm11 = KMeans(n_clusters=value, n_init=50, max_iter=1000).fit(features11)
				# clusteringkm22 = KMeans(n_clusters=value, n_init=50, max_iter=1000).fit(features22)
				# clusteringkmpca5 = KMeans(n_clusters=value, n_init=50, max_iter=1000).fit(features_pca5)
				# clusteringkmpca10 = KMeans(n_clusters=value, n_init=50, max_iter=1000).fit(features_pca10)
				# clusteringkmpca20 = KMeans(n_clusters=value, n_init=50, max_iter=1000).fit(features_pca20)
				# clusteringkmpca100 = KMeans(n_clusters=value, n_init=50, max_iter=1000).fit(features_pca100)
				#clusteringmbkm = MiniBatchKMeans(n_clusters=value, batch_size=10, max_iter=10).fit(features)
				#clusteringms = MeanShift().fit(features)
				#clusteringsc = SpectralClustering(n_clusters=value, eigen_solver='arpack').fit(features)
				# clusteringacew = AgglomerativeClustering(n_clusters=value, affinity='euclidean', linkage='ward', compute_full_tree= True).fit(features)
				# children = clusteringacew.children_
				# distance = np.arange(children.shape[0])
				# no_of_observations = np.arange(2, children.shape[0]+2)
				# linkage_matrix = np.column_stack([children, distance, no_of_observations]).astype(float)
				# #fig = plt.figure()
				# plt.title('Hierarchical Clustering Dendrogram '+str(len(self.laser_scan))+'_'+str(value))
				# dendrogram(linkage_matrix, labels=clusteringacew.labels_, leaf_rotation=360)
				# #plt.show()
				# figure = plt.gcf()
				# figure.set_size_inches(60, 40)
				# figure.savefig('Dendrogram_'+str(len(self.laser_scan))+'_'+str(value)+'.svg', dpi = 400)
				# plt.close()
				#clusteringacec = AgglomerativeClustering(n_clusters=value, affinity='euclidean', linkage='complete').fit(features)
				#clusteringacea = AgglomerativeClustering(n_clusters=value, affinity='euclidean', linkage='average').fit(features)
				#clusteringaces = AgglomerativeClustering(n_clusters=value, affinity='euclidean', linkage='single').fit(features)
				#clusteringacmc = AgglomerativeClustering(n_clusters=value, affinity='manhattan', linkage='complete').fit(features)
				#clusteringacma = AgglomerativeClustering(n_clusters=value, affinity='manhattan', linkage='average').fit(features)
				#clusteringacms = AgglomerativeClustering(n_clusters=value, affinity='manhattan', linkage='single').fit(features)
				#clusteringaccc = AgglomerativeClustering(n_clusters=value, affinity='cosine', linkage='complete').fit(features)
				#clusteringacca = AgglomerativeClustering(n_clusters=value, affinity='cosine', linkage='average').fit(features)
				#clusteringaccs = AgglomerativeClustering(n_clusters=value, affinity='cosine', linkage='single').fit(features)
				#newcentroids = np.array([clx[:,i] * stdev[i] for i in range(0,len(stdev))]).transpose()
				situation = 'clusteringkm_'+str(len(self.laser_scan))+'_'+str(value)+' '+str(scaler5.inverse_transform(clusteringkm.cluster_centers_))
				self.pub_situations.publish(situation)
				situation = 'clusteringkmsc_'+str(len(self.laser_scan))+'_'+str(value)+' '+str(scaler10.inverse_transform(clusteringkmsc.cluster_centers_))
				self.pub_situations.publish(situation)
				# situation = 'clusteringkm2_'+str(len(self.laser_scan))+'_'+str(value)+' '+str(clusteringkm2.cluster_centers_)
				# self.pub_situations.publish(situation)
				# situation = 'clusteringkm11_'+str(len(self.laser_scan))+'_'+str(value)+' '+str(clusteringkm11.cluster_centers_)
				# self.pub_situations.publish(situation)
				# situation = 'clusteringkmpca5_'+str(len(self.laser_scan))+'_'+str(value)+' '+str(pca5.inverse_transform(clusteringkmpca5.cluster_centers_))
				# self.pub_situations.publish(situation)
				# situation = 'clusteringkmpca10_'+str(len(self.laser_scan))+'_'+str(value)+' '+str(pca10.inverse_transform(clusteringkmpca10.cluster_centers_))
				# self.pub_situations.publish(situation)
				# situation = 'clusteringkmpca10_'+str(len(self.laser_scan))+'_'+str(value)+' '+str(pca10.inverse_transform(clusteringkmpca10.cluster_centers_))
				# self.pub_situations.publish(situation)
				#print clx
				#print hierclustlabel
				print value, 'clusteringkmsc5', clusteringkm.labels_
				print value, 'clusteringkmsc10', clusteringkmsc.labels_
				# print 'clusteringkm2', clusteringkm2.labels_
				# print 'clusteringkm11', clusteringkm11.labels_
				# print 'clusteringkm22', clusteringkm22.labels_
				# print value, 'clusteringkmpca5', clusteringkmpca5.labels_
				# print 'clusteringkmpca10', clusteringkmpca10.labels_
				# print 'clusteringkmpca20', clusteringkmpca20.labels_
				# print 'clusteringkmpca100', clusteringkmpca100.labels_
				#print clusteringmbkm.labels_
				#print clusteringms.labels_
				#print 'clusteringsc', clusteringsc.labels_
				# print 'clusteringacew', clusteringacew.labels_
				#print clusteringacec.labels_
				#print clusteringacea.labels_
				#print clusteringaces.labels_
				#print clusteringacmc.labels_
				#print clusteringacma.labels_
				#print clusteringacms.labels_
				#print clusteringaccc.labels_
				#print clusteringacca.labels_
				#print clusteringaccs.labels_
				#print metrics.silhouette_score(features, clx, metric='euclidean'), metrics.silhouette_score(features, hierclustlabel, metric='euclidean'), metrics.silhouette_score(features, clusteringdb.labels_, metric='euclidean'), metrics.silhouette_score(features, clusteringaf.labels_, metric='euclidean'), metrics.silhouette_score(features, clusteringkm.labels_, metric='euclidean'), metrics.silhouette_score(features, clusteringmbkm.labels_, metric='euclidean'), metrics.silhouette_score(features, clusteringms.labels_, metric='euclidean'), metrics.silhouette_score(features, clusteringsc.labels_, metric='euclidean')
				#print metrics.silhouette_score(features, clusteringacew.labels_, metric='euclidean'), metrics.silhouette_score(features, clusteringacec.labels_, metric='euclidean'), metrics.silhouette_score(features, clusteringacea.labels_, metric='euclidean'), metrics.silhouette_score(features, clusteringaces.labels_, metric='euclidean'), metrics.silhouette_score(features, clusteringacmc.labels_, metric='euclidean'), metrics.silhouette_score(features, clusteringacma.labels_, metric='euclidean'), metrics.silhouette_score(features, clusteringacms.labels_, metric='euclidean'), metrics.silhouette_score(features, clusteringaccc.labels_, metric='euclidean'), metrics.silhouette_score(features, clusteringacca.labels_, metric='euclidean'), metrics.silhouette_score(features, clusteringaccs.labels_, metric='euclidean')
				#print value, metrics.silhouette_score(features, clx, metric='euclidean'), metrics.silhouette_score(features, clusteringkm.labels_, metric='euclidean'), metrics.silhouette_score(features, clusteringmbkm.labels_, metric='euclidean'), metrics.silhouette_score(features, clusteringacew.labels_, metric='euclidean'), metrics.silhouette_score(features, clusteringacec.labels_, metric='euclidean'), metrics.silhouette_score(features, clusteringacmc.labels_, metric='euclidean'), metrics.silhouette_score(features, clusteringaccc.labels_, metric='euclidean')
				#print value, metrics.silhouette_score(features, clusteringkm.labels_, metric='euclidean'), metrics.silhouette_score(features2, clusteringkm2.labels_, metric='euclidean'), metrics.silhouette_score(features11, clusteringkm11.labels_, metric='euclidean'), metrics.silhouette_score(features22, clusteringkm22.labels_, metric='euclidean'), metrics.silhouette_score(features_pca5, clusteringkmpca5.labels_, metric='euclidean'), metrics.silhouette_score(features_pca10, clusteringkmpca10.labels_, metric='euclidean'), metrics.silhouette_score(features_pca20, clusteringkmpca20.labels_, metric='euclidean'), metrics.silhouette_score(features_pca100, clusteringkmpca100.labels_, metric='euclidean')
				#print value, metrics.calinski_harabaz_score(features, clusteringkm.labels_), metrics.calinski_harabaz_score(features2, clusteringkm2.labels_), metrics.calinski_harabaz_score(features11, clusteringkm11.labels_), metrics.calinski_harabaz_score(features22, clusteringkm22.labels_), metrics.calinski_harabaz_score(features_pca5, clusteringkmpca5.labels_), metrics.calinski_harabaz_score(features_pca10, clusteringkmpca10.labels_), metrics.calinski_harabaz_score(features_pca20, clusteringkmpca20.labels_), metrics.calinski_harabaz_score(features_pca100, clusteringkmpca100.labels_)
				#print value, metrics.davies_bouldin_score(features, clusteringkm.labels_), metrics.davies_bouldin_score(features2, clusteringkm2.labels_), metrics.davies_bouldin_score(features11, clusteringkm11.labels_), metrics.davies_bouldin_score(features22, clusteringkm22.labels_), metrics.davies_bouldin_score(features_pca5, clusteringkmpca5.labels_), metrics.davies_bouldin_score(features_pca10, clusteringkmpca10.labels_), metrics.davies_bouldin_score(features_pca20, clusteringkmpca20.labels_), metrics.davies_bouldin_score(features_pca100, clusteringkmpca100.labels_)

situation_model = SituationModel(10)
situation_model.listen()