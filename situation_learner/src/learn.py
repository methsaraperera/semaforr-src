#!/usr/bin/env python
import os
import fnmatch
import numpy as np
np.set_printoptions(threshold=np.inf)
import re
from sklearn.cluster import SpectralClustering
from sklearn.neural_network import MLPClassifier

def clusterActions(n_clus, n_neigh, min_clus_size, data):
	SC = SpectralClustering(n_clusters = n_clus, affinity='nearest_neighbors', assign_labels='kmeans', n_neighbors=n_neigh).fit(data)
	clusters = np.array(SC.labels_)
	cluster_vals = []
	for i in range(0,n_clus):
		if np.count_nonzero(clusters == i) < min_clus_size:
			cluster_vals.append(i)
	# print cluster_vals
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
	# print cluster_vals
	cluster_labels_new = []
	for val in cluster_labels:
		if val in cluster_vals:
			cluster_labels_new.append(-1)
		else:
			cluster_labels_new.append(val)
	return np.array(cluster_labels_new)

def publish_situations():
	# print "inside publish_situations"
	action_grid_data = []
	path = os.path.expanduser('~/catkin_ws1/src/situation_learner/config/')
	for filename in os.listdir(path):
		if fnmatch.fnmatch(filename, 'data.txt'):
			filename = os.path.join(path,filename)
			with open(filename,'r') as fin:
				for line in fin:
					line = re.split(r' ',line)
					if len(line) > 1:
						line = np.array(line[:-1]).astype('float')
						action_grid_data.append(line)
	# print len(action_grid_data), len(action_grid_data[0]), num_clusters, min_cluster_size, probability_cutoff
	Actions = np.array(action_grid_data)
	if len(Actions)*2 > num_clusters:
		clustersSC = clusterActions(num_clusters, num_neighbors, min_cluster_size, Actions)
		if len(set(clustersSC)) > 2:
			clusters = classifyActions(Actions, clustersSC, Actions, probability_cutoff, min_cluster_size)
		else:
			clusters = clustersSC
		# print clusters
		clus_values = sorted(set(clusters))
		if clus_values[0] == -1:
			clus_values = clus_values[1:]
		clusters_renumbered = []
		for val in clusters:
			if val == -1:
				clusters_renumbered.append(str(val))
			else:
				clusters_renumbered.append(str(clus_values.index(val)))
		print clusters_renumbered
		Mean_Values = []
		Mean_Counts = []
		for i in range(0, num_clusters):
			if len(Actions[np.array(clusters) == i]) > 0:
				Mean_Values.append(np.mean(Actions[np.array(clusters) == i], axis=0))
				Mean_Counts.append(len(Actions[np.array(clusters) == i]))
		situations = ""
		for i in range(0, len(Mean_Counts)):
			values = ""
			for j in range(0, len(Mean_Values[i])):
				values = values + str(Mean_Values[i][j]) + " "
			situations = situations+"\n"+str(Mean_Counts[i])+" "+values[:-1]

		file1 = open(os.path.join(path, "clusters.txt"),"w")
		file1.writelines(situations[1:])
		file1.close()
		file2 = open(os.path.join(path, "clusterassignments.txt"),"w")
		for val in clusters_renumbered[:-1]:
			file2.writelines(val + "\n")
		file2.writelines(clusters_renumbered[-1])
		file2.close()
	else:
		file1 = open(os.path.join(path, "clusters.txt"),"w")
		file1.writelines("#")
		file1.close()
		file2 = open(os.path.join(path, "clusterassignments.txt"),"w")
		file2.writelines("#")
		file2.close()

# print "inside learn.py"
num_clusters = 0
min_cluster_size = 0
num_neighbors = 0
probability_cutoff = 0

path = os.path.expanduser('~/catkin_ws1/src/situation_learner/config/')
for filename in os.listdir(path):
	if fnmatch.fnmatch(filename, 'params.conf'):
		filename = os.path.join(path,filename)
		with open(filename,'r') as fin:
			for line in fin:
				line = re.split(r' ',line)
				if len(line) > 1:
					if line[0] == "num_clusters":
						num_clusters = int(line[1])
					elif line[0] == "min_cluster_size":
						min_cluster_size = int(line[1])
					elif line[0] == "num_neighbors":
						num_neighbors = int(line[1])
					elif line[0] == "probability_cutoff":
						probability_cutoff = float(line[1])

# print "situation_model created"
publish_situations()