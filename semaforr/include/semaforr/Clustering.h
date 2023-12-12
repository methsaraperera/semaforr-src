/************************************************
Clustering.h 
This file contains the class which contains clusters for the Situations that FORR learns and uses

Written by Raj Korpan, 2019
**********************************************/

#ifndef CLUSTERING_H
#define CLUSTERING_H

#include <vector>
#include <math.h>
#include <iostream>
#include <fstream>
#include <cmath>        //for atan2 and M_PI
#include <algorithm>
#include <cstdlib>
#include <python2.7/Python.h>

using namespace std;

class SpectralCluster{
public:
    SpectralCluster(int num_neighbors, int num_clusters, int min_cluster_size, double probability_cutoff){
        clusters = vector< vector<float> >();
        n_neighbors = num_neighbors;
        n_clusters = num_clusters;
        min_clust_size = min_cluster_size;
        prob_cutoff = probability_cutoff;
        cout << n_neighbors << " " << n_clusters << " " << min_clust_size << " " << prob_cutoff << endl;
        ofstream myfile;
        myfile.open("/home/raj/catkin_ws1/src/situation_learner/config/params.conf");
        myfile << "num_clusters " << n_clusters << "\n";
        myfile << "min_cluster_size " << min_clust_size << "\n";
        myfile << "num_neighbors " << n_neighbors << "\n";
        myfile << "probability_cutoff " << prob_cutoff;
        myfile.close();
    };
    ~SpectralCluster(){};

    vector< vector<float> > cluster(vector< vector<float> > data){
        ofstream mydatafile;
        mydatafile.open("/home/raj/catkin_ws1/src/situation_learner/config/data.txt");
        for(int i = 0; i < data.size(); i++){
            for(int j = 0; j < data[i].size(); j++){
                mydatafile << data[i][j] << " ";
            }
            mydatafile << "\n";
        }
        mydatafile.close();
        cout << "data written" << endl;
        const char* filename = "/home/raj/catkin_ws1/src/situation_learner/src/learn.py";
        PyObject* PyFileObject = PyFile_FromString(const_cast<char*>(filename), (char*)"r");
        // cout << "file opened" << endl;
        PyRun_SimpleFile(PyFile_AsFile(PyFileObject), filename);
        // cout << "python script finished" << endl;
        vector< vector<float> > new_situations;
        string clusterfilename = "/home/raj/catkin_ws1/src/situation_learner/config/clusters.txt";
        string fileLine;
        std::ifstream file(clusterfilename.c_str());
        ROS_DEBUG_STREAM("Reading new_situation_file:" << clusterfilename);
        //cout << "Inside file in tasks " << endl;
        if(!file.is_open()){
            ROS_DEBUG("Unable to locate or read situation config file!");
        }
        while(getline(file, fileLine)){
            //cout << "Inside while in tasks" << endl;
            if(fileLine[0] == '#')  // skip comment lines
                continue;
            else{
                std::stringstream ss(fileLine);
                std::istream_iterator<std::string> begin(ss);
                std::istream_iterator<std::string> end;
                std::vector<std::string> vstrings(begin, end);
                vector<float> values;
                for (int i=0; i<vstrings.size(); i++){
                    values.push_back(atof(vstrings[i].c_str()));
                }
                new_situations.push_back(values);
            }
        }
        return new_situations;
    }

    vector<int> clusterAssignments(){
        vector<int> assignments;
        string clusterfilename = "/home/raj/catkin_ws1/src/situation_learner/config/clusterassignments.txt";
        string fileLine;
        std::ifstream file(clusterfilename.c_str());
        ROS_DEBUG_STREAM("Reading new_situation_file:" << clusterfilename);
        //cout << "Inside file in tasks " << endl;
        if(!file.is_open()){
            ROS_DEBUG("Unable to locate or read situation config file!");
        }
        while(getline(file, fileLine)){
            //cout << "Inside while in tasks" << endl;
            if(fileLine[0] == '#')  // skip comment lines
                continue;
            else{
                std::stringstream ss(fileLine);
                std::istream_iterator<std::string> begin(ss);
                std::istream_iterator<std::string> end;
                std::vector<std::string> vstrings(begin, end);
                assignments.push_back(atoi(vstrings[0].c_str()));
            }
        }
        return assignments;
    }

    void eraseFiles(){
        system("exec rm -r /home/raj/catkin_ws1/src/situation_learner/config/*");
    }

private:
    vector< vector<float> > clusters;
    int n_neighbors;
    int n_clusters;
    int min_clust_size;
    double prob_cutoff;
};


#endif
