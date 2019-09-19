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
    SpectralCluster(int num_neighbors, int num_clusters, double probability_cutoff){
        clusters = vector< vector<float> >();
        n_neighbors = num_neighbors;
        n_clusters = num_clusters;
        prob_cutoff = probability_cutoff;
        cout << n_neighbors << " " << n_clusters << " " << prob_cutoff << endl;
    };
    ~SpectralCluster(){};

    vector<int> cluster(vector< vector<float> > data){
        ofstream myfile;
        myfile.open ("/home/rajkochhar/catkin_ws1/src/situation_learner/config/data.txt");
        for(int i = 0; i < data.size(); i++){
            for(int j = 0; j < data[i].size(); j++){
                myfile << data[i][j] << " ";
            }
            myfile << "\n";
        }
        myfile.close();
        // cout << "data written" << endl;
        const char* filename = "/home/rajkochhar/catkin_ws1/src/situation_learner/src/learn.py";
        PyObject* PyFileObject = PyFile_FromString(const_cast<char*>(filename), "r");
        // cout << "file opened" << endl;
        PyRun_SimpleFile(PyFile_AsFile(PyFileObject), filename);
        // cout << "python script finished" << endl;
        vector<int> clusters;
        return clusters;
    }

private:
    vector< vector<float> > clusters;
    int n_neighbors;
    int n_clusters;
    double prob_cutoff;
};


#endif
