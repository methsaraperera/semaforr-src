/************************************************
FORRSituations.h 
This file contains the class which contains information about the Situations that FORR learns and uses

Written by Raj Korpan, 2019
**********************************************/

#ifndef FORRSITUATIONS_H
#define FORRSITUATIONS_H

#include <FORRGeometry.h>
#include <Position.h>
#include <FORRTrails.h>
#include <AgentState.h>
#include <FORRAction.h>
#include <Clustering.h>
#include <vector>
#include <string>
#include <math.h>
#include <iostream>
#include <cmath>        //for atan2 and M_PI
#include <algorithm>
#include <map>
#include <set>
#include <sensor_msgs/LaserScan.h>
#include <std_msgs/String.h>

using namespace std;

class SituationMarker{
public:
    sensor_msgs::LaserScan ls;
    Position pose;
    int assignment;
    vector<float> action_grid;
    SituationMarker(sensor_msgs::LaserScan laser, Position ps, int a, vector<float> ag): ls(laser), pose(ps), assignment(a), action_grid(ag){}
};

//=========================================================//=========================================================//

/* FORRSituations class
 *
 * FORRSituations represent the Situations that are learned by SemaFORR.
 *
 */
class FORRSituations{
public:
    FORRSituations(){
        situations = vector< vector<float> >();
        situation_counts = vector<int>();
        dist_cutoff = 75;
    };
    vector< vector<float> > getSituations(){return situations;}
    ~FORRSituations(){};

    void clearAllSituations(){
        situations.clear();
        situation_counts.clear();
        // situation_assignments.clear();
        action_assignments.clear();
        situation_observations.clear();
        action_assignment_weights.clear();
    }

    //Initialize situations from config
    void createSituations(int count, vector<float> values){
        situations.push_back(values);
        situation_counts.push_back(count);
        vector< vector<float> > grid;
        for(int i = 0; i < 51; i++){
            vector<float> col;
            for(int j = 0; j < 51; j++){
                col.push_back(0);
            }
            grid.push_back(col);
        }
        for (int i = 0; i < values.size(); i++){
            int row = i / 51 + 16;
            int col = i %51;
            grid[row][col] = values[i];
        }
        for(int i = 16; i < grid.size(); i++){
            for(int j = 0; j < grid[i].size(); j++){
                if(grid[i][j] == 0){
                    cout << "0.0 ";
                }
                else if(grid[i][j] == 1){
                    cout << "1.0 ";
                }
                else{
                    cout << floor(grid[i][j]*10+0.5)/10 << " ";
                }
            }
            cout << endl;
        }
        cout << endl;
    }

    //Print situations
    void printSituations(){
        for(int k = 0; k < situations.size(); k++){
            vector<float> values = situations[k];
            vector< vector<float> > grid;
            for(int i = 0; i < 51; i++){
                vector<float> col;
                for(int j = 0; j < 51; j++){
                    col.push_back(0);
                }
                grid.push_back(col);
            }
            for (int i = 0; i < values.size(); i++){
                int row = i / 51 + 16;
                int col = i %51;
                grid[row][col] = values[i];
            }
            cout << situation_counts[k] << " ";
            for(int i = 16; i < grid.size(); i++){
                for(int j = 0; j < grid[i].size(); j++){
                    if(grid[i][j] == 0){
                        cout << "0.0 ";
                    }
                    else if(grid[i][j] == 1){
                        cout << "1.0 ";
                    }
                    else{
                        cout << grid[i][j] << " ";
                    }
                }
                cout << endl;
            }
            cout << endl;
        }
    }

    // Modify situations from new observations
    void addObservationToSituations(sensor_msgs::LaserScan ls, Position pose, bool add_to_existing);

    // Cluster outliers
    void clusterOutlierObservations();

    // Update situations from new clustering
    void updateSituations(AgentState *agentState, std_msgs::String sits, vector< Position > *position_hist, vector< vector<CartesianPoint> > *laser_hist, vector< sensor_msgs::LaserScan > ls_hist, vector< vector< TrailMarker> > trails);

    // Fit laserscan to a situation
    vector<int> identifySituation(sensor_msgs::LaserScan ls);

    // Get situation assignment for current laserscan
    int identifySituationAssignment(sensor_msgs::LaserScan ls);

    // Associate situations with actions based on trails and target
    void learnSituationActions(AgentState *agentState, vector<TrailMarker> trail, int begin_vec, int end_vec);

    // Return weights for actions based on situations
    double getWeightForAction(AgentState *agentState, FORRAction action);

private:
    // Learned views
    vector< vector<float> > situations;

    // Number of instances per view
    vector<int> situation_counts;

    // Situation observations
    vector<SituationMarker> situation_observations;

    // View assignment for observations
    // vector<int> situation_assignments;

    // Laser observations
    // vector<sensor_msgs::LaserScan> situation_laserscans;

    // Assignment of actions for situations
    map< vector<int>, vector<FORRAction> > action_assignments;

    // Weights for actions for situations
    map< vector<int>, map< FORRAction, double > > action_assignment_weights;

    // Threshold for adding to situations
    float dist_cutoff;
};

#endif
