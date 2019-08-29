/************************************************
FORRSituations.h 
This file contains the class which contains information about the Situations that FORR learns and uses

Written by Raj Korpan, 2019
**********************************************/

#ifndef FORRSITUATIONS_H
#define FORRSITUATIONS_H

#include <FORRGeometry.h>
#include <vector>
#include <string>
#include <math.h>
#include <iostream>
#include <cmath>        //for atan2 and M_PI
#include <algorithm>
#include <map>
#include <set>
#include <sensor_msgs/LaserScan.h>

using namespace std;

//=========================================================//=========================================================//

/* FORRSituations class
 *
 * FORRSituations represent the Situations that are learned by SemaFORR.
 *
 */
class FORRSituations{
public:
    FORRSituations(){
        situations = vector< vector<int> >();
        situation_counts = vector<int>();
    };
    vector<LineSegment> getSituations(){return situations;}
    ~FORRSituations(){};

    void clearAllSituations(){
        situations.clear();
    }
    void createSituations(int count, vector<int> values); //Initialize situations from config

    void addObservationToSituations(); //Modify situations from new observations

    void updateSituations(); //Update situations from new clustering

    vector<int> identifySituation(sensor_msgs::LaserScan ls); //Fit laserscan to a situation

private:
    vector< vector<int> > situations;
    vector<int> situation_counts;
};

#endif
