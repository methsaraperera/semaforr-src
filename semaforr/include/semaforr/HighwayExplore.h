#ifndef HIGHWAYEXPLORE_H
#define HIGHWAYEXPLORE_H

/**!
  * HighwayExplore.h
  * 
  * /author: Raj Korpan
  *
  *          Construct Highway Graph
  */

#include <FORRGeometry.h>
#include <Position.h>
#include <FORRAction.h>
#include <vector>
#include <string>
#include <math.h>
#include <iostream>
#include <fstream>
#include <cmath>        //for atan2 and M_PI
#include <algorithm>
#include <map>
#include <set>
#include <numeric>
#include <queue>
#include <sensor_msgs/LaserScan.h>

using namespace std;

struct DecisionPoint{
	Position point;
	sensor_msgs::LaserScan laser;
	double farthest_view;
	double farthest_distance;
	DecisionPoint(): point(Position()), laser(sensor_msgs::LaserScan()) { }
	DecisionPoint(Position p, sensor_msgs::LaserScan ls){
		point = p;
		laser = ls;
		double max_value = 0;
		for(int i = 0; i < ls.ranges.size(); i++){
			if(ls.ranges[i] > max_value){
				farthest_distance = ls.ranges[i];
				farthest_view = i;
			}
		}
	}
	bool operator==(const DecisionPoint p) {
		return (point == p.point);
	}
	bool operator < (const DecisionPoint p) const{
		if(farthest_distance < p.farthest_distance)
			return true;
		else
			return false;
	}
};

class Highway{
public:
	Highway(DecisionPoint point){
		highway_points.push_back(point);
	}
	vector<DecisionPoint> getHighwayPoints(){return highway_points;}

	void addPointToHighway(DecisionPoint new_point, double distance_threshold){
		std::vector<DecisionPoint>::const_iterator it;
		it = find(highway_points.begin(), highway_points.end(), new_point);
		if(it == highway_points.end()){
			highway_points.push_back(new_point);
			if(new_point.farthest_distance >= distance_threshold){
				highway_queue.push(new_point);
			}
		}
	}

private:
	vector<DecisionPoint> highway_points;
	priority_queue<DecisionPoint> highway_queue;
};

class HighwayExplorer{
public:
	HighwayExplorer(double threshold){
		distance_threshold = threshold;
		highways = vector<Highway>();
	};
	~HighwayExplorer(){};

	FORRAction exploreDecision(Position current_point, sensor_msgs::LaserScan current_laser){
		DecisionPoint current_position = DecisionPoint(current_point, current_laser);
		if(abs(current_position.point.getTheta() - last_position.point.getTheta()) <= 0.2){
			highways[last_highway].addPointToHighway(current_position, distance_threshold);
		}
	}

private:
	double distance_threshold;
	vector<Highway> highways;
	DecisionPoint last_position;
	int last_highway;
};

#endif