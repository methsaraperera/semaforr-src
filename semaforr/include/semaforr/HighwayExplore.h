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
	double middle_distance;
	DecisionPoint(): point(Position()), laser(sensor_msgs::LaserScan()) { }
	DecisionPoint(Position p, sensor_msgs::LaserScan ls){
		point = p;
		laser = ls;
		farthest_distance = 0;
		farthest_view = 329;
		middle_distance = 0;
		double max_value = 0;
		for(int i = 0; i < ls.ranges.size(); i++){
			if((i <= 262 or i >= 398)){
				if(ls.ranges[i] > max_value){
					farthest_distance = ls.ranges[i];
					farthest_view = i;
					max_value = ls.ranges[i];
				}
			}
			else{
				middle_distance += ls.ranges[i];
			}
		}
		middle_distance = middle_distance / 135.0;
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

	void addPointToHighway(DecisionPoint new_point){
		std::vector<DecisionPoint>::const_iterator it;
		it = find(highway_points.begin(), highway_points.end(), new_point);
		if(it == highway_points.end()){
			highway_points.push_back(new_point);
		}
	}

private:
	vector<DecisionPoint> highway_points;
};

class HighwayExplorer{
public:
	HighwayExplorer(int l, int h, double threshold){
		distance_threshold = threshold;
		length = l;
		height = h;
		cout << "Highway length " << length << " height " << height << endl;
		for(int i = 0; i < l; i++){
			vector<int> col;
			for(int j = 0; j < h; j ++){
				col.push_back(-1);
			}
			highway_grid.push_back(col);
		}
		highways = vector<Highway>();
		highways_complete = false;
	};
	~HighwayExplorer(){};

	bool getHighwaysComplete(){return highways_complete;}

	FORRAction exploreDecision(Position current_point, sensor_msgs::LaserScan current_laser){
		DecisionPoint current_position = DecisionPoint(current_point, current_laser);
		cout << "current_position " << current_position.point.getX() << " " << current_position.point.getY() << " " << current_position.point.getTheta() << " " << current_position.middle_distance << " " << current_position.farthest_distance << endl;
		if(highways.size() == 0){
			Highway initial_highway = Highway(current_position);
			highways.push_back(initial_highway);
			last_position = current_position;
			last_highway = 0;
		}
		cout << "last_position " << last_position.point.getX() << " " << last_position.point.getY() << " " << last_position.point.getTheta() << " " << last_highway << endl;
		if(current_position.middle_distance >= 1.0){
			highways[last_highway].addPointToHighway(current_position);
			cout << "Current grid value " << highway_grid[(int)(current_position.point.getX())][(int)(current_position.point.getY())] << endl;
			if(current_position.farthest_distance >= distance_threshold and highway_grid[(int)(current_position.point.getX())][(int)(current_position.point.getY())] == -1){
				cout << "Adding to priority queue " << highway_queue.size() << " distance " << current_position.farthest_distance << " view " << current_position.farthest_view << endl;
				highway_queue.push(current_position);
			}
			highway_grid[(int)(current_position.point.getX())][(int)(current_position.point.getY())] = last_highway;
			last_position = current_position;
			return FORRAction(FORWARD, 1);
		}
		else if(highway_queue.size() > 0){
			DecisionPoint top_point = highway_queue.top();
			highway_queue.pop();
			Highway new_highway = Highway(top_point);
			highways.push_back(new_highway);
			last_position = current_position;
			last_highway = last_highway + 1;
			if(top_point.farthest_view > 330){
				return FORRAction(RIGHT_TURN, 6);
			}
			else{
				return FORRAction(LEFT_TURN, 6);
			}
		}
		else{
			return FORRAction(FORWARD, 0);
		}
	}

private:
	int length;
	int height;
	double distance_threshold;
	vector< vector<int> > highway_grid;
	vector<Highway> highways;
	priority_queue<DecisionPoint> highway_queue;
	DecisionPoint last_position;
	int last_highway;
	bool highways_complete;
};

#endif