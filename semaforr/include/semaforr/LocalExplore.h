#ifndef LOCALEXPLORE_H
#define LOCALEXPLORE_H

/**!
  * LocalExplore.h
  * 
  * /author: Raj Korpan
  *
  *          Conduct local exploration
  */

#include <FORRGeometry.h>
#include <Position.h>
#include <vector>
#include <math.h>
#include <iostream>
#include <cmath>        //for atan2 and M_PI
#include <algorithm>
#include <map>
#include <set>
#include <numeric>
#include <queue>

using namespace std;

struct PotentialPoints{
	LineSegment pair;
	CartesianPoint start;
	CartesianPoint end;
	double dist_to_goal;
	double start_dist_to_goal;
	double end_dist_to_goal;
	PotentialPoints(): pair(LineSegment()), start(CartesianPoint()), end(CartesianPoint()), dist_to_goal(0), start_dist_to_goal(0), end_dist_to_goal(0) { }
	PotentialPoints(LineSegment p, CartesianPoint target){
		pair = p;
		start = pair.get_endpoints().first;
		end = pair.get_endpoints().second;
		dist_to_goal = distance(target, pair);
		start_dist_to_goal = start.get_distance(target);
		end_dist_to_goal = end.get_distance(target);
		printDetails();
	}
	void printDetails(){
		cout << "start " << start.get_x() << " " << start.get_y() << " end " << end.get_x() << " " << end.get_y() << " dist_to_goal " << dist_to_goal << " start_dist_to_goal " << start_dist_to_goal << " end_dist_to_goal " << end_dist_to_goal << endl;
	}
	bool operator==(const PotentialPoints p) {
		return (start == p.start and end == p.end);
	}
	bool operator < (const PotentialPoints p) const{
		if(dist_to_goal < p.dist_to_goal){
			return true;
		}
		else if(dist_to_goal > p.dist_to_goal){
			return false;
		}
		else{
			if(start_dist_to_goal < p.start_dist_to_goal or end_dist_to_goal < p.end_dist_to_goal){
				return true;
			}
			else{
				return false;
			}
		}
	}
};

class LocalExplorer{
public:
	LocalExplorer(){
		already_started = false;
		start_of_potential = false;
	};
	~LocalExplorer(){};
	bool getAlreadyStarted() { return already_started; }
	bool getStartOfPotential() { return start_of_potential; }
	void setQueue(CartesianPoint goal, vector< LineSegment > pairs){
		task = goal;
		potential_exploration = pairs;
		for(int i = 0; i < potential_exploration.size(); i++){
			potential_queue.push(PotentialPoints(potential_exploration[i], task));
		}
		current_potential = potential_queue.top();
		cout << "current_potential ";
		current_potential.printDetails();
		potential_queue.pop();
		already_started = true;
	}

private:
	bool already_started;
	CartesianPoint task;
	vector< LineSegment > potential_exploration;
	priority_queue<PotentialPoints> potential_queue;
	PotentialPoints current_potential;
	bool start_of_potential;
};

#endif