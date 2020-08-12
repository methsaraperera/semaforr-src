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
#include "PathPlanner.h"
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
	bool operator > (const PotentialPoints p) const{
		if(dist_to_goal > p.dist_to_goal){
			return true;
		}
		else if(dist_to_goal < p.dist_to_goal){
			return false;
		}
		else{
			if(start_dist_to_goal > p.start_dist_to_goal or end_dist_to_goal > p.end_dist_to_goal){
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
		finished_potentials = false;
	};
	~LocalExplorer(){};
	bool getAlreadyStarted() { return already_started; }
	bool getStartOfPotential() { return start_of_potential; }
	bool getFinishedPotentials() { return finished_potentials; }
	void resetLocalExplorer(){
		already_started = false;
		start_of_potential = false;
		finished_potentials = false;
		task = CartesianPoint();
		potential_exploration.clear();
		potential_queue = priority_queue<PotentialPoints, vector<PotentialPoints>, greater<PotentialPoints> >();
		current_potential = PotentialPoints();
	}
	void setQueue(CartesianPoint goal, vector< LineSegment > pairs, PathPlanner *planner){
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
		pathPlanner = planner;
	}
	void atStartOfPotential(CartesianPoint current){
		if(current.get_distance(current_potential.start) < 0.75){
			start_of_potential = true;
		}
	}
	bool atEndOfPotential(CartesianPoint current){
		if(current.get_distance(current_potential.end) < 0.75){
			if(potential_queue.size() > 0){
				current_potential = potential_queue.top();
				cout << "current_potential ";
				current_potential.printDetails();
				potential_queue.pop();
			}
			else{
				finished_potentials = true;
			}
			return true;
		}
		else{
			return false;
		}
	}
	vector<CartesianPoint> getPathToStart(CartesianPoint current){
		vector<CartesianPoint> waypoints;
		Node s(1, current.get_x()*100, current.get_y()*100);
		pathPlanner->setSource(s);
		Node t(1, current_potential.start.get_x()*100, current_potential.start.get_y()*100);
		pathPlanner->setTarget(t);
		cout << "plan generation status" << pathPlanner->calcPath(true) << endl;
		list<int> waypointInd = pathPlanner->getPath();
		list<int>::iterator it;
		for ( it = waypointInd.begin(); it != waypointInd.end(); it++ ){
			// cout << "node " << (*it) << endl;
			double r_x = pathPlanner->getGraph()->getNode(*it).getX()/100.0;
			double r_y = pathPlanner->getGraph()->getNode(*it).getY()/100.0;
			// cout << r_x << " " << r_y << endl;
			CartesianPoint waypoint(r_x,r_y);
			waypoints.push_back(waypoint);
		}
		return waypoints;
	}

	vector<CartesianPoint> getPathToEnd(){
		vector<CartesianPoint> waypoints;
		double tx, ty;
		for(double j = 0; j <= 1; j += 0.1){
			tx = (current_potential.end.get_x() * j) + (current_potential.start.get_x() * (1 - j));
			ty = (current_potential.end.get_y() * j) + (current_potential.start.get_y() * (1 - j));
			waypoints.push_back(CartesianPoint(tx, ty));
		}
		return waypoints;
	}

private:
	bool already_started;
	CartesianPoint task;
	vector< LineSegment > potential_exploration;
	priority_queue<PotentialPoints, vector<PotentialPoints>, greater<PotentialPoints> > potential_queue;
	PotentialPoints current_potential;
	bool start_of_potential;
	PathPlanner *pathPlanner;
	bool finished_potentials;
};

#endif