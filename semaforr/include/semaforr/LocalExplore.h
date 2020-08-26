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
		if((start == p.start and end == p.end) or end.get_distance(p.end) < 0.75){
			return true;
		}
		else{
			return false;
		}
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
		started_random = false;
		found_new_potentials = false;
		distance_threshold = 2;
	};
	~LocalExplorer(){};
	bool getAlreadyStarted() { return already_started; }
	bool getStartedRandom() { return started_random; }
	bool getAtStartOfPotential() { return start_of_potential; }
	bool getFinishedPotentials() {
		cout << "potential_queue " << potential_queue.size() << endl;
		if(potential_queue.size() > 0){
			bool picked_new = false;
			int count = potential_queue.size()-1;
			cout << "count " << count << endl;
			while(!picked_new and count > 0){
				current_potential = potential_queue.top();
				if(!alreadyInStack(current_potential)){
					picked_new = true;
				}
				potential_exploration.push_back(current_potential);
				potential_queue.pop();
				count = count - 1;
				cout << "potential_queue " << potential_queue.size() << " count " << count << " picked_new " << picked_new << endl;
			}
			if(picked_new == true){
				cout << "current_potential ";
				current_potential.printDetails();
				potential_queue.pop();
				finished_potentials = false;
			}
			else{
				finished_potentials = true;
			}
		}
		else{
			finished_potentials = true;
		}
		return finished_potentials;
	}
	void resetLocalExplorer(){
		already_started = false;
		start_of_potential = false;
		finished_potentials = false;
		started_random = false;
		found_new_potentials = false;
		task = CartesianPoint();
		potential_exploration.clear();
		potential_queue = priority_queue<PotentialPoints, vector<PotentialPoints>, greater<PotentialPoints> >();
		current_potential = PotentialPoints();
	}
	void setPathPlanner(PathPlanner *planner){
		pathPlanner = planner;
	}
	void setQueue(CartesianPoint goal, vector< LineSegment > pairs){
		cout << "inside setQueue " << pairs.size() << endl;
		task = goal;
		for(int i = 0; i < pairs.size(); i++){
			cout << "pair " << i << " " << pairs[i].get_length() << endl;
			if(pairs[i].get_length() >= distance_threshold){
				potential_queue.push(PotentialPoints(pairs[i], task));
			}
		}
		current_potential = potential_queue.top();
		potential_exploration.push_back(current_potential);
		cout << "current_potential ";
		current_potential.printDetails();
		potential_queue.pop();
		already_started = true;
	}
	void addToQueue(vector< LineSegment > pairs){
		cout << "inside addToQueue " << pairs.size() << endl;
		int count_before = potential_queue.size();
		for(int i = 0; i < pairs.size(); i++){
			cout << "pair " << i << " " << pairs[i].get_length() << endl;
			if(pairs[i].get_length() >= distance_threshold){
				potential_queue.push(PotentialPoints(pairs[i], task));
			}
		}
		int count_after = potential_queue.size();
		if(count_before == 0 and count_after > 0 and started_random == true){
			found_new_potentials = true;
		}
	}
	void atStartOfPotential(CartesianPoint current){
		if(current.get_distance(current_potential.start) < 0.75){
			start_of_potential = true;
		}
		else{
			start_of_potential = false;
		}
	}
	bool atEndOfPotential(CartesianPoint current){
		if(current.get_distance(current_potential.end) < 0.75 or found_new_potentials == true){
			found_new_potentials = false;
			return true;
		}
		else{
			return false;
		}
	}
	CartesianPoint getStartOfPotential(){
		return current_potential.start;
	}
	CartesianPoint getEndOfPotential(){
		return current_potential.end;
	}
	bool alreadyInStack(PotentialPoints segment){
		bool inCompleted = false;
		for(int i = 0; i < potential_exploration.size(); i++){
			if(segment == potential_exploration[i]){
				inCompleted = true;
				break;
			}
		}
		bool alreadyCovered = false;
		if(started_random){
			if(coverage[(int)(segment.end.get_x())][(int)(segment.end.get_y())] >= 0){
				alreadyCovered = true;
			}
		}
		if(inCompleted or alreadyCovered){
			return true;
		}
		else{
			return false;
		}
	}
	vector<CartesianPoint> getPathToStart(CartesianPoint current){
		cout << "in getPathToStart " << endl;
		pathPlanner->resetPath();
		vector<CartesianPoint> waypoints;
		cout << current.get_x() << " " << current.get_y() << endl;
		Node s(1, current.get_x()*100, current.get_y()*100);
		pathPlanner->setSource(s);
		cout << current_potential.start.get_x() << " " << current_potential.start.get_y() << endl;
		Node t(1, current_potential.start.get_x()*100, current_potential.start.get_y()*100);
		pathPlanner->setTarget(t);
		cout << "plan generation status" << pathPlanner->calcPath(true) << endl;
		list<int> waypointInd = pathPlanner->getPath();
		if(waypointInd.size() > 0){
			list<int>::iterator it;
			for ( it = waypointInd.begin(); it != waypointInd.end(); it++ ){
				// cout << "node " << (*it) << endl;
				double r_x = pathPlanner->getGraph()->getNode(*it).getX()/100.0;
				double r_y = pathPlanner->getGraph()->getNode(*it).getY()/100.0;
				// cout << r_x << " " << r_y << endl;
				CartesianPoint waypoint(r_x,r_y);
				waypoints.push_back(waypoint);
			}
		}
		else{
			waypoints.push_back(current);
		}
		waypoints.push_back(current_potential.start);
		pathPlanner->resetPath();
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
	vector<CartesianPoint> randomExploration(CartesianPoint current, vector<CartesianPoint> laserEndpoints, CartesianPoint goal, vector< vector<int> > coverage_grid){
		already_started = true;
		started_random = true;
		cout << "randomExploration " << current.get_x() << " " << current.get_y() << " " << goal.get_x() << " " << goal.get_y() << endl;
		task = goal;
		coverage = coverage_grid;
		cout << "coverage" << endl;
		for(int i = 0; i < coverage.size(); i++){
			for(int j = 0; j < coverage[i].size(); j++){
				cout << coverage[i][j] << " ";
			}
			cout << endl;
		}
		double dist_to_goal = task.get_distance(current);
		double max_search_radius = (int)(dist_to_goal)+1.0;
		double min_search_radius = 1.0;
		cout << "dist_to_goal " << dist_to_goal << " min_search_radius " << min_search_radius << " max_search_radius " << max_search_radius << endl;
		vector<double> search_radii;
		if(max_search_radius >= min_search_radius){
			for(double i = min_search_radius; i <= max_search_radius; i += 1.0){
				search_radii.push_back(i);
			}
		}
		else{
			search_radii.push_back(min_search_radius);
		}
		cout << "search_radii " << search_radii.size() << endl;
		vector<bool> search_access;
		vector<int> laser_index;
		for(int i = 0; i < search_radii.size(); i++){
			bool canAccessRegion = false;
			double distLaserPosToPoint = current.get_distance(task);
			if(distLaserPosToPoint - search_radii[i] > 20){
				cout << search_radii[i] << " 0" << endl;
				search_access.push_back(false);
				laser_index.push_back(-1);
			}
			else{
				int ind = -1;
				for(int j = 0; j < laserEndpoints.size(); j++){
					//ROS_DEBUG_STREAM("Laser endpoint : " << laserEndpoints[j].get_x() << "," << laserEndpoints[j].get_y());
					if(do_intersect(Circle(task, search_radii[i]), LineSegment(current, laserEndpoints[j]))){
						canAccessRegion = true;
						ind = j;
						break;
					}
				}
				cout << search_radii[i] << " " << canAccessRegion << endl;
				search_access.push_back(canAccessRegion);
				laser_index.push_back(ind);
			}
		}
		PotentialPoints laser_to_explore;
		bool found_closest = false;
		for(int i = 0; i < search_radii.size(); i++){
			if(search_access[i] and coverage[(int)(laserEndpoints[laser_index[i]].get_x())][(int)(laserEndpoints[laser_index[i]].get_y())] < 0){
				cout << "closest visible radii " << search_radii[i] << endl;
				laser_to_explore = PotentialPoints(LineSegment(current, laserEndpoints[laser_index[i]]), task);
				found_closest = true;
				break;
			}
		}
		if(found_closest == true){
			cout << "found_closest " << found_closest << endl;
			vector<CartesianPoint> waypoints;
			double tx, ty;
			for(double j = 0; j <= 1; j += 0.1){
				tx = (laser_to_explore.end.get_x() * j) + (laser_to_explore.start.get_x() * (1 - j));
				ty = (laser_to_explore.end.get_y() * j) + (laser_to_explore.start.get_y() * (1 - j));
				waypoints.push_back(CartesianPoint(tx, ty));
			}
			return waypoints;
		}
		else{
			cout << "go to closest_coverage" << endl;
			int min_distance = 10000;
			int task_x = (int)(task.get_x());
			int task_y = (int)(task.get_y());
			int closest_x, closest_y;
			for(int i = 0; i < coverage.size(); i++){
				for(int j = 0; j < coverage[i].size(); j++){
					int dist_to_task = abs(task_x - i) + abs(task_y - j);
					if(coverage[i][j] >= 0 and dist_to_task < min_distance){
						min_distance = dist_to_task;
						closest_x = i;
						closest_y = j;
					}
				}
			}
			cout << "closest_x " << closest_x << " closest_y " << closest_y << endl;
			PotentialPoints closest_coverage = PotentialPoints(LineSegment(CartesianPoint(closest_x, closest_y), CartesianPoint(closest_x, closest_y)), task);
			pathPlanner->resetPath();
			vector<CartesianPoint> waypoints;
			cout << current.get_x() << " " << current.get_y() << endl;
			Node s(1, current.get_x()*100, current.get_y()*100);
			pathPlanner->setSource(s);
			cout << closest_coverage.start.get_x() << " " << closest_coverage.start.get_y() << endl;
			Node t(1, closest_coverage.start.get_x()*100, closest_coverage.start.get_y()*100);
			pathPlanner->setTarget(t);
			cout << "plan generation status" << pathPlanner->calcPath(true) << endl;
			list<int> waypointInd = pathPlanner->getPath();
			if(waypointInd.size() > 0){
				list<int>::iterator it;
				for ( it = waypointInd.begin(); it != waypointInd.end(); it++ ){
					// cout << "node " << (*it) << endl;
					double r_x = pathPlanner->getGraph()->getNode(*it).getX()/100.0;
					double r_y = pathPlanner->getGraph()->getNode(*it).getY()/100.0;
					// cout << r_x << " " << r_y << endl;
					CartesianPoint waypoint(r_x,r_y);
					waypoints.push_back(waypoint);
				}
			}
			else{
				waypoints.push_back(current);
			}
			waypoints.push_back(closest_coverage.start);
			pathPlanner->resetPath();
			return waypoints;
		}
	}

	void updateCoverage(CartesianPoint current){
		if(started_random){
			cout << "updated Coverage " << (int)(current.get_x()) << " " << (int)(current.get_y()) << endl;
			coverage[(int)(current.get_x())][(int)(current.get_y())] = 0;
		}
	}

	vector< vector<int> > getCoverage() { return coverage; }

private:
	double distance_threshold;
	bool already_started;
	CartesianPoint task;
	vector< PotentialPoints > potential_exploration;
	priority_queue<PotentialPoints, vector<PotentialPoints>, greater<PotentialPoints> > potential_queue;
	PotentialPoints current_potential;
	bool start_of_potential;
	PathPlanner *pathPlanner;
	bool finished_potentials;
	bool started_random;
	vector< vector<int> > coverage;
	bool found_new_potentials;
};

#endif