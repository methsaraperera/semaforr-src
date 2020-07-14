/**!
  * Task.h
  * 
  * /author: Anoop Aroor
  *
  *          Defines tasks as simple target and stores its status and statistics of completion
  */

#ifndef TASK_H
#define TASK_H

#include "FORRAction.h"
#include "FORRGeometry.h"
#include "Position.h"
#include "PathPlanner.h"
#include <vector>
#include <map>
#include <algorithm>
#include <iostream>
#include <fstream>
#include <sensor_msgs/LaserScan.h>

/*
 * Struct for skeleton waypoint.
 */
struct sk_waypoint {
	bool isRegion;
    FORRRegion region;
    vector<CartesianPoint> path;
    vector<CartesianPoint> original_path;
    sk_waypoint(): isRegion(false), region(), path(), original_path() { }
    sk_waypoint(bool b, FORRRegion r, vector<CartesianPoint> p): isRegion(b), region(r), path(p), original_path(p) { }

    bool waypointIsRegion() { return isRegion; }
    FORRRegion getRegion() { return region; }
    vector<CartesianPoint> getPath() { return path; }
    CartesianPoint getPathEnd() { return path[path.size()-1]; }
    void setPath(vector<CartesianPoint> np) { path = np; }
};

class Task {
  
 public:
  
  Task(double x_in, double y_in)  
    {
      x = x_in;
      wx = x_in; // current waypoint x
      y = y_in;  
      wy = y_in; // current waypoint y
      wr = sk_waypoint();
      decision_count = 0;
      isPlanActive = false;
      isPlanComplete = true;
      plannerName = "none";
      decisionSequence = new std::vector<FORRAction>;
      pos_hist = new vector<Position>();
      laser_hist = new vector< vector<CartesianPoint> >();
      laser_scan_hist = new vector< sensor_msgs::LaserScan >();
      int dimension = 200;
      for(int i = 0; i < dimension; i++){
      	vector<int> col;
      	for(int j = 0; j < dimension; j++){
      		col.push_back(0);
      	}
      	planPositions.push_back(col);
      }
    }

  double getTaskX(){ return x;}
  double getTaskY(){ return y;}

  double getX() { 
	if(isPlanActive == false){
		return x;
	}
	else{
		if(plannerName != "skeleton"){
			return wx;
		}
		else{
			if(wr.waypointIsRegion()){
				return wr.getRegion().getCenter().get_x();
			}
			else{
				wr.getPathEnd().get_x();
			}
		}
	}
  }
  
  double getY(){
	if(isPlanActive == false){
		return y; 
	}
	else{
		if(plannerName != "skeleton"){
			return wy;
		}
		else{
			if(wr.waypointIsRegion()){
				return wr.getRegion().getCenter().get_y();
			}
			else{
				wr.getPathEnd().get_y();
			}
		}
	}
  }

  sk_waypoint getSkeletonWaypoint(){
	return wr;
  }

  vector<sk_waypoint> getSkeletonWaypoints(){
  	return skeleton_waypoints;
  }

  bool getIsPlanActive(){return isPlanActive;}
  void setIsPlanActive(bool status){isPlanActive = status;}
  bool getIsPlanComplete(){return isPlanComplete;}
  
  int getDecisionCount(){return decision_count;} 
 
  int incrementDecisionCount() {decision_count += 1;}

  std::vector<FORRAction> getPreviousDecisions(){
	return *decisionSequence;
  }

  FORRAction saveDecision(FORRAction decision){
	decisionSequence->push_back(decision);
	//cout << "After decisionToPush" << endl;
  }

  vector<Position> *getPositionHistory(){return pos_hist;}

  void clearPositionHistory(){pos_hist->clear();}

  void saveSensor(Position currentPosition, vector<CartesianPoint> laserEndpoints, sensor_msgs::LaserScan ls){
  	pos_hist->push_back(currentPosition);
  	laser_hist->push_back(laserEndpoints);
  	laser_scan_hist->push_back(ls);
	// if(pos_hist->size() < 1){
	// 	pos_hist->push_back(currentPosition);
	// 	laser_hist->push_back(laserEndpoints);
	// }
	// else{	
 //     		Position pos = pos_hist->back();
 //     		if(!(pos == currentPosition)) {
	// 		pos_hist->push_back(currentPosition);
	// 		laser_hist->push_back(laserEndpoints);
	// 	}
	// }
  }

  vector< vector <CartesianPoint> > *getLaserHistory(){return laser_hist;}

  vector< sensor_msgs::LaserScan > *getLaserScanHistory(){return laser_scan_hist;}

  vector<CartesianPoint> getWaypoints(){
  	// cout << "in getWaypoints" << endl;
  	if(plannerName != "skeleton"){
  		return waypoints;
  	}
  	else{
  		vector<CartesianPoint> points;
  		// cout << "number of skeleton_waypoints " << skeleton_waypoints.size() << endl;
  		for(int i = 0; i < skeleton_waypoints.size(); i++){
  			if(skeleton_waypoints[i].waypointIsRegion()){
  				points.push_back(skeleton_waypoints[i].getRegion().getCenter());
  				// cout << "region waypoint " << i << " " << skeleton_waypoints[i].getRegion().getCenter().get_x() << " " << skeleton_waypoints[i].getRegion().getCenter().get_y() << " " << skeleton_waypoints[i].getRegion().getRadius() << endl;
  			}
  			else{
  				vector<CartesianPoint> pathBetween = skeleton_waypoints[i].getPath();
  				for(int j = 0; j < pathBetween.size(); j++){
  					points.push_back(pathBetween[j]);
  					// cout << "path waypoint " << i << " " << j << " " << pathBetween[j].get_x() << " " << pathBetween[j].get_y() << endl;
  				}
  			}
  		}
  		// cout << "number of points " << points.size() << endl;
  		return points;
  	}
  }
  vector<CartesianPoint> getOrigWaypoints(){return origWaypoints;}
  int getPlanSize(){
  	if(plannerName != "skeleton"){
  		return waypoints.size();
  	}
  	else{
  		return skeleton_waypoints.size();
  	}
  }

  string getPlannerName(){ return plannerName; }

  void clearWaypoints(){
  	waypoints.clear();
  	tierTwoWaypoints.clear();
  	origWaypoints.clear();
  }

  // list<int> getWaypointInds(){return waypointInd;}
  vector< list<int> > getPlansInds(){return plansInds;}

  // generates new waypoints given currentposition and a planner
  bool generateWaypoints(Position source, PathPlanner *planner){
	waypoints.clear();
	tierTwoWaypoints.clear();
	//a_star planner works in cms so all units are converts into cm
	//once plan is generated waypoints are stored in meters
	Node s(1, source.getX()*100, source.getY()*100);
	planner->setSource(s);
	Node t(1, x*100, y*100);
	planner->setTarget(t);

	cout << "plan generation status" << planner->calcPath(true) << endl;

	// waypointInd = planner->getPath();
	plansInds = planner->getPaths();
	// Graph *navGraph = planner->getGraph();
	// if(waypointInd.size() > 0){
	// 	isPlanActive = true;
	// 	isPlanComplete = false;
	// }
	// else{
	// 	isPlanActive = false;
	// }
	/*list<int>::iterator it;
	for ( it = waypointInd.begin(); it != waypointInd.end(); it++ ){
		double x = navGraph->getNode(*it).getX()/100.0;
		double y = navGraph->getNode(*it).getY()/100.0;
		cout << x << " " << y << endl;
		CartesianPoint waypoint(x,y);
		waypoints.push_back(waypoint);
		//if atleast one point is generated
		cout << "Plan active is true" << endl;
		isPlanActive = true;
	}
	setupNextWaypoint(source);*/
	planner->resetPath();
	cout << "plan generation complete" << endl;
  }

  bool generateOriginalWaypoints(Position source, PathPlanner *planner){
	origWaypoints.clear();
	//a_star planner works in cms so all units are converts into cm
	//once plan is generated waypoints are stored in meters
	Node s(1, source.getX()*100, source.getY()*100);
	planner->setSource(s);
	Node t(1, x*100, y*100);
	planner->setTarget(t);

	cout << "plan generation status" << planner->calcOrigPath(true) << endl;

	list<int> path = planner->getOrigPath();
	navGraph = planner->getOrigGraph();
	list<int>::iterator it;
	for ( it = path.begin(); it != path.end(); it++ ){
		double x = navGraph->getNode(*it).getX()/100.0;
		double y = navGraph->getNode(*it).getY()/100.0;
		CartesianPoint waypoint(x,y);
		origWaypoints.push_back(waypoint);
  	}
  	origPathCostInOrigNavGraph = planner->getOrigPathCost();
  	origPathCostInNavGraph = planner->calcPathCost(path);
  	/*vector<CartesianPoint> skippedwaypoints;
	for(int i = 0; i < origWaypoints.size(); i+=4){
		skippedwaypoints.push_back(origWaypoints[i]);
	}
	origWaypoints = skippedwaypoints;*/
	planner->resetOrigPath();
	cout << "plan generation complete" << endl;
  }

  bool generateWaypointsFromInds(Position source, vector<CartesianPoint> currentLaserEndpoints, PathPlanner *planner, list<int> indices){
	waypoints.clear();
	tierTwoWaypoints.clear();
	//a_star planner works in cms so all units are converts into cm
	//once plan is generated waypoints are stored in meters
	//Node s(1, source.getX()*100, source.getY()*100);
	//planner->setSource(s);
	//Node t(1, x*100, y*100);
	//planner->setTarget(t);

	//cout << "plan generation status" << planner->calcPath(true) << endl;
	waypointInd = indices;
	navGraph = planner->getGraph();
	plannerName = planner->getName();
	if(plannerName != "skeleton"){
		list<int>::iterator it;
		for ( it = waypointInd.begin(); it != waypointInd.end(); it++ ){
			cout << "node " << (*it) << endl;
			double r_x = navGraph->getNode(*it).getX()/100.0;
			double r_y = navGraph->getNode(*it).getY()/100.0;
			cout << r_x << " " << r_y << endl;
			CartesianPoint waypoint(r_x,r_y);
			waypoints.push_back(waypoint);
			//if atleast one point is generated
			cout << "Plan active is true" << endl;
			isPlanActive = true;
			isPlanComplete = false;
		}
		pathCostInNavGraph = planner->getPathCost();
		vector<CartesianPoint> skippedwaypoints;
		for(int i = 0; i < waypoints.size(); i+=1){
			skippedwaypoints.push_back(waypoints[i]);
		}
		waypoints = skippedwaypoints;
		tierTwoWaypoints = skippedwaypoints;
		pathCostInNavOrigGraph = planner->calcOrigPathCost(waypointInd);
	}
	else{
		int step = -1;
		int max_step = waypointInd.size()-1;
		list<int>::iterator it;
		for ( it = waypointInd.begin(); it != waypointInd.end(); it++ ){
			step = step + 1;
			// cout << "node " << (*it) << " step " << step << endl;
			double r_x = navGraph->getNode(*it).getX()/100.0;
			double r_y = navGraph->getNode(*it).getY()/100.0;
			double r = navGraph->getNode(*it).getRadius();
			// cout << r_x << " " << r_y << " " << r << endl;
			skeleton_waypoints.push_back(sk_waypoint(true, FORRRegion(CartesianPoint(r_x,r_y), r), vector<CartesianPoint>()));
			list<int>::iterator itr1; 
			itr1 = it; 
			advance(itr1, 1);
			int forward_step = step + 1;
			// cout << "max_step " << max_step << " forward_step " << forward_step << endl;
			if(forward_step <= max_step){
				// if(navGraph->getEdge(*it, *itr1)->getFrom() == *it){
				// 	skeleton_waypoints.push_back(sk_waypoint(false, FORRRegion(), navGraph->getEdge(*it, *itr1)->getEdgePath(true)));
				// }
				// else if(navGraph->getEdge(*it, *itr1)->getFrom() == *itr1){
				// 	skeleton_waypoints.push_back(sk_waypoint(false, FORRRegion(), navGraph->getEdge(*it, *itr1)->getEdgePath(false)));
				// }
				vector<CartesianPoint> path_from_edge = navGraph->getEdge(*it, *itr1)->getEdgePath(true);
				if(FORRRegion(CartesianPoint(r_x,r_y), r).inRegion(path_from_edge[0])){
					skeleton_waypoints.push_back(sk_waypoint(false, FORRRegion(), path_from_edge));
				}
				else{
					std::reverse(path_from_edge.begin(),path_from_edge.end());
					skeleton_waypoints.push_back(sk_waypoint(false, FORRRegion(), path_from_edge));
				}
			}
			// cout << "num of waypoints " << skeleton_waypoints.size() << endl;
		}
		if(skeleton_waypoints.size() > 0){
			cout << "Plan active is true" << endl;
			isPlanActive = true;
			isPlanComplete = false;
		}
		pathCostInNavGraph = planner->getPathCost();
		pathCostInNavOrigGraph = 0;
	}
	//setupNextWaypoint(source);
	setupNearestWaypoint(source, currentLaserEndpoints);
	//planner->resetPath();
	//cout << "plan generation complete" << endl;
  }


   double planCost(vector<CartesianPoint> waypoints, PathPlanner *planner, Position source, Position target){
   	double cost = planner->calcPathCost(waypoints, source, target);
   	return cost;
   }

 //   void setupNextWaypoint(Position currentPosition){
 //   	cout << "inside setup next waypoint" << endl;
	// double dis;
	// while(waypoints.size() >= 1){
	// 	cout << "inside while" << endl;
	// 	wx = waypoints[0].get_x();
	// 	wy = waypoints[0].get_y();
	// 	dis = currentPosition.getDistance(wx, wy);
	// 	if(dis < 0.75){
	// 		cout << "found waypoint with dist < 0.75" << endl;
	// 		waypoints.erase(waypoints.begin());
	// 	}
	// 	else{
	// 		break;
	// 	} 	
	// }
	// cout << "check plan active: " << waypoints.size() << endl;
	// if(waypoints.size() > 0){
	// 	isPlanActive = true;
	// }
	// else{
	// 	isPlanActive = false;
	// }
	// cout << "end setup next waypoint" << endl;
 //   }

	void createNewWaypoint(CartesianPoint new_point, bool addAnyway){
		bool found = false;
		for(int i = 0; i < waypoints.size(); i++){
			if(waypoints[i] == new_point){
				found = true;
				break;
			}
		}
		if(found == false or addAnyway){
			waypoints.insert(waypoints.begin(), new_point);
			// waypoints.push_back(new_point);
			isPlanActive = true;
			wx = new_point.get_x();
			wy = new_point.get_y();
			cout << wx << " " << wy << endl;
		}
	}

   void setupNearestWaypoint(Position currentPosition, vector<CartesianPoint> currentLaserEndpoints){
   	// cout << "inside setup nearest waypoint" << endl;
   	if(plannerName != "skeleton"){
		double dis;
		int farthest = -1;
		// cout << "waypoints size: " << waypoints.size() << endl;
		for (int i = 0; i < waypoints.size(); i++){
			dis = currentPosition.getDistance(waypoints[i].get_x(), waypoints[i].get_y());
			if(dis < 0.75){
				// cout << "found waypoint with dist < 0.75: " << i << endl;
				farthest = i;
			}
		}
		if(farthest == 0){
			if(waypoints.begin() == tierTwoWaypoints.begin()){
				tierTwoWaypoints.erase(tierTwoWaypoints.begin());
			}
			waypoints.erase(waypoints.begin());
		}
		else if(farthest > 0){
			for(int i = 0; i <= farthest; i++){
				for(int j = 0; j < tierTwoWaypoints.size(); j++){
					if(waypoints[i] == tierTwoWaypoints[j]){
						tierTwoWaypoints.erase(tierTwoWaypoints.begin()+j);
						j--;
					}
				}
			}
			waypoints.erase(waypoints.begin(), waypoints.begin()+farthest);
		}
		wx = waypoints[0].get_x();
		wy = waypoints[0].get_y();
		//cout << "check plan active: " << waypoints.size() << endl;
		if(waypoints.size() > 0){
			isPlanActive = true;
		}
		else{
			isPlanActive = false;
		}
		if(tierTwoWaypoints.size() == 0){
			isPlanComplete = true;
		}
	}
	else{
		int farthest = -1;
		int farthest_path = -1;
		// cout << "skeleton_waypoints size: " << skeleton_waypoints.size() << " finished_sk_waypoints size: " << finished_sk_waypoints.size() << endl;
		for (int i = 0; i < skeleton_waypoints.size(); i++){
			if(skeleton_waypoints[i].waypointIsRegion()){
				// cout << "region " << i << endl;
				if(skeleton_waypoints[i].getRegion().inRegion(CartesianPoint(currentPosition.getX(), currentPosition.getY())) and skeleton_waypoints[i].getRegion().getCenter().get_distance(CartesianPoint(currentPosition.getX(), currentPosition.getY())) < 0.75){
					// cout << "found skeleton region waypoint: " << i << endl;
					farthest = i;
				}
			}
			else{
				vector<CartesianPoint> pathBetween = skeleton_waypoints[i].getPath();
				double dis;
				// cout << "pathBetween " << i  << " size: " << pathBetween.size() << endl;
				int farthest_here = -1;
				for (int j = 0; j < pathBetween.size()-1; j++){
					dis = currentPosition.getDistance(pathBetween[j].get_x(), pathBetween[j].get_y());
					if(dis < 0.5 and canAccessPoint(currentLaserEndpoints, CartesianPoint(currentPosition.getX(), currentPosition.getY()), pathBetween[j+1], 20)){
						// cout << "found pathBetween with dist < 0.5: " << j << endl;
						farthest_here = j;
					}
				}
				if(farthest_here > -1 or currentPosition.getDistance(pathBetween[pathBetween.size()-1].get_x(), pathBetween[pathBetween.size()-1].get_y()) < 0.5){
					// cout << "found skeleton region waypoint: " << i << " farthest_here " << farthest_here << endl;
					farthest = i;
					farthest_path = farthest_here;
				}
			}
		}
		// cout << "farthest " << farthest << " farthest_path " << farthest_path << endl;
		if(farthest == 0 and (skeleton_waypoints[0].waypointIsRegion() or (!skeleton_waypoints[0].waypointIsRegion() and farthest_path == skeleton_waypoints[0].getPath().size()-1))){
			// cout << "eliminate first" << endl;
			finished_sk_waypoints.push_back(skeleton_waypoints[0]);
			skeleton_waypoints.erase(skeleton_waypoints.begin());
		}
		else if(farthest == 0){
			// cout << "eliminate first path up to farthest_path" << endl;
			vector<CartesianPoint> new_path;
			for(int i = farthest_path+1; i < skeleton_waypoints[0].getPath().size(); i++){
				new_path.push_back(skeleton_waypoints[0].getPath()[i]);
			}
			skeleton_waypoints[0].setPath(new_path);
		}
		else if(farthest > 0 and skeleton_waypoints[farthest].waypointIsRegion()){
			// cout << "eliminate up to farthest region" << endl;
			for(int i = 0; i <= farthest; i++){
				finished_sk_waypoints.push_back(skeleton_waypoints[i]);
			}
			skeleton_waypoints.erase(skeleton_waypoints.begin(), skeleton_waypoints.begin()+farthest);
		}
		else if(farthest > 0 and !skeleton_waypoints[farthest].waypointIsRegion()){
			// cout << "eliminate up to farthest path up to farthest_path" << endl;
			if(farthest_path == skeleton_waypoints[farthest].getPath().size()-1){
				// cout << "farthest_path is equal to last in path " << skeleton_waypoints[farthest].getPath().size()-1 << endl;
				for(int i = 0; i <= farthest; i++){
					finished_sk_waypoints.push_back(skeleton_waypoints[i]);
				}
				skeleton_waypoints.erase(skeleton_waypoints.begin(), skeleton_waypoints.begin()+farthest);
			}
			else{
				// cout << "eliminate up to one before farthest path" << endl;
				for(int i = 0; i <= farthest-1; i++){
					finished_sk_waypoints.push_back(skeleton_waypoints[i]);
				}
				skeleton_waypoints.erase(skeleton_waypoints.begin(), skeleton_waypoints.begin()+(farthest-1));
				vector<CartesianPoint> new_path;
				for(int i = farthest_path+1; i < skeleton_waypoints[0].getPath().size(); i++){
					new_path.push_back(skeleton_waypoints[0].getPath()[i]);
				}
				skeleton_waypoints[0].setPath(new_path);
			}
		}
		// cout << "skeleton_waypoints size: " << skeleton_waypoints.size() << " finished_sk_waypoints size: " << finished_sk_waypoints.size() << endl;
		wr = skeleton_waypoints[0];
		cout << "new current waypoint " << this->getX() << " " << this->getY() << endl;
		cout << "check plan active: " << skeleton_waypoints.size() << endl;
		if(skeleton_waypoints.size() > 0){
			isPlanActive = true;
		}
		else{
			isPlanActive = false;
		}
	}
	//cout << "end setup next waypoint" << endl;
   }

  
  bool isTaskComplete(Position currentPosition){
	bool status = false;
	double dis = currentPosition.getDistance(x, y);
	if (dis < 1){
		status = true;
	}
	return status;
  }


 //   bool isWaypointComplete(Position currentPosition){
	// bool status = false;
	// double dis = currentPosition.getDistance(wx, wy);
	// if (isPlanActive && (dis < 0.75)){
	// 	status = true;
	// }
	// return status;
 //   }

   bool isAnyWaypointComplete(Position currentPosition, vector<CartesianPoint> currentLaserEndpoints){
	bool status = false;
	if(isPlanActive && plannerName != "skeleton"){
		for (int i = 0; i < waypoints.size(); i++){
			double dis = currentPosition.getDistance(waypoints[i].get_x(), waypoints[i].get_y());
			if ((dis < 0.75)){
				status = true;
				break;
			}
		}
	}
	else if(isPlanActive){
		for (int i = 0; i < skeleton_waypoints.size(); i++){
			if(skeleton_waypoints[i].waypointIsRegion()){
				if(skeleton_waypoints[i].getRegion().inRegion(CartesianPoint(currentPosition.getX(), currentPosition.getY())) and skeleton_waypoints[i].getRegion().getCenter().get_distance(CartesianPoint(currentPosition.getX(), currentPosition.getY())) < 0.75){
					status = true;
					break;
				}
			}
			else{
				vector<CartesianPoint> pathBetween = skeleton_waypoints[i].getPath();
				double dis;
				int farthest_path = -1;
				// cout << "pathBetween size: " << pathBetween.size() << endl;
				for (int j = 0; j < pathBetween.size()-1; j++){
					dis = currentPosition.getDistance(pathBetween[j].get_x(), pathBetween[j].get_y());
					if(dis < 0.5 and canAccessPoint(currentLaserEndpoints, CartesianPoint(currentPosition.getX(), currentPosition.getY()), pathBetween[j+1], 20)){
						// cout << "found pathBetween with dist < 0.5: " << j << endl;
						farthest_path = j;
					}
				}
				if(farthest_path > -1 or currentPosition.getDistance(pathBetween[pathBetween.size()-1].get_x(), pathBetween[pathBetween.size()-1].get_y()) < 0.5){
					status = true;
					break;
				}
			}
		}
	}
	return status;
   }

  double getPathCostInNavGraph(){return pathCostInNavGraph;}
  double getPathCostInNavOrigGraph(){return pathCostInNavOrigGraph;}
  double getOrigPathCostInOrigNavGraph(){return origPathCostInOrigNavGraph;}
  double getOrigPathCostInNavGraph(){return origPathCostInNavGraph;}

  void updatePlanPositions(double p_x, double p_y){
  	cout << "In updatePlanPositions p_x " << p_x << " p_y " << p_y << endl;
  	int floor_x = (int)(floor(p_x))-1;
  	int floor_y = (int)(floor(p_y))-1;
  	int ceil_x = (int)(ceil(p_x))+1;
  	int ceil_y = (int)(ceil(p_y))+1;
  	if(floor_x < 0)
  		floor_x = 0;
  	if(floor_y < 0)
  		floor_y = 0;
  	if(ceil_x >= planPositions[0].size())
  		ceil_x = planPositions[0].size()-1;
  	if(ceil_y >= planPositions[0].size())
  		ceil_y = planPositions[0].size()-1;
  	cout << "Updating planPositions floor " << floor_x << " " << floor_y << " ceil " << ceil_x << " " << ceil_y << endl;
  	for(int i = floor_x; i <= ceil_x; i++){
  		for(int j = floor_y; j <= ceil_y; j++){
  			planPositions[i][j] = 1;
  		}
  	}
  	// if(floor_x >= 0 and floor_y >= 0 and floor_x < planPositions[0].size() and floor_y < planPositions[0].size()){
  	// 	planPositions[floor_x][floor_y] = 1;
  	// }
  	// if(floor_x >= 0 and ceil_y >= 0 and floor_x < planPositions[0].size() and ceil_y < planPositions[0].size()){
  	// 	planPositions[floor_x][ceil_y] = 1;
  	// }
  	// if(ceil_x >= 0 and floor_y >= 0 and ceil_x < planPositions[0].size() and floor_y < planPositions[0].size()){
  	// 	planPositions[ceil_x][floor_y] = 1;
  	// }
  	// if(ceil_x >= 0 and ceil_y >= 0 and ceil_x < planPositions[0].size() and ceil_y < planPositions[0].size()){
  	// 	planPositions[ceil_x][ceil_y] = 1;
  	// }
  }

  bool getPlanPositionValue(double p_x, double p_y){
  	cout << "In getPlanPositionValue" << endl;
  	int floor_x = (int)(floor(p_x));
  	int floor_y = (int)(floor(p_y));
  	int ceil_x = (int)(ceil(p_x));
  	int ceil_y = (int)(ceil(p_y));
  	if(floor_x >= 0 and floor_y >= 0 and floor_x < planPositions[0].size() and floor_y < planPositions[0].size()){
  		// cout << "planPositions[" << floor_x << "][" << floor_y << "] = " << planPositions[floor_x][floor_y] << endl;
  		if(planPositions[floor_x][floor_y] == 1)
  			return true;
  	}
  	if(floor_x >= 0 and ceil_y >= 0 and floor_x < planPositions[0].size() and ceil_y < planPositions[0].size()){
  		// cout << "planPositions[" << floor_x << "][" << ceil_y << "] = " << planPositions[floor_x][ceil_y] << endl;
  		if(planPositions[floor_x][ceil_y] == 1)
  			return true;
  	}
  	if(ceil_x >= 0 and floor_y >= 0 and ceil_x < planPositions[0].size() and floor_y < planPositions[0].size()){
  		// cout << "planPositions[" << ceil_x << "][" << floor_y << "] = " << planPositions[ceil_x][floor_y] << endl;
  		if(planPositions[ceil_x][floor_y] == 1)
  			return true;
  	}
  	if(ceil_x >= 0 and ceil_y >= 0 and ceil_x < planPositions[0].size() and ceil_y < planPositions[0].size()){
  		// cout << "planPositions[" << ceil_x << "][" << ceil_y << "] = " << planPositions[ceil_x][ceil_y] << endl;
  		if(planPositions[ceil_x][ceil_y] == 1)
  			return true;
  	}
  	return false;
  }

  void resetPlanPositions(){
  	cout << "In resetPlanPositions" << endl;
  	vector< vector<int> > grid;
  	int dimension = 200;
	for(int i = 0; i < dimension; i++){
		vector<int> col;
		for(int j = 0; j < dimension; j++){
			col.push_back(0);
		}
		grid.push_back(col);
	}
	planPositions = grid;
  }

  void setPassageValues(vector< vector<int> > pg, map<int, vector< vector<int> > > pgn, map<int, vector< vector<int> > > pge, vector< vector<int> > pgr){
  	passage_grid = pg;
  	passage_graph_nodes = pgn;
  	passage_graph_edges = pge;
  	passage_graph = pgr;
  }

  vector< vector<int> > getPassageGrid(){
  	return passage_grid;
  }

  map<int, vector< vector<int> > > getPassageGraphNodes(){
  	return passage_graph_nodes;
  }

  vector< vector<int> > getPassageGraph(){
  	return passage_graph;
  }
   
 private:
  
  // Current plan generated by A*, stored as waypoints , index 0 being the beginning of the plan
  vector<CartesianPoint> waypoints;
  vector<CartesianPoint> tierTwoWaypoints;
  vector<CartesianPoint> origWaypoints;
  double pathCostInNavGraph, pathCostInNavOrigGraph;
  double origPathCostInOrigNavGraph, origPathCostInNavGraph;

  vector<sk_waypoint> skeleton_waypoints;
  vector<sk_waypoint> finished_sk_waypoints;

  vector< vector<int> > passage_grid;
  map<int, vector< vector<int> > > passage_graph_nodes;
  map<int, vector< vector<int> > > passage_graph_edges;
  vector< vector<int> > passage_graph;

  list<int> waypointInd;
  vector< list<int> > plansInds;

  CartesianPoint currentWaypoint;

  bool isPlanActive;
  bool isPlanComplete;
  string plannerName;
  Graph *navGraph;
  
  //<! expected task execution time in seconds 
  float time_taken;

  // distance covered by the robot to get to the target
  float distance_travelled; 

  // Sequence of decisions made while pursuing the target
  std::vector<FORRAction> *decisionSequence;

  // Position History as is: Set of all unique positions the robot has been in , while pursuing the target
  std::vector<Position> *pos_hist; 

  // Laser scan history as is:
  vector< vector<CartesianPoint> > *laser_hist; 

  // Laser scan history sensor
  vector< sensor_msgs::LaserScan > *laser_scan_hist;

  // Cleaned Position History, along with its corresponding laser scan data : Set of cleaned positions
  std::pair < std::vector<CartesianPoint>, std::vector<vector<CartesianPoint> > > *cleaned_trail;

  // decision count
  int decision_count;

  // t1, t2, t3 counters
  int tier1_decisions, tier2_decisions, tier3_decisions;

  //<! The point in the map, that the robot needs to go in order to execute this task 
  double x,y,wx,wy;
  sk_waypoint wr;

  // Plan positions
  vector< vector<int> > planPositions;

};

#endif
