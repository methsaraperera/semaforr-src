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

class Task {
  
 public:
  
  Task(double x_in, double y_in)  
    {
      x = x_in;
      wx = x_in; // current waypoint x
      y = y_in;  
      wy = y_in; // current waypoint y
      decision_count = 0;
      isPlanActive = false;
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
		return wx;
	}
  }
  
  double getY(){
	if(isPlanActive == false){
		return y; 
	}
	else{
		return wy;
	}
  }

  bool getIsPlanActive(){return isPlanActive;}
  void setIsPlanActive(bool status){isPlanActive = status;}
  
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

  vector<CartesianPoint> getWaypoints(){return waypoints;}
  vector<CartesianPoint> getOrigWaypoints(){return origWaypoints;}

  void clearWaypoints(){
  	waypoints.clear();
  }

  list<int> getWaypointInds(){return waypointInd;}
  vector< list<int> > getPlansInds(){return plansInds;}

  // generates new waypoints given currentposition and a planner
  bool generateWaypoints(Position source, PathPlanner *planner){
	waypoints.clear();
	//a_star planner works in cms so all units are converts into cm
	//once plan is generated waypoints are stored in meters
	Node s(1, source.getX()*100, source.getY()*100);
	planner->setSource(s);
	Node t(1, x*100, y*100);
	planner->setTarget(t);

	cout << "plan generation status" << planner->calcPath(true) << endl;

	waypointInd = planner->getPath();
	plansInds = planner->getPaths();
	Graph *navGraph = planner->getGraph();
	if(waypointInd.size() > 0){
		isPlanActive = true;
	}
	else{
		isPlanActive = false;
	}
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
	Graph *navGraph = planner->getOrigGraph();
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

  bool generateWaypointsFromInds(Position source, PathPlanner *planner, list<int> indices){
	waypoints.clear();
	//a_star planner works in cms so all units are converts into cm
	//once plan is generated waypoints are stored in meters
	//Node s(1, source.getX()*100, source.getY()*100);
	//planner->setSource(s);
	//Node t(1, x*100, y*100);
	//planner->setTarget(t);

	//cout << "plan generation status" << planner->calcPath(true) << endl;
	waypointInd = indices;
	Graph *navGraph = planner->getGraph();
	list<int>::iterator it;
	for ( it = waypointInd.begin(); it != waypointInd.end(); it++ ){
		cout << "node " << (*it) << endl;
		double x = navGraph->getNode(*it).getX()/100.0;
		double y = navGraph->getNode(*it).getY()/100.0;
		cout << x << " " << y << endl;
		CartesianPoint waypoint(x,y);
		waypoints.push_back(waypoint);
		//if atleast one point is generated
		cout << "Plan active is true" << endl;
		isPlanActive = true;
	}
	pathCostInNavGraph = planner->getPathCost();
	if(planner->getName() != "skeleton"){
		pathCostInNavOrigGraph = planner->calcOrigPathCost(waypointInd);
	}
	else{
		pathCostInNavOrigGraph = 0;
	}
	vector<CartesianPoint> skippedwaypoints;
	for(int i = 0; i < waypoints.size(); i+=1){
		skippedwaypoints.push_back(waypoints[i]);
	}
	waypoints = skippedwaypoints;
	//setupNextWaypoint(source);
	setupNearestWaypoint(source);
	//planner->resetPath();
	//cout << "plan generation complete" << endl;
  }


   double planCost(vector<CartesianPoint> waypoints, PathPlanner *planner, Position source, Position target){
   	double cost = planner->calcPathCost(waypoints, source, target);
   	return cost;
   }

   void setupNextWaypoint(Position currentPosition){
   	cout << "inside setup next waypoint" << endl;
	double dis;
	while(waypoints.size() >= 1){
		cout << "inside while" << endl;
		wx = waypoints[0].get_x();
		wy = waypoints[0].get_y();
		dis = currentPosition.getDistance(wx, wy);
		if(dis < 0.75){
			cout << "found waypoint with dist < 0.75" << endl;
			waypoints.erase(waypoints.begin());
		}
		else{
			break;
		} 	
	}
	cout << "check plan active: " << waypoints.size() << endl;
	if(waypoints.size() > 0){
		isPlanActive = true;
	}
	else{
		isPlanActive = false;
	}
	cout << "end setup next waypoint" << endl;
   }

   void createNewWaypoint(CartesianPoint new_point){
    waypoints.push_back(new_point);
    isPlanActive = true;
    wx = new_point.get_x();
    wy = new_point.get_y();
   }

   void setupNearestWaypoint(Position currentPosition){
   	cout << "inside setup nearest waypoint" << endl;
	double dis;
	int farthest = 0;
	//cout << "waypoints size: " << waypoints.size() << endl;
	for (int i = 0; i < waypoints.size(); i++){
		dis = currentPosition.getDistance(waypoints[i].get_x(), waypoints[i].get_y());
		if(dis < 0.75){
			//cout << "found waypoint with dist < 0.75: " << i << endl;
			farthest = i;
		}
	}
	if(farthest == 0){
		waypoints.erase(waypoints.begin());
	}
	else{
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


   bool isWaypointComplete(Position currentPosition){
	bool status = false;
	double dis = currentPosition.getDistance(wx, wy);
	if (isPlanActive && (dis < 0.75)){
		status = true;
	}
	return status;
   }

   bool isAnyWaypointComplete(Position currentPosition){
	bool status = false;
	for (int i = 0; i < waypoints.size(); i++){
		double dis = currentPosition.getDistance(waypoints[i].get_x(), waypoints[i].get_y());
		if (isPlanActive && (dis < 0.75)){
			status = true;
			break;
		}
	}
	return status;
   }

  double getPathCostInNavGraph(){return pathCostInNavGraph;}
  double getPathCostInNavOrigGraph(){return pathCostInNavOrigGraph;}
  double getOrigPathCostInOrigNavGraph(){return origPathCostInOrigNavGraph;}
  double getOrigPathCostInNavGraph(){return origPathCostInNavGraph;}

  void updatePlanPositions(double x, double y){
  	cout << "In updatePlanPositions x " << x << " y " << y << endl;
  	int floor_x = (int)(floor(x))-1;
  	int floor_y = (int)(floor(y))-1;
  	int ceil_x = (int)(ceil(x))+1;
  	int ceil_y = (int)(ceil(y))+1;
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

  bool getPlanPositionValue(double x, double y){
  	cout << "In getPlanPositionValue" << endl;
  	int floor_x = (int)(floor(x));
  	int floor_y = (int)(floor(y));
  	int ceil_x = (int)(ceil(x));
  	int ceil_y = (int)(ceil(y));
  	if(floor_x >= 0 and floor_y >= 0 and floor_x < planPositions[0].size() and floor_y < planPositions[0].size()){
  		cout << "planPositions[" << floor_x << "][" << floor_y << "] = " << planPositions[floor_x][floor_y] << endl;
  		if(planPositions[floor_x][floor_y] == 1)
  			return true;
  	}
  	if(floor_x >= 0 and ceil_y >= 0 and floor_x < planPositions[0].size() and ceil_y < planPositions[0].size()){
  		cout << "planPositions[" << floor_x << "][" << ceil_y << "] = " << planPositions[floor_x][ceil_y] << endl;
  		if(planPositions[floor_x][ceil_y] == 1)
  			return true;
  	}
  	if(ceil_x >= 0 and floor_y >= 0 and ceil_x < planPositions[0].size() and floor_y < planPositions[0].size()){
  		cout << "planPositions[" << ceil_x << "][" << floor_y << "] = " << planPositions[ceil_x][floor_y] << endl;
  		if(planPositions[ceil_x][floor_y] == 1)
  			return true;
  	}
  	if(ceil_x >= 0 and ceil_y >= 0 and ceil_x < planPositions[0].size() and ceil_y < planPositions[0].size()){
  		cout << "planPositions[" << ceil_x << "][" << ceil_y << "] = " << planPositions[ceil_x][ceil_y] << endl;
  		if(planPositions[ceil_x][ceil_y] == 1)
  			return true;
  	}
  	return false;
  }
   
 private:
  
  // Current plan generated by A*, stored as waypoints , index 0 being the beginning of the plan
  vector<CartesianPoint> waypoints;
  vector<CartesianPoint> origWaypoints;
  double pathCostInNavGraph, pathCostInNavOrigGraph;
  double origPathCostInOrigNavGraph, origPathCostInNavGraph;

  list<int> waypointInd;
  vector< list<int> > plansInds;

  CartesianPoint currentWaypoint;

  bool isPlanActive;
  
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

  // Plan positions
  vector< vector<int> > planPositions;

};

#endif
