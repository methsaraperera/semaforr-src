/*!
 * AgentState.h
 *
 * \brief Represents the current state of an agent: its current position,
 *        its plans, etc.).
 *
 * \author Eric Schneider <esch@hunter.cuny.edu>
 * \date 11/11/2011 Created
 */
#ifndef AGENTSTATE_H
#define AGENTSTATE_H

#include "Task.h"
#include "FORRAction.h"
#include "Position.h"
#include "FORRGeometry.h"

#include <time.h>
#include <unistd.h>

#include <vector>
#include <list>
#include <deque>
#include <set>

#include <ros/ros.h>
#include <ros/console.h>
#include <sensor_msgs/LaserScan.h>
#include <geometry_msgs/PoseArray.h>
#include <tf/transform_datatypes.h>

using namespace std;

class AgentState
{
public:
  /** \brief AgentState constructor */
  AgentState(double arrMove[], double arrRotate[], int moveArrMax, int rotateArrMax) : currentTask(NULL) {
    vetoedActions = new set<FORRAction>();
    action_set = new set<FORRAction>();
    forward_set = new set<FORRAction>();
    rotation_set = new set<FORRAction>();
    rotateMode = true;
    //rotateMode = false;
    numMoves = moveArrMax;
    numRotates = rotateArrMax;
    robotConfined = false;
    getOutTriggered = false;

    for(int i = 1; i < numRotates; i++){
      action_set->insert(FORRAction(LEFT_TURN, i));
      action_set->insert(FORRAction(RIGHT_TURN, i));
      rotation_set->insert(FORRAction(LEFT_TURN, i));
      rotation_set->insert(FORRAction(RIGHT_TURN, i));
    }
    for(int i = 1; i < numMoves; i++){
      action_set->insert(FORRAction(FORWARD, i));
      forward_set->insert(FORRAction(FORWARD, i));
    }
    action_set->insert(FORRAction(PAUSE,0));
    forward_set->insert(FORRAction(PAUSE,0));
    //rotation_set->insert(FORRAction(PAUSE,0));
    //double m[] = {0, 0.2, 0.4, 0.8, 1.6, 3.2};  
    //double r[] = {0, 0.25, 0.5, 1, 2};
    for(int i = 0 ; i < numMoves ; i++) move[i] = arrMove[i];
    for(int i = 0 ; i < numRotates ; i++) rotate[i] = arrRotate[i];
    all_position_trace = new vector<Position>();
    all_laser_history = new vector< vector<CartesianPoint> >();
  }
  
  // Best possible move towards the target
  FORRAction moveTowards(CartesianPoint target);

  set<FORRAction> *getActionSet(){return action_set;}
  set<FORRAction> *getForwardActionSet(){return forward_set;}
  set<FORRAction> *getRotationActionSet(){return rotation_set;}

  double getDistanceToTarget(double x, double y){ 
    double dx = x - currentTask->getX();
    double dy = y - currentTask->getY();
    return sqrt((dx*dx) + (dy*dy));
  }

  
  // returns the euclidiean distance to target from the current robot position 
  double getDistanceToTarget(){
     return currentPosition.getDistance(currentTask->getX(), currentTask->getY());
  }
  
  bool isDestinationReached(int epsilon){
    if(getDistanceToTarget() < epsilon) {
        return true;
    }
    return false;
  }

  Position getCurrentPosition() { return currentPosition; }
  vector<CartesianPoint> getCurrentLaserEndpoints() { return laserEndpoints; }

  void setCurrentSensor(Position p, sensor_msgs::LaserScan scan) { 
	currentPosition = p;
     	currentLaserScan = scan;
     	transformToEndpoints();
	if(currentTask != NULL){
		//save the current position and laser endpoints 
    all_laserscan_history.push_back(scan);
		currentTask->saveSensor(p,laserEndpoints, scan);
	}
  }
  
  Task *getCurrentTask() { return currentTask; }
  /*void setCurrentTask(Task *task, Position current, PathPlanner *planner, bool aStarOn) { 
    currentTask = task; 
    if(aStarOn){
    	currentTask->generateWaypoints(current, planner);
    } 
  }*/

  void setCurrentTask(Task *task){
    currentTask = task;
    ROS_DEBUG_STREAM("Current task set");
  }

  list<int> getWaypoints(Position current, PathPlanner *planner, bool aStarOn){
    if(aStarOn){
      ROS_DEBUG_STREAM("Generating waypoints");
      currentTask->generateWaypoints(current, planner);
      return currentTask->getWaypointInds();
    }
  }

  vector< list<int> > getPlansWaypoints(Position current, PathPlanner *planner, bool aStarOn){
    if(aStarOn){
      ROS_DEBUG_STREAM("Generating multiple sets of waypoints");
      currentTask->generateWaypoints(current, planner);
      return currentTask->getPlansInds();
    }
  }

  void setCurrentWaypoints(Position current, PathPlanner *planner, bool aStarOn, list<int> indices){
    if(aStarOn){
      currentTask->generateWaypointsFromInds(current, planner, indices);
      //currentTask->generateOriginalWaypoints(current, planner);
    }
  }

  set<FORRAction> *getVetoedActions() { 
	//std::cout << "returning vetoed action list " << vetoedActions->size() << std::endl;
	return vetoedActions;
  }
  void clearVetoedActions() { vetoedActions->clear();}
  
  void addTask(float x, float y) {
    Task *task = new Task(x,y);
    agenda.push_back(task); 
    all_agenda.push_back(task);
  }
  
  Task *getNextTask() { return agenda.front(); }
  
  list<Task*>& getAgenda() { return agenda; }
  list<Task*>& getAllAgenda() { return all_agenda; }
  
  //Merge finish task and skip task they are both doing the same right now
  void finishTask() {
    if (currentTask != NULL){
      //save the current task position into all_trace
      vector<CartesianPoint> trace;
      vector<Position> *pos_hist = currentTask->getPositionHistory();
      for(int i = 0 ; i < pos_hist->size() ; i++){
        trace.push_back(CartesianPoint((*pos_hist)[i].getX(),(*pos_hist)[i].getY()));
        all_position_trace->push_back((*pos_hist)[i]);
      }
      all_trace.push_back(trace);
      vector< vector<CartesianPoint> > *laser_hist = currentTask->getLaserHistory();
      for(int i = 0 ; i < laser_hist->size() ; i++){
        all_laser_history->push_back((*laser_hist)[i]);
      }
      task_decision_count.push_back(currentTask->getDecisionCount());
      agenda.remove(currentTask);
    }
    rotateMode = true;
    currentTask = NULL;
  }

  vector< vector<CartesianPoint> > getAllTrace(){return all_trace;}
  vector< Position > *getAllPositionTrace(){return all_position_trace;}
  vector< vector<CartesianPoint> > *getAllLaserHistory(){return all_laser_history;}
  vector< sensor_msgs::LaserScan > getAllLaserScanHistory(){return all_laserscan_history;}

  vector<int> getTaskDecisionCount(){return task_decision_count;}

  void skipTask() {
    if (currentTask != NULL)
      agenda.remove(currentTask);

    currentTask = NULL;
  }

  bool isMissionComplete(){
	bool status = false; 
	if(getAgenda().size() == 0 && currentTask == NULL){
		status = true;
 	}
	return status;
  }

  sensor_msgs::LaserScan getCurrentLaserScan(){return currentLaserScan;}

  vector<CartesianPoint> transformToEndpoints(Position p, sensor_msgs::LaserScan scan);
  
  Position getExpectedPositionAfterAction(FORRAction action);

  Position getExpectedPositionAfterAction(FORRAction action, vector<CartesianPoint> initialLaser, Position currPosition);

  // Returns distance from obstacle 
  double getDistanceToNearestObstacle(Position pos);

  // Returns nearest obstacle 
  Position getNearestObstacle(Position pos);

  // returns distance to obstacle in the direction of rotation
  double getDistanceToObstacle(double rotation_angle);
  double getDistanceToObstacle(Position initialPosition, vector<CartesianPoint> initialLaser, double rotation_angle);
  double getDistanceToForwardObstacle(){
	//ROS_DEBUG("in getDistance to forward obstacle");
	if(currentLaserScan.ranges.size() == 0)
		return 25;
	return currentLaserScan.ranges[currentLaserScan.ranges.size()/2];
  }
  double getDistanceToForwardObstacle(Position initialPosition, vector<CartesianPoint> initialLaser){
    //ROS_DEBUG("in getDistance to forward obstacle");
    if(initialLaser.size() == 0)
      return 25;
    return initialPosition.getDistance(initialLaser[initialLaser.size()/2].get_x(), initialLaser[initialLaser.size()/2].get_y());
  }

  FORRAction maxForwardAction();

  bool getRobotConfined();
  bool getGetOutTriggered(){return getOutTriggered;}
  void setGetOutTriggered(bool status){
    getOutTriggered = status;
  }

  // Can a robot see a segment or a point using its laser scan data?
  bool canSeeSegment(CartesianPoint point1, CartesianPoint point2);
  bool canSeeSegment(vector<CartesianPoint> givenLaserEndpoints, CartesianPoint laserPos, CartesianPoint point1, CartesianPoint point2);
  bool canSeePoint(vector<CartesianPoint> givenLaserEndpoints, CartesianPoint laserPos, CartesianPoint point);
  bool canSeePoint(CartesianPoint point, double distanceLimit);
  bool canAccessPoint(vector<CartesianPoint> givenLaserEndpoints, CartesianPoint laserPos, CartesianPoint point, double distanceLimit);
  std::pair < std::vector<CartesianPoint>, std::vector< vector<CartesianPoint> > > getCleanedTrailMarkers();

  double getMovement(int para){return move[para];}
  double getRotation(int para){return rotate[para];}

  bool getRotateMode(){return rotateMode;}
  void setRotateMode(bool mode){rotateMode = mode;}
 
  void setAgentStateParameters(double val1, double val2, double val3, double val4, double val5, double val6, double val7);
  
  geometry_msgs::PoseArray getCrowdPose(){ return currentCrowd;}
  void setCrowdPose(geometry_msgs::PoseArray crowdpose){
	currentCrowd = crowdpose;
  }

  vector <Position> getCrowdPositions(geometry_msgs::PoseArray crowdpose);

  geometry_msgs::PoseArray getCrowdPoseAll(){ return allCrowd;}
  void setCrowdPoseAll(geometry_msgs::PoseArray crowdposeall){
	allCrowd = crowdposeall;
  }

  void setCrowdModel(semaforr::CrowdModel c){ 
    crowdModel = c;
  }
  semaforr::CrowdModel getCrowdModel(){ return crowdModel;}

  bool crowdModelLearned();
  bool riskModelLearned();
  bool flowModelLearned();
  double getGridValue(double x, double y);
  double getRiskValue(double x, double y);
  double getFlowValue(double x, double y, double theta);
  double getCrowdObservation(double x, double y);
  double getRiskExperience(double x, double y);
  double getFLowObservation(double x, double y);

 private:

  // Stores the move and rotate action values
  double move[300];  
  double rotate[300];
  int numMoves, numRotates;

  FORRAction get_max_allowed_forward_move();

  // Current position of the agent x, y, theta 
  Position currentPosition;

  // All position history of all targets
  vector< vector<CartesianPoint> > all_trace;
  vector< Position > *all_position_trace;

  // All laser history of all targets
  vector< vector<CartesianPoint> > *all_laser_history;
  vector< sensor_msgs::LaserScan > all_laserscan_history;

  // Decision count by task
  vector<int> task_decision_count;

  // set of vetoed actions that the robot cant execute in its current state
  set<FORRAction> *vetoedActions;

  // Set of all actions that the robot has in its action set
  set<FORRAction> *action_set;

  // Set of all forward actions that the robot has in its action set
  set<FORRAction> *forward_set;

  // Set of all forward actions that the robot has in its action set
  set<FORRAction> *rotation_set;
  
  // aggregate decision making statistics of the agent
  // Total travel time
  double total_travel_time;

  // Total travel distance
  double total_travel_distance;

  // Total tier 1 decisions 
  int total_t1_decisions;

  // Total tier 2 decisions
  int total_t2_decisions;

  // Total tier 3 decisions
  int total_t3_decisions;
  
  /** \brief The agent's currnet list of tasks */
  list<Task*> agenda;

  // list of all targets;
  list<Task*> all_agenda;
  
  // Current task in progress
  Task *currentTask;

  // Currrent laser scan reading at the current position
  sensor_msgs::LaserScan currentLaserScan;

  // Current laser scan data as endpoints in the x-y coordinate frame
  vector<CartesianPoint> laserEndpoints;

  //Converts current laser range scanner to endpoints
  void transformToEndpoints();

  // Nearby crowd positions
  geometry_msgs::PoseArray currentCrowd;

  // All crowd positions
  geometry_msgs::PoseArray allCrowd;

  // Current crowd model
  semaforr::CrowdModel crowdModel;

  //Rotate mode tells if the t3 should rotate or move
  bool rotateMode;

  // Robot confined currently
  bool robotConfined;
  bool getOutTriggered;

  //after linear move
  Position afterLinearMove(Position initialPosition, double distance);
  Position afterAngularMove(Position initialPosition, double angle);
  Position afterLinearMove(Position initialPosition, vector<CartesianPoint> initialLaser, double distance);
  Position afterAngularMove(Position initialPosition, vector<CartesianPoint> initialLaser, double angle);

  // Parameters
  double canSeePointEpsilon, laserScanRadianIncrement, robotFootPrint, robotFootPrintBuffer, maxLaserRange, maxForwardActionBuffer, maxForwardActionSweepAngle;

};

#endif
