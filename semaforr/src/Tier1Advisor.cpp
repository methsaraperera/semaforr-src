
#include "Tier1Advisor.h"


void Tier1Advisor::advisorNotOpposite(){
  ROS_DEBUG("COntroller::advisorNotOpposite > Entering function");
  vector<FORRAction> actions = beliefs->getAgentState()->getCurrentTask()->getPreviousDecisions();
  set<FORRAction> *rotation_set = beliefs->getAgentState()->getRotationActionSet();
  int size = actions.size();
  if(actions.size() < 2){
    ROS_DEBUG("actions list less than 2. Exiting not opposite");
    return;
  }
  FORRAction lastAction = actions[size - 1];
  FORRAction lastlastAction = actions[size - 2];
  FORRAction lastlastlastAction = actions[size - 3];
  ROS_DEBUG_STREAM("Controller::advisorNotOpposite > " << lastAction.type << " " << lastAction.parameter << ", " << lastlastAction.type << " " << lastlastAction.parameter << ", " << lastlastlastAction.type << " " << lastlastlastAction.parameter); 
  // if(((lastlastAction.type == RIGHT_TURN or lastlastAction.type == LEFT_TURN) and lastAction.type == PAUSE) or lastAction.type == RIGHT_TURN or lastAction.type == LEFT_TURN){
  //   ROS_DEBUG("Not opposite active ");
  //   if((lastlastAction.type == RIGHT_TURN and lastAction.type == PAUSE) or lastAction.type == RIGHT_TURN or (lastlastAction.type == LEFT_TURN and lastAction.type == LEFT_TURN)){
  //     for(int i = 1; i < rotation_set->size()/2+1 ; i++){
  //       (beliefs->getAgentState()->getVetoedActions()->insert(FORRAction(LEFT_TURN, i)));
  //       ROS_DEBUG_STREAM("Vetoed action : " << FORRAction(LEFT_TURN, i).type << " " << i);
  //     }
  //   }
  //   if((lastlastAction.type == LEFT_TURN and lastAction.type == PAUSE) or lastAction.type == LEFT_TURN or (lastlastAction.type == RIGHT_TURN and lastAction.type == RIGHT_TURN)){
  //     for(int i = 1; i < rotation_set->size()/2+1 ; i++){
  //       (beliefs->getAgentState()->getVetoedActions()->insert(FORRAction(RIGHT_TURN, i)));
  //       ROS_DEBUG_STREAM("Vetoed action : " << FORRAction(RIGHT_TURN, i).type << " " << i);
  //     }
  //   }
  // }
  if(lastlastAction.type == RIGHT_TURN and lastAction.type == PAUSE){
    ROS_DEBUG("Not opposite active ");
    (beliefs->getAgentState()->getVetoedActions()->insert(FORRAction(LEFT_TURN, lastlastAction.parameter)));
    ROS_DEBUG_STREAM("Vetoed action : " << FORRAction(LEFT_TURN, lastlastAction.parameter).type << " " << lastlastAction.parameter);
    if(lastlastAction.parameter > 1){
      for(int i = 1; i < lastlastAction.parameter; i++){
        (beliefs->getAgentState()->getVetoedActions()->insert(FORRAction(LEFT_TURN, i)));
        ROS_DEBUG_STREAM("Vetoed action : " << FORRAction(LEFT_TURN, i).type << " " << i);
      }
    }
  }
  else if(lastlastAction.type == RIGHT_TURN and lastAction.type == RIGHT_TURN){
    ROS_DEBUG("Not opposite active ");
    double angle_rotated = beliefs->getAgentState()->getRotation(lastlastAction.parameter) + beliefs->getAgentState()->getRotation(lastAction.parameter);
    for(int i = 1; i < rotation_set->size()/2+1 ; i++){
      if(angle_rotated + beliefs->getAgentState()->getRotation(i) - M_PI > 0){
        (beliefs->getAgentState()->getVetoedActions()->insert(FORRAction(RIGHT_TURN, i)));
        ROS_DEBUG_STREAM("Vetoed action : " << FORRAction(RIGHT_TURN, i).type << " " << i);
      }
    }
    (beliefs->getAgentState()->getVetoedActions()->insert(FORRAction(LEFT_TURN, lastlastAction.parameter)));
    ROS_DEBUG_STREAM("Vetoed action : " << FORRAction(LEFT_TURN, lastlastAction.parameter).type << " " << lastlastAction.parameter);
    if(beliefs->getAgentState()->getRotation(lastlastAction.parameter) + beliefs->getAgentState()->getRotation(lastAction.parameter) == beliefs->getAgentState()->getRotation(lastlastAction.parameter + lastAction.parameter)){
      (beliefs->getAgentState()->getVetoedActions()->insert(FORRAction(LEFT_TURN, lastlastAction.parameter+lastAction.parameter)));
      ROS_DEBUG_STREAM("Vetoed action : " << FORRAction(LEFT_TURN, lastlastAction.parameter).type << " " << lastlastAction.parameter+lastAction.parameter);
    }
  }
  if(lastAction.type == RIGHT_TURN){
    ROS_DEBUG("Not opposite active ");
    (beliefs->getAgentState()->getVetoedActions()->insert(FORRAction(LEFT_TURN, lastAction.parameter)));
    ROS_DEBUG_STREAM("Vetoed action : " << FORRAction(LEFT_TURN, lastAction.parameter).type << " " << lastAction.parameter);
    if(lastAction.parameter > 1){
      for(int i = 1; i < lastAction.parameter; i++){
        (beliefs->getAgentState()->getVetoedActions()->insert(FORRAction(LEFT_TURN, i)));
        ROS_DEBUG_STREAM("Vetoed action : " << FORRAction(LEFT_TURN, i).type << " " << i);
      }
    }
  }


  if(lastlastAction.type == LEFT_TURN and lastAction.type == PAUSE){
    ROS_DEBUG("Not opposite active ");
    (beliefs->getAgentState()->getVetoedActions()->insert(FORRAction(RIGHT_TURN, lastlastAction.parameter)));
    ROS_DEBUG_STREAM("Vetoed action : " << FORRAction(RIGHT_TURN, lastlastAction.parameter).type << " " << lastlastAction.parameter);
    if(lastlastAction.parameter > 1){
      for(int i = 1; i < lastlastAction.parameter; i++){
        (beliefs->getAgentState()->getVetoedActions()->insert(FORRAction(RIGHT_TURN, i)));
        ROS_DEBUG_STREAM("Vetoed action : " << FORRAction(RIGHT_TURN, i).type << " " << i);
      }
    }
  }
  else if(lastlastAction.type == LEFT_TURN and lastAction.type == LEFT_TURN){
    ROS_DEBUG("Not opposite active ");
    double angle_rotated = beliefs->getAgentState()->getRotation(lastlastAction.parameter) + beliefs->getAgentState()->getRotation(lastAction.parameter);
    for(int i = 1; i < rotation_set->size()/2+1 ; i++){
      if(angle_rotated + beliefs->getAgentState()->getRotation(i) - M_PI > 0){
        (beliefs->getAgentState()->getVetoedActions()->insert(FORRAction(LEFT_TURN, i)));
        ROS_DEBUG_STREAM("Vetoed action : " << FORRAction(LEFT_TURN, i).type << " " << i);
      }
    }
    (beliefs->getAgentState()->getVetoedActions()->insert(FORRAction(RIGHT_TURN, lastlastAction.parameter)));
    ROS_DEBUG_STREAM("Vetoed action : " << FORRAction(RIGHT_TURN, lastlastAction.parameter).type << " " << lastlastAction.parameter);
    if(beliefs->getAgentState()->getRotation(lastlastAction.parameter) + beliefs->getAgentState()->getRotation(lastAction.parameter) == beliefs->getAgentState()->getRotation(lastlastAction.parameter + lastAction.parameter)){
      (beliefs->getAgentState()->getVetoedActions()->insert(FORRAction(RIGHT_TURN, lastlastAction.parameter+lastAction.parameter)));
      ROS_DEBUG_STREAM("Vetoed action : " << FORRAction(RIGHT_TURN, lastlastAction.parameter).type << " " << lastlastAction.parameter+lastAction.parameter);
    }
  }
  if(lastAction.type == LEFT_TURN){
    ROS_DEBUG("Not opposite active ");
    (beliefs->getAgentState()->getVetoedActions()->insert(FORRAction(RIGHT_TURN, lastAction.parameter)));
    ROS_DEBUG_STREAM("Vetoed action : " << FORRAction(RIGHT_TURN, lastAction.parameter).type << " " << lastAction.parameter);
    if(lastAction.parameter > 1){
      for(int i = 1; i < lastAction.parameter; i++){
        (beliefs->getAgentState()->getVetoedActions()->insert(FORRAction(RIGHT_TURN, i)));
        ROS_DEBUG_STREAM("Vetoed action : " << FORRAction(RIGHT_TURN, i).type << " " << i);
      }
    }
  }
  // if(lastlastAction.type == RIGHT_TURN or lastlastAction.type == LEFT_TURN){
  //   if(lastAction.type == PAUSE){
  //     ROS_DEBUG("Not opposite active ");
  //     if(lastlastAction.type == RIGHT_TURN)    for(int i = 1; i < rotation_set->size()/2+1 ; i++)   (beliefs->getAgentState()->getVetoedActions()->insert(FORRAction(LEFT_TURN, i)));
  //     else                                     for(int i = 1; i < rotation_set->size()/2+1 ; i++)   (beliefs->getAgentState()->getVetoedActions()->insert(FORRAction(RIGHT_TURN, i)));
  //   }
  // }
  ROS_DEBUG("leaving notOpposite");
  return;
}

/*
// This advisor should ban all forward moves not in the direction of the exit unless all of the exits
// are already blocked by other robots
void Tier1Advisor::advisorCircle(Beliefs *beliefs){

  // Initialize variables
  vector<FORRCircle> circles = (beliefs->abstractMap).getCircles();
  Position curr_pos = beliefs->getCurrentPosition();
  Task *task = beliefs->getCurrentTask();
  CartesianPoint targetPoint (task->getX() , task->getY());
  CartesianPoint currentPosition (curr_pos.getX(), curr_pos.getY());
  bool targetInCircle = false;
  bool currPosInCircleWithExit = false;
  int robotCircle=-1, targetCircle = -1;
  //cout << "Controller::advisorCircle >> Initializing variables " << endl;
  // check the preconditions for activating the advisor
  for(int i = 0; i < circles.size() ; i++){
    // check if the target point is in circle
    if(circles[i].inCircle(targetPoint.get_x(), targetPoint.get_y())){
      targetInCircle = true;
      targetCircle = i;
    }
    // check if the rob_pos is in a circle and the circle has atleast one exit
    if(circles[i].inCircle(curr_pos.getX(), curr_pos.getY()) and ((circles[i]).getExits().size() >= 1)){
      currPosInCircleWithExit = true;
      robotCircle = i;
    }
  }

  // if preconditions are met, veto forward moves in the direction of non exits
  if(targetInCircle == true and currPosInCircleWithExit == true and robotCircle != targetCircle){
    //cout << "Controller::advisorCircle >> Activating tier 1 get out of circle advisor" << endl;
    vector<FORRExit> exits = circles[robotCircle].getExits(); 
    bool facingExit = false;
    double forward_distance = beliefs->wallDistanceVector[0];
    for(int i = 0; i< exits.size(); i++){
      CartesianPoint exitPoint = exits[i].getExitPoint();
      double exitDistance = Utils::get_euclidian_distance(exitPoint.get_x(), exitPoint.get_y(), curr_pos.getX() , curr_pos.getY());
      if(!(beliefs->abstractMap).isExitToLeaf(exits[i]) || exits[i].getExitCircle() == targetCircle){
	if( ( isFacing(curr_pos, exitPoint, circles[robotCircle].getRadius()) and forward_distance - exitDistance > 20 ) or exitDistance < 40 ){
	  //cout << "Controller::advisorCircle >> Robot is facing exit " << exitPoint.get_x() << exitPoint.get_y() << endl;
	  facingExit = true;
	  break;
	}
      }
    }
    // Robot is not facing any of the exits ban all forward moves
    if(facingExit == false){
      //cout << "Controller::advisorCircle >> Vetoing all forward moves" << endl;
      for(int i = 1; i < 6; ++i){
	(beliefs->vetoedActions)->insert(FORRAction(FORWARD, i));
      }
    }
  }
  return;
}

bool Tier1Advisor::isFacing(Position robotPos , CartesianPoint point, double radius){
  bool isFacing = false;
  double robot_point_angle = atan2(point.get_y() - robotPos.getY(), point.get_x() - robotPos.getX());
  double angleDiff = robotPos.getTheta() - robot_point_angle;
  //cout << "In function isFacing " << endl;
  //cout << "Robot angle " << robotPos.getTheta() << endl;
  //cout << "Angle made by the robot and the position with x axis" << robot_point_angle << endl;
  double ratio = Utils::get_euclidian_distance(robotPos.getX(), robotPos.getY() ,point.get_x(), point.get_y())/(2*radius);
  double min_delta = 0.72; //30 degrees on each side
  double max_delta = 3.14/(2); //60 degrees on each side
  double delta = ratio*(min_delta) + (1-ratio)*(max_delta) ;

  if(abs(angleDiff) < delta){
    isFacing = true;
  }
  return isFacing;
}


*/

bool Tier1Advisor::advisorVictory(FORRAction *decision) {
  ROS_DEBUG("Begin Victory advisor");
  // if the robot is oriented towards the goal and the robot actions which are not vetoed allows the robot to reach the goal then take that action.
  bool decisionMade = false;
  CartesianPoint task(beliefs->getAgentState()->getCurrentTask()->getTaskX(),beliefs->getAgentState()->getCurrentTask()->getTaskY());
  ROS_DEBUG("Check if target can be spotted using laser scan");
  cout << "Target = " << task.get_x() << " " << task.get_y() << endl;
  bool targetInSight = beliefs->getAgentState()->canSeePoint(task, 20);
  if(targetInSight == false){
    ROS_DEBUG("Target not in sight, Victory advisor skipped");
  }
  else{
    ROS_DEBUG("Target in sight , Victory advisor active");
    (*decision) = beliefs->getAgentState()->moveTowards(task);
    if(decision->parameter != 0){
      Position expectedPosition = beliefs->getAgentState()->getExpectedPositionAfterAction((*decision));
      if(expectedPosition.getDistance(beliefs->getAgentState()->getCurrentPosition()) >= 0.1){
        if(decision->type == RIGHT_TURN or decision->type == LEFT_TURN){
          ROS_DEBUG("Target in sight and no obstacles, Victory advisor to take decision");
          decisionMade = true;
        }
        else{
          FORRAction forward = beliefs->getAgentState()->maxForwardAction();
          if(forward.parameter >= decision->parameter){
            ROS_DEBUG("Target in sight and no obstacles, Victory advisor to take decision");
            decisionMade = true;
            // Position currentPosition = beliefs->getAgentState()->getCurrentPosition();
            // beliefs->getAgentState()->getCurrentTask()->updatePlanPositions(currentPosition.getX(), currentPosition.getY());
            // if(decision->type == FORWARD)
            //   beliefs->getAgentState()->setGetOutTriggered(false);
          }
        }
      }
    }
    /*vector<FORRAction> actions = beliefs->getAgentState()->getCurrentTask()->getPreviousDecisions();
    if(actions.size() > 1){
      FORRAction lastAction = actions[actions.size() - 1];
      if((lastAction.type == RIGHT_TURN or lastAction.type == LEFT_TURN) and (decision->type == RIGHT_TURN or decision->type == LEFT_TURN)){
        decisionMade = false;
      }
      else if((lastAction.type == FORWARD or lastAction.type == PAUSE) and (decision->type == FORWARD or decision->type == PAUSE)){
        decisionMade = false;
      }
      else if(((decision->type == RIGHT_TURN or decision->type == LEFT_TURN) or (forward.parameter >= decision->parameter)) and decision->parameter != 0){
        ROS_DEBUG("Target in sight and no obstacles , victory advisor to take decision");
        decisionMade = true;
      }
    }*/
  }
  return decisionMade;
}

bool Tier1Advisor::advisorEnforcer(FORRAction *decision) {
  ROS_DEBUG("Begin Enforcer advisor");
  // if the robot is oriented towards the goal and the robot actions which are not vetoed allows the robot to reach the goal then take that action.
  bool decisionMade = false;
  ROS_DEBUG("Check if waypoint can be spotted using laser scan");
  if(beliefs->getAgentState()->getCurrentTask()->getPlannerName() != "skeleton"){
    CartesianPoint waypoint(beliefs->getAgentState()->getCurrentTask()->getX(),beliefs->getAgentState()->getCurrentTask()->getY());
    cout << "Waypoint = " << waypoint.get_x() << " " << waypoint.get_y() << endl;
    bool waypointInSight = beliefs->getAgentState()->canSeePoint(waypoint, 20);
    if(waypointInSight == true){
      ROS_DEBUG("Waypoint in sight , Enforcer advisor active");
      (*decision) = beliefs->getAgentState()->moveTowards(waypoint);
      if(decision->parameter != 0){
        set<FORRAction> *vetoed_actions = beliefs->getAgentState()->getVetoedActions();
        if(vetoed_actions->find(*decision) != vetoed_actions->end()){
          decisionMade = false;
        }
        else{
          Position expectedPosition = beliefs->getAgentState()->getExpectedPositionAfterAction((*decision));
          if(expectedPosition.getDistance(beliefs->getAgentState()->getCurrentPosition()) >= 0.1){
            if(decision->type == RIGHT_TURN or decision->type == LEFT_TURN){
              ROS_DEBUG("Waypoint in sight and no obstacles and not vetoed, Enforcer advisor to take decision");
              decisionMade = true;
            }
            else{
              FORRAction forward = beliefs->getAgentState()->maxForwardAction();
              if(forward.parameter >= decision->parameter){
                ROS_DEBUG("Waypoint in sight and no obstacles and not vetoed, Enforcer advisor to take decision");
                decisionMade = true;
                // Position currentPosition = beliefs->getAgentState()->getCurrentPosition();
                // beliefs->getAgentState()->getCurrentTask()->updatePlanPositions(currentPosition.getX(), currentPosition.getY());
                // if(decision->type == FORWARD)
                //   beliefs->getAgentState()->setGetOutTriggered(false);
              }
            }
          }
        }
      }
    }
    else{
      ROS_DEBUG("Waypoint not in sight, Enforcer advisor skipped");
    }
  }
  else if(beliefs->getAgentState()->getCurrentTask()->getPlanSize() > 0){
    cout << "Waypoint Region = " << beliefs->getAgentState()->getCurrentTask()->getX() << " " << beliefs->getAgentState()->getCurrentTask()->getY() << endl;
    bool waypointRegionInSight = false;
    int regionID = -1;
    bool nextWaypointRegionInSight = false;
    int nextRegionId = -1;
    bool waypointPathInSight = false;
    int pathID = -1;
    if(beliefs->getAgentState()->getCurrentTask()->getSkeletonWaypoint().waypointIsRegion()){
      waypointRegionInSight = beliefs->getAgentState()->canSeeRegion(beliefs->getAgentState()->getCurrentTask()->getSkeletonWaypoint().getRegion().getCenter(), beliefs->getAgentState()->getCurrentTask()->getSkeletonWaypoint().getRegion().getRadius(), 20);
      regionID = 0;
    }
    else{
      if(beliefs->getAgentState()->getCurrentTask()->getPlanSize() > 1){
        if(beliefs->getAgentState()->getCurrentTask()->getSkeletonWaypoints()[1].waypointIsRegion()){
          waypointRegionInSight = beliefs->getAgentState()->canSeeRegion(beliefs->getAgentState()->getCurrentTask()->getSkeletonWaypoints()[1].getRegion().getCenter(), beliefs->getAgentState()->getCurrentTask()->getSkeletonWaypoints()[1].getRegion().getRadius(), 20);
          regionID = 1;
        }
      }
    }
    if(beliefs->getAgentState()->getCurrentTask()->getSkeletonWaypoint().waypointIsRegion() and beliefs->getAgentState()->getCurrentTask()->getPlanSize() >= 3){
      nextWaypointRegionInSight = beliefs->getAgentState()->canSeeRegion(beliefs->getAgentState()->getCurrentTask()->getSkeletonWaypoints()[2].getRegion().getCenter(), beliefs->getAgentState()->getCurrentTask()->getSkeletonWaypoints()[2].getRegion().getRadius(), 20);
      nextRegionId = 2;
    }
    else if(beliefs->getAgentState()->getCurrentTask()->getPlanSize() >= 4 and !beliefs->getAgentState()->getCurrentTask()->getSkeletonWaypoint().waypointIsRegion()){
      nextWaypointRegionInSight = beliefs->getAgentState()->canSeeRegion(beliefs->getAgentState()->getCurrentTask()->getSkeletonWaypoints()[3].getRegion().getCenter(), beliefs->getAgentState()->getCurrentTask()->getSkeletonWaypoints()[3].getRegion().getRadius(), 20);
      nextRegionId = 3;
    }
    if(!beliefs->getAgentState()->getCurrentTask()->getSkeletonWaypoint().waypointIsRegion()){
      vector<CartesianPoint> pathBetween = beliefs->getAgentState()->getCurrentTask()->getSkeletonWaypoint().getPath();
      for(int i = 0; i < pathBetween.size(); i++){
        if(beliefs->getAgentState()->canSeePoint(pathBetween[i], 20)){
          waypointPathInSight = true;
          pathID = 0;
          break;
        }
      }
    }
    else{
      if(beliefs->getAgentState()->getCurrentTask()->getPlanSize() > 1){
        if(!beliefs->getAgentState()->getCurrentTask()->getSkeletonWaypoints()[1].waypointIsRegion()){
          vector<CartesianPoint> pathBetween = beliefs->getAgentState()->getCurrentTask()->getSkeletonWaypoints()[1].getPath();
          for(int i = 0; i < pathBetween.size(); i++){
            if(beliefs->getAgentState()->canSeePoint(pathBetween[i], 20)){
              waypointPathInSight = true;
              pathID = 1;
              break;
            }
          }
        }
      }
    }
    // cout << "waypointRegionInSight " << waypointRegionInSight << " regionID " << regionID << " nextWaypointRegionInSight " << nextWaypointRegionInSight << " nextRegionId " << nextRegionId << " waypointPathInSight " << waypointPathInSight << " pathID " << pathID << endl;
    if(nextWaypointRegionInSight == true){
      ROS_DEBUG("Next Waypoint Region in sight, Enforcer advisor active");
      set<FORRAction> *vetoedActions = beliefs->getAgentState()->getVetoedActions();
      set<FORRAction> *action_set = beliefs->getAgentState()->getActionSet();
      set<FORRAction> possible_set;
      set<FORRAction>::iterator actionIter;
      FORRAction closest_action;
      double dist_to_region = 100000;
      vector<FORRAction> actions = beliefs->getAgentState()->getCurrentTask()->getPreviousDecisions();
      FORRAction lastAction;
      int size = actions.size();
      // cout << size << " " << (size - 1) << " " << ((size - 1) >= 0) << endl;
      if(size > 0){
        // cout << "first" << endl;
        lastAction = actions[size - 1];
        // cout << "after" << endl;
      }
      else{
        // cout << "second" << endl;
        lastAction = FORRAction(PAUSE, 0);
        // cout << "after" << endl;
      }
      // cout << "lastAction " << lastAction.type << " " << lastAction.parameter << endl;
      for(actionIter = action_set->begin(); actionIter != action_set->end(); actionIter++){
        FORRAction forrAction = *actionIter;
        if(std::find(vetoedActions->begin(), vetoedActions->end(), forrAction) != vetoedActions->end()){
          ROS_DEBUG_STREAM("Vetoed action : " << forrAction.type << " " << forrAction.parameter);
          continue;
        }
        else if(forrAction.type == PAUSE or forrAction.parameter == 0){
          ROS_DEBUG_STREAM("Pause action : " << forrAction.type << " " << forrAction.parameter);
          continue;
        }
        else if((lastAction.type == RIGHT_TURN or lastAction.type == LEFT_TURN) and (forrAction.type == RIGHT_TURN or forrAction.type == LEFT_TURN)){
          ROS_DEBUG_STREAM("Skip action : " << forrAction.type << " " << forrAction.parameter);
          continue;
        }
        else{
          Position expectedPosition = beliefs->getAgentState()->getExpectedPositionAfterAction(forrAction);
          if(nextRegionId == 2){
            if(beliefs->getAgentState()->getCurrentTask()->getSkeletonWaypoints()[2].getRegion().inRegion(expectedPosition.getX(), expectedPosition.getY())){
              FORRAction a(forrAction.type,forrAction.parameter);
              ROS_DEBUG_STREAM("Potential action : " << a.type << " " << a.parameter);
              possible_set.insert(a);
            }
            else{
              ROS_DEBUG_STREAM("Get closer action : " << forrAction.type << " " << forrAction.parameter);
              double dist_from_exp = beliefs->getAgentState()->getCurrentTask()->getSkeletonWaypoints()[2].getRegion().getCenter().get_distance(CartesianPoint(expectedPosition.getX(), expectedPosition.getY()));
              // cout << dist_from_exp << endl;
              if(dist_from_exp < dist_to_region){
                dist_to_region = dist_from_exp;
                closest_action = forrAction;
              }
            }
          }
          else if(nextRegionId == 3){
            if(beliefs->getAgentState()->getCurrentTask()->getSkeletonWaypoints()[3].getRegion().inRegion(expectedPosition.getX(), expectedPosition.getY())){
              FORRAction a(forrAction.type,forrAction.parameter);
              ROS_DEBUG_STREAM("Potential action : " << a.type << " " << a.parameter);
              possible_set.insert(a);
            }
            else{
              ROS_DEBUG_STREAM("Get closer action : " << forrAction.type << " " << forrAction.parameter);
              double dist_from_exp = beliefs->getAgentState()->getCurrentTask()->getSkeletonWaypoints()[3].getRegion().getCenter().get_distance(CartesianPoint(expectedPosition.getX(), expectedPosition.getY()));
              // cout << dist_from_exp << endl;
              if(dist_from_exp < dist_to_region){
                dist_to_region = dist_from_exp;
                closest_action = forrAction;
              }
            }
          }
        }
      }
      if(possible_set.size() == 1){
        actionIter = possible_set.begin();
        (*decision) = (*actionIter);
        ROS_DEBUG("Only one action to get to Next Waypoint Region, Enforcer advisor to take decision");
        decisionMade = true;
      }
      else if(possible_set.size() > 1){
        ROS_DEBUG("More than one action to get to Next Waypoint Region, Enforcer advisor to take decision");
        if(nextRegionId == 2){
          double minDistance = 1000000;
          for(actionIter = possible_set.begin(); actionIter != possible_set.end(); actionIter++){
            FORRAction forrAction = *actionIter;
            ROS_DEBUG_STREAM("Potential action : " << forrAction.type << " " << forrAction.parameter);
            Position expectedPosition = beliefs->getAgentState()->getExpectedPositionAfterAction(forrAction);
            if(beliefs->getAgentState()->getCurrentTask()->getSkeletonWaypoints()[2].getRegion().getCenter().get_distance(CartesianPoint(expectedPosition.getX(), expectedPosition.getY())) < minDistance){
              minDistance = beliefs->getAgentState()->getCurrentTask()->getSkeletonWaypoints()[2].getRegion().getCenter().get_distance(CartesianPoint(expectedPosition.getX(), expectedPosition.getY()));
              // cout << "New minDistance " << minDistance << endl;
              (*decision) = forrAction;
              decisionMade = true;
            }
          }
        }
        else if(nextRegionId == 3){
          double minDistance = 1000000;
          for(actionIter = possible_set.begin(); actionIter != possible_set.end(); actionIter++){
            FORRAction forrAction = *actionIter;
            ROS_DEBUG_STREAM("Potential action : " << forrAction.type << " " << forrAction.parameter);
            Position expectedPosition = beliefs->getAgentState()->getExpectedPositionAfterAction(forrAction);
            if(beliefs->getAgentState()->getCurrentTask()->getSkeletonWaypoints()[3].getRegion().getCenter().get_distance(CartesianPoint(expectedPosition.getX(), expectedPosition.getY())) < minDistance){
              minDistance = beliefs->getAgentState()->getCurrentTask()->getSkeletonWaypoints()[3].getRegion().getCenter().get_distance(CartesianPoint(expectedPosition.getX(), expectedPosition.getY()));
              // cout << "New minDistance " << minDistance << endl;
              (*decision) = forrAction;
              decisionMade = true;
            }
          }
        }
      }
      // else if(dist_to_region < 100000){
      //   ROS_DEBUG("Action to get closest to visible Next Waypoint Region, Enforcer advisor to take decision");
      //   (*decision) = closest_action;
      //   decisionMade = true;
      // }
    }
    else{
      ROS_DEBUG("Next Waypoint Region not in sight");
    }
    if(decisionMade == false and waypointRegionInSight == true){
      ROS_DEBUG("Waypoint Region in sight, Enforcer advisor active");
      set<FORRAction> *vetoedActions = beliefs->getAgentState()->getVetoedActions();
      set<FORRAction> *action_set = beliefs->getAgentState()->getActionSet();
      set<FORRAction> possible_set;
      set<FORRAction>::iterator actionIter;
      FORRAction closest_action;
      double dist_to_region = 100000;
      vector<FORRAction> actions = beliefs->getAgentState()->getCurrentTask()->getPreviousDecisions();
      FORRAction lastAction;
      int size = actions.size();
      // cout << size << " " << (size - 1) << " " << ((size - 1) >= 0) << endl;
      if(size > 0){
        // cout << "first" << endl;
        lastAction = actions[size - 1];
        // cout << "after" << endl;
      }
      else{
        // cout << "second" << endl;
        lastAction = FORRAction(PAUSE, 0);
        // cout << "after" << endl;
      }
      // cout << "lastAction " << lastAction.type << " " << lastAction.parameter << endl;
      for(actionIter = action_set->begin(); actionIter != action_set->end(); actionIter++){
        FORRAction forrAction = *actionIter;
        if(std::find(vetoedActions->begin(), vetoedActions->end(), forrAction) != vetoedActions->end()){
          ROS_DEBUG_STREAM("Vetoed action : " << forrAction.type << " " << forrAction.parameter);
          continue;
        }
        else if(forrAction.type == PAUSE or forrAction.parameter == 0){
          ROS_DEBUG_STREAM("Pause action : " << forrAction.type << " " << forrAction.parameter);
          continue;
        }
        else if((lastAction.type == RIGHT_TURN or lastAction.type == LEFT_TURN) and (forrAction.type == RIGHT_TURN or forrAction.type == LEFT_TURN)){
          ROS_DEBUG_STREAM("Skip action : " << forrAction.type << " " << forrAction.parameter);
          continue;
        }
        else{
          Position expectedPosition = beliefs->getAgentState()->getExpectedPositionAfterAction(forrAction);
          if(regionID == 0){
            if(beliefs->getAgentState()->getCurrentTask()->getSkeletonWaypoint().getRegion().inRegion(expectedPosition.getX(), expectedPosition.getY())){
              FORRAction a(forrAction.type,forrAction.parameter);
              ROS_DEBUG_STREAM("Potential action : " << a.type << " " << a.parameter);
              possible_set.insert(a);
            }
            else{
              ROS_DEBUG_STREAM("Get closer action : " << forrAction.type << " " << forrAction.parameter);
              double dist_from_exp = beliefs->getAgentState()->getCurrentTask()->getSkeletonWaypoint().getRegion().getCenter().get_distance(CartesianPoint(expectedPosition.getX(), expectedPosition.getY()));
              // cout << dist_from_exp << endl;
              if(dist_from_exp < dist_to_region){
                dist_to_region = dist_from_exp;
                closest_action = forrAction;
              }
            }
          }
          else if(regionID == 1){
            if(beliefs->getAgentState()->getCurrentTask()->getSkeletonWaypoints()[1].getRegion().inRegion(expectedPosition.getX(), expectedPosition.getY())){
              FORRAction a(forrAction.type,forrAction.parameter);
              ROS_DEBUG_STREAM("Potential action : " << a.type << " " << a.parameter);
              possible_set.insert(a);
            }
            else{
              ROS_DEBUG_STREAM("Get closer action : " << forrAction.type << " " << forrAction.parameter);
              double dist_from_exp = beliefs->getAgentState()->getCurrentTask()->getSkeletonWaypoints()[1].getRegion().getCenter().get_distance(CartesianPoint(expectedPosition.getX(), expectedPosition.getY()));
              // cout << dist_from_exp << endl;
              if(dist_from_exp < dist_to_region){
                dist_to_region = dist_from_exp;
                closest_action = forrAction;
              }
            }
          }
        }
      }
      if(possible_set.size() == 1){
        actionIter = possible_set.begin();
        (*decision) = (*actionIter);
        ROS_DEBUG("Only one action to get to Waypoint Region, Enforcer advisor to take decision");
        decisionMade = true;
      }
      else if(possible_set.size() > 1){
        ROS_DEBUG("More than one action to get to Waypoint Region, Enforcer advisor to take decision");
        if(regionID == 0){
          double minDistance = 1000000;
          for(actionIter = possible_set.begin(); actionIter != possible_set.end(); actionIter++){
            FORRAction forrAction = *actionIter;
            ROS_DEBUG_STREAM("Potential action : " << forrAction.type << " " << forrAction.parameter);
            Position expectedPosition = beliefs->getAgentState()->getExpectedPositionAfterAction(forrAction);
            if(beliefs->getAgentState()->getCurrentTask()->getSkeletonWaypoint().getRegion().getCenter().get_distance(CartesianPoint(expectedPosition.getX(), expectedPosition.getY())) < minDistance){
              minDistance = beliefs->getAgentState()->getCurrentTask()->getSkeletonWaypoint().getRegion().getCenter().get_distance(CartesianPoint(expectedPosition.getX(), expectedPosition.getY()));
              // cout << "New minDistance " << minDistance << endl;
              (*decision) = forrAction;
              decisionMade = true;
            }
          }
        }
        else if(regionID == 1){
          double minDistance = 1000000;
          for(actionIter = possible_set.begin(); actionIter != possible_set.end(); actionIter++){
            FORRAction forrAction = *actionIter;
            ROS_DEBUG_STREAM("Potential action : " << forrAction.type << " " << forrAction.parameter);
            Position expectedPosition = beliefs->getAgentState()->getExpectedPositionAfterAction(forrAction);
            if(beliefs->getAgentState()->getCurrentTask()->getSkeletonWaypoints()[1].getRegion().getCenter().get_distance(CartesianPoint(expectedPosition.getX(), expectedPosition.getY())) < minDistance){
              minDistance = beliefs->getAgentState()->getCurrentTask()->getSkeletonWaypoints()[1].getRegion().getCenter().get_distance(CartesianPoint(expectedPosition.getX(), expectedPosition.getY()));
              // cout << "New minDistance " << minDistance << endl;
              (*decision) = forrAction;
              decisionMade = true;
            }
          }
        }
      }
      else if(dist_to_region < 100000){
        ROS_DEBUG("Action to get closest to visible Waypoint Region, Enforcer advisor to take decision");
        (*decision) = closest_action;
        decisionMade = true;
      }
    }
    else{
      ROS_DEBUG("Waypoint Region not in sight");
    }
    if(decisionMade == false and waypointPathInSight == true){
      ROS_DEBUG("Waypoint Region not in sight or no action can get there");
      vector<CartesianPoint> pathBetweenWaypoints;
      if(pathID == 0){
        pathBetweenWaypoints = beliefs->getAgentState()->getCurrentTask()->getSkeletonWaypoint().getPath();
      }
      else if(pathID == 1){
        pathBetweenWaypoints = beliefs->getAgentState()->getCurrentTask()->getSkeletonWaypoints()[1].getPath();
      }
      // cout << "pathBetweenWaypoints " << pathBetweenWaypoints.size() << endl;
      CartesianPoint farthestVisible;
      int farthest = -1;
      for(int i = pathBetweenWaypoints.size()-1; i >= 0; i--){
        if(beliefs->getAgentState()->canSeePoint(pathBetweenWaypoints[i], 20)){
          farthestVisible = pathBetweenWaypoints[i];
          // cout << "farthestVisible " << i << endl;
          farthest = i;
          break;
        }
      }
      if(farthest >= 0){
        (*decision) = beliefs->getAgentState()->moveTowards(farthestVisible);
        if(decision->parameter != 0){
          set<FORRAction> *vetoed_actions = beliefs->getAgentState()->getVetoedActions();
          if(vetoed_actions->find(*decision) != vetoed_actions->end()){
            decisionMade = false;
          }
          else{
            Position expectedPosition = beliefs->getAgentState()->getExpectedPositionAfterAction((*decision));
            if(expectedPosition.getDistance(beliefs->getAgentState()->getCurrentPosition()) >= 0.1){
              if(decision->type == RIGHT_TURN or decision->type == LEFT_TURN){
                ROS_DEBUG("Waypoint in sight and no obstacles and not vetoed, Enforcer advisor to take decision");
                decisionMade = true;
              }
              else{
                FORRAction forward = beliefs->getAgentState()->maxForwardAction();
                if(forward.parameter >= decision->parameter){
                  ROS_DEBUG("Waypoint in sight and no obstacles and not vetoed, Enforcer advisor to take decision");
                  decisionMade = true;
                  // Position currentPosition = beliefs->getAgentState()->getCurrentPosition();
                  // beliefs->getAgentState()->getCurrentTask()->updatePlanPositions(currentPosition.getX(), currentPosition.getY());
                  // if(decision->type == FORWARD)
                  //   beliefs->getAgentState()->setGetOutTriggered(false);
                }
              }
            }
          }
        }
      }
    }
  }
  return decisionMade;
}


/*
 * This advisor has to do prevent all the actions that will result in robot hitting the obstacles.
 * Straight forward thing is to check for collsion in the orientation.
 */
bool Tier1Advisor::advisorAvoidObstacles(){
  ROS_DEBUG("In advisor avoid obstacles");
  FORRAction max_forward = beliefs->getAgentState()->maxForwardAction();
  ROS_DEBUG_STREAM("Max allowed forward action : " << max_forward.type << " " << max_forward.parameter);
  int intensity = max_forward.parameter;
  set<FORRAction> *vetoedActions = beliefs->getAgentState()->getVetoedActions();
  set<FORRAction> *forward_set = beliefs->getAgentState()->getForwardActionSet();
  for(int i = forward_set->size()-1 ; i > 0; i--){
    FORRAction a(FORWARD,i);
    if(i > intensity){
      ROS_DEBUG_STREAM("Vetoed action : " << a.type << " " << a.parameter);
      vetoedActions->insert(a);
    }
    // Position expectedPosition = beliefs->getAgentState()->getExpectedPositionAfterAction(a);
    // cout << "expectedPosition " << expectedPosition.getX() << " " << expectedPosition.getY() << " Action : " << a.type << " " << a.parameter << endl;
  }
  set<FORRAction> *rotation_set = beliefs->getAgentState()->getRotationActionSet();
  for(int i = 1; i < rotation_set->size()/2+1 ; i++){
    FORRAction forrAction = FORRAction(RIGHT_TURN, i);
    Position expectedPosition = beliefs->getAgentState()->getExpectedPositionAfterAction(forrAction);
    // cout << "expectedPosition " << expectedPosition.getX() << " " << expectedPosition.getY() << " Action : " << forrAction.type << " " << forrAction.parameter << endl;
    if(expectedPosition.getDistance(beliefs->getAgentState()->getCurrentPosition()) < 0.1){
      (beliefs->getAgentState()->getVetoedActions()->insert(forrAction));
      ROS_DEBUG_STREAM("Vetoed action : " << forrAction.type << " " << forrAction.parameter);
    }
    forrAction = FORRAction(LEFT_TURN, i);
    expectedPosition = beliefs->getAgentState()->getExpectedPositionAfterAction(forrAction);
    // cout << "expectedPosition " << expectedPosition.getX() << " " << expectedPosition.getY() << " Action : " << forrAction.type << " " << forrAction.parameter << endl;
    if(expectedPosition.getDistance(beliefs->getAgentState()->getCurrentPosition()) < 0.1){
      (beliefs->getAgentState()->getVetoedActions()->insert(forrAction));
      ROS_DEBUG_STREAM("Vetoed action : " << forrAction.type << " " << forrAction.parameter);
    }
  }
  return false; 
}

bool Tier1Advisor::advisorDontGoBack(){
  ROS_DEBUG("In advisor don't go back");
  set<FORRAction> *vetoedActions = beliefs->getAgentState()->getVetoedActions();
  set<FORRAction> *action_set = beliefs->getAgentState()->getActionSet();
  set<FORRAction>::iterator actionIter;
  for(actionIter = action_set->begin(); actionIter != action_set->end(); actionIter++){
    FORRAction forrAction = *actionIter;
    if(std::find(vetoedActions->begin(), vetoedActions->end(), forrAction) != vetoedActions->end()){
      continue;
    }
    else if(forrAction.type == PAUSE or forrAction.type == FORWARD){
      continue;
    }
    else{
      Position expectedPosition = beliefs->getAgentState()->getExpectedPositionAfterAction(forrAction);
      if(beliefs->getAgentState()->getCurrentTask()->getPlanPositionValue(expectedPosition.getX(), expectedPosition.getY())){
        FORRAction a(forrAction.type,forrAction.parameter);
        ROS_DEBUG_STREAM("Vetoed action : " << a.type << " " << a.parameter);
        vetoedActions->insert(a);
      }
    }
  }
  cout << "Don't go back number of vetoes " << vetoedActions->size() << " " << action_set->size() << endl;
  if(vetoedActions->size() == beliefs->getAgentState()->getRotationActionSet()->size()){
    beliefs->getAgentState()->getCurrentTask()->resetPlanPositions();
  }
  return false; 
}

bool Tier1Advisor::advisorSituation(){
  ROS_DEBUG("In advisor situation");
  // Find closest situation based on similarity to median
  set<FORRAction> *vetoedActions = beliefs->getAgentState()->getVetoedActions();
  set<FORRAction> *action_set = beliefs->getAgentState()->getActionSet();
  double accuracy = beliefs->getSpatialModel()->getSituations()->getAccuracyForSituation(beliefs->getAgentState());
  cout << "Situation accuracy: " << accuracy << endl;
  if(accuracy >= 0.75){
    set<FORRAction>::iterator actionIter;
    for(actionIter = action_set->begin(); actionIter != action_set->end(); actionIter++){
      FORRAction forrAction = *actionIter;
      if(std::find(vetoedActions->begin(), vetoedActions->end(), forrAction) != vetoedActions->end()){
        continue;
      }
      else if(forrAction.type == PAUSE){
        continue;
      }
      else{
        double action_weight = beliefs->getSpatialModel()->getSituations()->getWeightForAction(beliefs->getAgentState(), forrAction);
        cout << "action_weight " << action_weight << endl;
        if(action_weight < 0.25){
          FORRAction a(forrAction.type,forrAction.parameter);
          ROS_DEBUG_STREAM("Vetoed action : " << a.type << " " << a.parameter);
          vetoedActions->insert(a);
        }
      }
    }
  }
  return false;
}

bool Tier1Advisor::advisorGetOut(FORRAction *decision) {
  ROS_DEBUG("Begin get out advisor");
  // if the robot is in a confined space and it sees a way out then take the action that does that.
  bool decisionMade = false;
  set<FORRAction> *rotation_set = beliefs->getAgentState()->getRotationActionSet();
  int decisionLimit = 10 + (int)(beliefs->getAgentState()->getCurrentTask()->getPositionHistory()->size()/50);
  if(beliefs->getAgentState()->getGetOutTriggered() == false and beliefs->getAgentState()->getRobotConfined(decisionLimit, 1)){
    vector<FORRAction> actions = beliefs->getAgentState()->getCurrentTask()->getPreviousDecisions();
    int size = actions.size();
    cout << "actions size " << size << " rotation size " << rotation_set->size()/2 << endl;
    if(size >= 5){
      FORRAction lastAction = actions[size - 1];
      FORRAction lastlastAction = actions[size - 2];
      FORRAction lastlastlastAction = actions[size - 3];
      FORRAction lastlastlastlastAction = actions[size - 4];
      cout << "lastAction " << lastAction.type << " " << lastAction.parameter << " lastlastAction " <<  lastlastAction.type << " " << lastlastAction.parameter << " lastlastlastAction " <<  lastlastlastAction.type << " " << lastlastlastAction.parameter << " lastlastlastlastAction " <<  lastlastlastlastAction.type << " " << lastlastlastlastAction.parameter << endl;
      if(lastlastAction.type == RIGHT_TURN and lastlastAction.parameter == rotation_set->size()/2 and lastAction.type == RIGHT_TURN and lastAction.parameter == rotation_set->size()/2 and lastlastlastAction.type == RIGHT_TURN and lastlastlastAction.parameter == rotation_set->size()/2 and lastlastlastlastAction.type == RIGHT_TURN and lastlastlastlastAction.parameter == rotation_set->size()/2){
        vector<Position> *positionHis = beliefs->getAgentState()->getAllPositionTrace();
        vector< vector <CartesianPoint> > *laserHis = beliefs->getAgentState()->getAllLaserHistory();
        CartesianPoint current_position = CartesianPoint(positionHis->at(positionHis->size()-1).getX(), positionHis->at(positionHis->size()-1).getY());
        cout << "current_position " << current_position.get_x() << " " << current_position.get_y() << endl;
        vector<Position> last_positions;
        vector< vector<CartesianPoint> > last_endpoints;
        int previous_count = 4;
        for(int i = 1; i < previous_count+1; i++){
          last_positions.push_back(positionHis->at(positionHis->size()-i));
          last_endpoints.push_back(laserHis->at(laserHis->size()-i));
        }
        // cout << last_positions.size() << " " << last_lasers.size() << " " << last_endpoints.size() << endl;
        cout << last_positions.size() << " " << last_endpoints.size() << endl;
        int last_invisible = 1;
        int last_visible = 1;
        for(int i = 2; i < positionHis->size(); i++){
          bool anyVisible = false;
          for(int j = 0; j < last_positions.size(); j++){
            if(beliefs->getAgentState()->canSeePoint(last_endpoints[j], CartesianPoint(last_positions[j].getX(), last_positions[j].getY()), CartesianPoint(positionHis->at(positionHis->size()-i).getX(), positionHis->at(positionHis->size()-i).getY()), 25)){
              anyVisible = true;
              break;
            }
          }
          if(anyVisible == false){
            last_invisible = i;
            break;
          }
          last_visible = i;
        }
        cout << "last_visible " << last_visible << " last_invisible " << last_invisible << endl;
        CartesianPoint last_invisible_point = CartesianPoint(positionHis->at(positionHis->size()-last_invisible).getX(), positionHis->at(positionHis->size()-last_invisible).getY());
        CartesianPoint last_visible_point = CartesianPoint(positionHis->at(positionHis->size()-last_visible).getX(), positionHis->at(positionHis->size()-last_visible).getY());
        cout << "last_invisible_point " << last_invisible_point.get_x() << " " << last_invisible_point.get_y() << endl;
        cout << "last_visible_point " << last_visible_point.get_x() << " " << last_visible_point.get_y() << endl;
        beliefs->getAgentState()->setGetOutTriggered(true);
        cout << "after setGetOutTriggered" << endl;
        beliefs->getAgentState()->setFarthestPoint(last_invisible_point);
        cout << "after setFarthestPoint" << endl;
        beliefs->getAgentState()->setIntermediatePoint(last_visible_point);
        cout << "after setIntermediatePoint" << endl;
        (*decision) = beliefs->getAgentState()->moveTowards(last_visible_point);
        ROS_DEBUG("last_visible_point in sight and no obstacles, get out advisor to take decision");
        decisionMade = true;
        cout << "after decisionMade" << endl;
        int end_waypoint = last_invisible + 2;
        if(end_waypoint > positionHis->size()){
          end_waypoint = positionHis->size();
        }
        cout << "end_waypoint " << end_waypoint << endl;;
        for(int i = end_waypoint; i >= 1; i--){
          cout << i << endl;
          beliefs->getAgentState()->getCurrentTask()->createNewWaypoint(CartesianPoint(positionHis->at(positionHis->size()-i).getX(), positionHis->at(positionHis->size()-i).getY()), true);
        }
      }
      else{
        // cout << "else make turn" << endl;
        (*decision) = FORRAction(RIGHT_TURN, rotation_set->size()/2);
        decisionMade = true;
      }
    }
  }
  else if(beliefs->getAgentState()->getGetOutTriggered() == true){
    CartesianPoint farthest_position = beliefs->getAgentState()->getFarthestPoint();
    CartesianPoint intermediate_position = beliefs->getAgentState()->getIntermediatePoint();
    vector<Position> *positionHis = beliefs->getAgentState()->getCurrentTask()->getPositionHistory();
    vector< vector <CartesianPoint> > *laserHis = beliefs->getAgentState()->getCurrentTask()->getLaserHistory();
    CartesianPoint current_position = CartesianPoint(positionHis->at(positionHis->size()-1).getX(), positionHis->at(positionHis->size()-1).getY());
    CartesianPoint last_position = CartesianPoint(positionHis->at(positionHis->size()-2).getX(), positionHis->at(positionHis->size()-2).getY());
    vector <CartesianPoint> current_laser = laserHis->at(laserHis->size()-1);
    double distance_from_last = current_position.get_distance(last_position);
    double distance_from_farthest = current_position.get_distance(farthest_position);
    double distance_from_intermediate = current_position.get_distance(intermediate_position);
    cout << "farthest_position " << farthest_position.get_x() << " " << farthest_position.get_y() << " intermediate_position " << intermediate_position.get_x() << " " << intermediate_position.get_y() << endl;
    cout << "current_position " << current_position.get_x() << " " << current_position.get_y() << " last_position " << last_position.get_x() << " " << last_position.get_y() << endl;
    cout << "distance_from_last " << distance_from_last << " distance_from_intermediate " << distance_from_intermediate << " distance_from_farthest " << distance_from_farthest << endl;
    FORRAction forward = beliefs->getAgentState()->maxForwardAction();
    if(distance_from_farthest <= 0.1){
      beliefs->getAgentState()->setGetOutTriggered(false);
    }
    else if(beliefs->getAgentState()->canSeePoint(farthest_position, 25)){
      cout << "farthest_position " << farthest_position.get_x() << " " << farthest_position.get_y() << endl;
      (*decision) = beliefs->getAgentState()->moveTowards(farthest_position);
      ROS_DEBUG("farthest_position in sight and no obstacles, get out advisor to take decision");
      decisionMade = true;
    }
    else if(distance_from_intermediate > 0.1 and beliefs->getAgentState()->canSeePoint(intermediate_position, 25)){
      cout << "intermediate_position " << intermediate_position.get_x() << " " << intermediate_position.get_y() << endl;
      (*decision) = beliefs->getAgentState()->moveTowards(intermediate_position);
      ROS_DEBUG("intermediate_position in sight and no obstacles, get out advisor to take decision");
      decisionMade = true;
    }
    else{
      CartesianPoint shifted_point1 = CartesianPoint(current_position.get_x()+0.5, current_position.get_y());
      CartesianPoint shifted_point2 = CartesianPoint(current_position.get_x()-0.5, current_position.get_y());
      CartesianPoint shifted_point3 = CartesianPoint(current_position.get_x(), current_position.get_y()+0.5);
      CartesianPoint shifted_point4 = CartesianPoint(current_position.get_x(), current_position.get_y()-0.5);
      CartesianPoint shifted_point5 = CartesianPoint(current_position.get_x()+0.5, current_position.get_y()+0.5);
      CartesianPoint shifted_point6 = CartesianPoint(current_position.get_x()-0.5, current_position.get_y()-0.5);
      CartesianPoint shifted_point7 = CartesianPoint(current_position.get_x()-0.5, current_position.get_y()+0.5);
      CartesianPoint shifted_point8 = CartesianPoint(current_position.get_x()+0.5, current_position.get_y()-0.5);
      vector<CartesianPoint> shifted_points;
      shifted_points.push_back(shifted_point1);
      shifted_points.push_back(shifted_point2);
      shifted_points.push_back(shifted_point3);
      shifted_points.push_back(shifted_point4);
      shifted_points.push_back(shifted_point5);
      shifted_points.push_back(shifted_point6);
      shifted_points.push_back(shifted_point7);
      shifted_points.push_back(shifted_point8);
      bool can_see_1 = beliefs->getAgentState()->canSeePoint(shifted_point1, 25);
      bool can_see_2 = beliefs->getAgentState()->canSeePoint(shifted_point2, 25);
      bool can_see_3 = beliefs->getAgentState()->canSeePoint(shifted_point3, 25);
      bool can_see_4 = beliefs->getAgentState()->canSeePoint(shifted_point4, 25);
      bool can_see_5 = beliefs->getAgentState()->canSeePoint(shifted_point5, 25);
      bool can_see_6 = beliefs->getAgentState()->canSeePoint(shifted_point6, 25);
      bool can_see_7 = beliefs->getAgentState()->canSeePoint(shifted_point7, 25);
      bool can_see_8 = beliefs->getAgentState()->canSeePoint(shifted_point8, 25);
      vector<bool> can_see_points;
      can_see_points.push_back(can_see_1);
      can_see_points.push_back(can_see_2);
      can_see_points.push_back(can_see_3);
      can_see_points.push_back(can_see_4);
      can_see_points.push_back(can_see_5);
      can_see_points.push_back(can_see_6);
      can_see_points.push_back(can_see_7);
      can_see_points.push_back(can_see_8);
      cout << "shifted_point1 " << shifted_point1.get_x() << " " << shifted_point1.get_y() << " can_see_1 " << can_see_1 << endl;
      cout << "shifted_point2 " << shifted_point2.get_x() << " " << shifted_point2.get_y() << " can_see_2 " << can_see_2 << endl;
      cout << "shifted_point3 " << shifted_point3.get_x() << " " << shifted_point3.get_y() << " can_see_3 " << can_see_3 << endl;
      cout << "shifted_point4 " << shifted_point4.get_x() << " " << shifted_point4.get_y() << " can_see_4 " << can_see_4 << endl;
      cout << "shifted_point5 " << shifted_point5.get_x() << " " << shifted_point5.get_y() << " can_see_5 " << can_see_5 << endl;
      cout << "shifted_point6 " << shifted_point6.get_x() << " " << shifted_point6.get_y() << " can_see_6 " << can_see_6 << endl;
      cout << "shifted_point7 " << shifted_point7.get_x() << " " << shifted_point7.get_y() << " can_see_7 " << can_see_7 << endl;
      cout << "shifted_point8 " << shifted_point8.get_x() << " " << shifted_point8.get_y() << " can_see_8 " << can_see_8 << endl;
      vector< pair <double,int> > distance_to_farthest;
      distance_to_farthest.push_back(make_pair(shifted_point1.get_distance(farthest_position),0));
      distance_to_farthest.push_back(make_pair(shifted_point2.get_distance(farthest_position),1));
      distance_to_farthest.push_back(make_pair(shifted_point3.get_distance(farthest_position),2));
      distance_to_farthest.push_back(make_pair(shifted_point4.get_distance(farthest_position),3));
      distance_to_farthest.push_back(make_pair(shifted_point5.get_distance(farthest_position),4));
      distance_to_farthest.push_back(make_pair(shifted_point6.get_distance(farthest_position),5));
      distance_to_farthest.push_back(make_pair(shifted_point7.get_distance(farthest_position),6));
      distance_to_farthest.push_back(make_pair(shifted_point8.get_distance(farthest_position),7));
      sort(distance_to_farthest.begin(), distance_to_farthest.end());
      if(can_see_points[distance_to_farthest[0].second]){
        // beliefs->getAgentState()->setIntermediatePoint(shifted_points[distance_to_farthest[0].second]);
        (*decision) = beliefs->getAgentState()->moveTowards(shifted_points[distance_to_farthest[0].second]);
        cout << "shifted_point" << distance_to_farthest[0].second+1 << " in sight and no obstacles, get out advisor to take decision" << endl;
        decisionMade = true;
      }
      else if(can_see_points[distance_to_farthest[1].second]){
        // beliefs->getAgentState()->setIntermediatePoint(shifted_points[distance_to_farthest[1].second]);
        (*decision) = beliefs->getAgentState()->moveTowards(shifted_points[distance_to_farthest[1].second]);
        cout << "shifted_point" << distance_to_farthest[1].second+1 << " in sight and no obstacles, get out advisor to take decision" << endl;
        decisionMade = true;
      }
      else if(can_see_points[distance_to_farthest[2].second]){
        // beliefs->getAgentState()->setIntermediatePoint(shifted_points[distance_to_farthest[2].second]);
        (*decision) = beliefs->getAgentState()->moveTowards(shifted_points[distance_to_farthest[2].second]);
        cout << "shifted_point" << distance_to_farthest[2].second+1 << " in sight and no obstacles, get out advisor to take decision" << endl;
        decisionMade = true;
      }
      else if(can_see_points[distance_to_farthest[3].second]){
        // beliefs->getAgentState()->setIntermediatePoint(shifted_points[distance_to_farthest[3].second]);
        (*decision) = beliefs->getAgentState()->moveTowards(shifted_points[distance_to_farthest[3].second]);
        cout << "shifted_point" << distance_to_farthest[3].second+1 << " in sight and no obstacles, get out advisor to take decision" << endl;
        decisionMade = true;
      }
      else if(can_see_points[distance_to_farthest[4].second]){
        // beliefs->getAgentState()->setIntermediatePoint(shifted_points[distance_to_farthest[4].second]);
        (*decision) = beliefs->getAgentState()->moveTowards(shifted_points[distance_to_farthest[4].second]);
        cout << "shifted_point" << distance_to_farthest[4].second+1 << " in sight and no obstacles, get out advisor to take decision" << endl;
        decisionMade = true;
      }
      else if(can_see_points[distance_to_farthest[5].second]){
        // beliefs->getAgentState()->setIntermediatePoint(shifted_points[distance_to_farthest[5].second]);
        (*decision) = beliefs->getAgentState()->moveTowards(shifted_points[distance_to_farthest[5].second]);
        cout << "shifted_point" << distance_to_farthest[5].second+1 << " in sight and no obstacles, get out advisor to take decision" << endl;
        decisionMade = true;
      }
      else if(can_see_points[distance_to_farthest[6].second]){
        // beliefs->getAgentState()->setIntermediatePoint(shifted_points[distance_to_farthest[6].second]);
        (*decision) = beliefs->getAgentState()->moveTowards(shifted_points[distance_to_farthest[6].second]);
        cout << "shifted_point" << distance_to_farthest[6].second+1 << " in sight and no obstacles, get out advisor to take decision" << endl;
        decisionMade = true;
      }
      else if(can_see_points[distance_to_farthest[7].second]){
        // beliefs->getAgentState()->setIntermediatePoint(shifted_points[distance_to_farthest[7].second]);
        (*decision) = beliefs->getAgentState()->moveTowards(shifted_points[distance_to_farthest[7].second]);
        cout << "shifted_point" << distance_to_farthest[7].second+1 << " in sight and no obstacles, get out advisor to take decision" << endl;
        decisionMade = true;
      }
      else{
        (*decision) = FORRAction(LEFT_TURN, rotation_set->size()/2);
        decisionMade = true;
      }
    }
  }
  cout << "decisionMade " << decisionMade << endl;
  return decisionMade;
}

// bool Tier1Advisor::advisorCircumnavigate(FORRAction *decision) {
//   ROS_DEBUG("Begin circumnavigate advisor");
//   // if the robot is in a confined space and it sees a way out then take the action that does that.
//   bool decisionMade = false;
//   if(beliefs->getAgentState()->getCurrentTask()->getIsPlanComplete() == true){
//     CartesianPoint task(beliefs->getAgentState()->getCurrentTask()->getTaskX(),beliefs->getAgentState()->getCurrentTask()->getTaskY());
//     cout << "Target = " << task.get_x() << " " << task.get_y() << endl;
//     Position currentPosition = beliefs->getAgentState()->getCurrentPosition();
//     cout << "Current Position = " << currentPosition.getX() << " " << currentPosition.getY() << endl;
//     double x_diff = task.get_x() - currentPosition.getX();
//     double y_diff = task.get_y() - currentPosition.getY();
//     if(abs(x_diff) > abs(y_diff)){
//       if(x_diff > 0){
//         beliefs->getAgentState()->setCurrentDirection(1);
//       }
//       else{
//         beliefs->getAgentState()->setCurrentDirection(2);
//       }
//       if(y_diff > 0){
//         beliefs->getAgentState()->setDesiredDirection(3);
//       }
//       else{
//         beliefs->getAgentState()->setDesiredDirection(4);
//       }
//     }
//     else{
//       if(x_diff > 0){
//         beliefs->getAgentState()->setDesiredDirection(1);
//       }
//       else{
//         beliefs->getAgentState()->setDesiredDirection(2);
//       }
//       if(y_diff > 0){
//         beliefs->getAgentState()->setCurrentDirection(3);
//       }
//       else{
//         beliefs->getAgentState()->setCurrentDirection(4);
//       }
//     }
//     cout << "Current Direction " << beliefs->getAgentState()->getCurrentDirection() << " Desired Direction " << beliefs->getAgentState()->getDesiredDirection() << endl;
//     CartesianPoint shifted_point1 = CartesianPoint(currentPosition.getX()+1, currentPosition.getY());
//     CartesianPoint shifted_point2 = CartesianPoint(currentPosition.getX()-1, currentPosition.getY());
//     CartesianPoint shifted_point3 = CartesianPoint(currentPosition.getX(), currentPosition.getY()+1);
//     CartesianPoint shifted_point4 = CartesianPoint(currentPosition.getX(), currentPosition.getY()-1);
//     vector<CartesianPoint> shifted_points;
//     shifted_points.push_back(shifted_point1);
//     shifted_points.push_back(shifted_point2);
//     shifted_points.push_back(shifted_point3);
//     shifted_points.push_back(shifted_point4);
//     bool can_see_1 = beliefs->getAgentState()->canSeePoint(shifted_point1, 25);
//     bool can_see_2 = beliefs->getAgentState()->canSeePoint(shifted_point2, 25);
//     bool can_see_3 = beliefs->getAgentState()->canSeePoint(shifted_point3, 25);
//     bool can_see_4 = beliefs->getAgentState()->canSeePoint(shifted_point4, 25);
//     vector<bool> can_see_points;
//     can_see_points.push_back(can_see_1);
//     can_see_points.push_back(can_see_2);
//     can_see_points.push_back(can_see_3);
//     can_see_points.push_back(can_see_4);
//     double robot_direction = currentPosition.getTheta();
//     double goal_direction = atan2((shifted_point1.get_y() - currentPosition.getY()), (shifted_point1.get_x() - currentPosition.getX()));
//     double required_rotation = goal_direction - robot_direction;
//     if(required_rotation > M_PI)
//       required_rotation = required_rotation - (2*M_PI);
//     if(required_rotation < -M_PI)
//       required_rotation = required_rotation + (2*M_PI);
//     double dist_to_1 = beliefs->getAgentState()->getDistanceToObstacle(required_rotation);
//     goal_direction = atan2((shifted_point2.get_y() - currentPosition.getY()), (shifted_point2.get_x() - currentPosition.getX()));
//     required_rotation = goal_direction - robot_direction;
//     if(required_rotation > M_PI)
//       required_rotation = required_rotation - (2*M_PI);
//     if(required_rotation < -M_PI)
//       required_rotation = required_rotation + (2*M_PI);
//     double dist_to_2 = beliefs->getAgentState()->getDistanceToObstacle(required_rotation);
//     goal_direction = atan2((shifted_point3.get_y() - currentPosition.getY()), (shifted_point3.get_x() - currentPosition.getX()));
//     required_rotation = goal_direction - robot_direction;
//     if(required_rotation > M_PI)
//       required_rotation = required_rotation - (2*M_PI);
//     if(required_rotation < -M_PI)
//       required_rotation = required_rotation + (2*M_PI);
//     double dist_to_3 = beliefs->getAgentState()->getDistanceToObstacle(required_rotation);
//     goal_direction = atan2((shifted_point4.get_y() - currentPosition.getY()), (shifted_point4.get_x() - currentPosition.getX()));
//     required_rotation = goal_direction - robot_direction;
//     if(required_rotation > M_PI)
//       required_rotation = required_rotation - (2*M_PI);
//     if(required_rotation < -M_PI)
//       required_rotation = required_rotation + (2*M_PI);
//     double dist_to_4 = beliefs->getAgentState()->getDistanceToObstacle(required_rotation);
//     vector<double> distances;
//     distances.push_back(dist_to_1);
//     distances.push_back(dist_to_2);
//     distances.push_back(dist_to_3);
//     distances.push_back(dist_to_4);
//     cout << "shifted_point1 " << shifted_point1.get_x() << " " << shifted_point1.get_y() << " can_see_1 " << can_see_1 << " dist_to_1 " << dist_to_1 << endl;
//     cout << "shifted_point2 " << shifted_point2.get_x() << " " << shifted_point2.get_y() << " can_see_2 " << can_see_2 << " dist_to_2 " << dist_to_2 << endl;
//     cout << "shifted_point3 " << shifted_point3.get_x() << " " << shifted_point3.get_y() << " can_see_3 " << can_see_3 << " dist_to_3 " << dist_to_3 << endl;
//     cout << "shifted_point4 " << shifted_point4.get_x() << " " << shifted_point4.get_y() << " can_see_4 " << can_see_4 << " dist_to_4 " << dist_to_4 << endl;
//     if(can_see_points[beliefs->getAgentState()->getCurrentDirection()-1] and beliefs->getAgentState()->getDirectionEnd() == false){
//       cout << "Adding current direction" << endl;
//       vector<CartesianPoint> new_waypoints;
//       for(double i = 0.2; i <= distances[beliefs->getAgentState()->getCurrentDirection()-1]; i+=0.2){
//         if(beliefs->getAgentState()->getCurrentDirection() == 1){
//           new_waypoints.insert(new_waypoints.begin(),CartesianPoint(currentPosition.getX()+i, currentPosition.getY()));
//         }
//         else if(beliefs->getAgentState()->getCurrentDirection() == 2){
//           new_waypoints.insert(new_waypoints.begin(),CartesianPoint(currentPosition.getX()-i, currentPosition.getY()));
//         }
//         else if(beliefs->getAgentState()->getCurrentDirection() == 3){
//           new_waypoints.insert(new_waypoints.begin(),CartesianPoint(currentPosition.getX(), currentPosition.getY()+i));
//         }
//         else if(beliefs->getAgentState()->getCurrentDirection() == 4){
//           new_waypoints.insert(new_waypoints.begin(),CartesianPoint(currentPosition.getX(), currentPosition.getY()-i));
//         }
//       }
//       for(int i = 0; i < new_waypoints.size(); i++){
//         beliefs->getAgentState()->getCurrentTask()->createNewWaypoint(new_waypoints[i], false);
//       }
//       // (*decision) = beliefs->getAgentState()->moveTowards(shifted_points[beliefs->getAgentState()->getCurrentDirection()-1]);
//       // decisionMade = true;
//     }
//     if(can_see_points[beliefs->getAgentState()->getDesiredDirection()-1] and beliefs->getAgentState()->getDirectionEnd() == false){
//       cout << "Adding desired direction" << endl;
//       vector<CartesianPoint> new_waypoints;
//       for(double i = 0.2; i <= distances[beliefs->getAgentState()->getDesiredDirection()-1]; i+=0.2){
//         if(beliefs->getAgentState()->getDesiredDirection() == 1){
//           new_waypoints.insert(new_waypoints.begin(),CartesianPoint(currentPosition.getX()+i, currentPosition.getY()));
//         }
//         else if(beliefs->getAgentState()->getDesiredDirection() == 2){
//           new_waypoints.insert(new_waypoints.begin(),CartesianPoint(currentPosition.getX()-i, currentPosition.getY()));
//         }
//         else if(beliefs->getAgentState()->getDesiredDirection() == 3){
//           new_waypoints.insert(new_waypoints.begin(),CartesianPoint(currentPosition.getX(), currentPosition.getY()+i));
//         }
//         else if(beliefs->getAgentState()->getDesiredDirection() == 4){
//           new_waypoints.insert(new_waypoints.begin(),CartesianPoint(currentPosition.getX(), currentPosition.getY()-i));
//         }
//       }
//       for(int i = 0; i < new_waypoints.size(); i++){
//         beliefs->getAgentState()->getCurrentTask()->createNewWaypoint(new_waypoints[i], false);
//       }
//       // (*decision) = beliefs->getAgentState()->moveTowards(shifted_points[beliefs->getAgentState()->getDesiredDirection()-1]);
//       // decisionMade = true;
//     }
//     // if(can_see_points[beliefs->getAgentState()->getCurrentDirection()-1] and can_see_points[beliefs->getAgentState()->getDesiredDirection()-1]){
//     //   beliefs->getAgentState()->setDirectionEnd(false);
//     // }
//     // if(!can_see_points[beliefs->getAgentState()->getCurrentDirection()-1] and !can_see_points[beliefs->getAgentState()->getDesiredDirection()-1] and beliefs->getAgentState()->getDirectionEnd() == false){
//     //   vector<Position> *positionHis = beliefs->getAgentState()->getAllPositionTrace();
//     //   vector< vector <CartesianPoint> > *laserHis = beliefs->getAgentState()->getAllLaserHistory();
//     //   CartesianPoint current_position = CartesianPoint(positionHis->at(positionHis->size()-1).getX(), positionHis->at(positionHis->size()-1).getY());
//     //   cout << "current_position " << current_position.get_x() << " " << current_position.get_y() << endl;
//     //   vector<Position> last_positions;
//     //   vector< vector<CartesianPoint> > last_endpoints;
//     //   int previous_count = 4;
//     //   for(int i = 1; i < previous_count+1; i++){
//     //     last_positions.push_back(positionHis->at(positionHis->size()-i));
//     //     last_endpoints.push_back(laserHis->at(laserHis->size()-i));
//     //   }
//     //   // cout << last_positions.size() << " " << last_lasers.size() << " " << last_endpoints.size() << endl;
//     //   // cout << last_positions.size() << " " << last_endpoints.size() << endl;
//     //   int last_invisible = 1;
//     //   int last_visible = 1;
//     //   for(int i = 2; i < positionHis->size(); i++){
//     //     bool anyVisible = false;
//     //     for(int j = 0; j < last_positions.size(); j++){
//     //       if(beliefs->getAgentState()->canSeePoint(last_endpoints[j], CartesianPoint(last_positions[j].getX(), last_positions[j].getY()), CartesianPoint(positionHis->at(positionHis->size()-i).getX(), positionHis->at(positionHis->size()-i).getY()), 25)){
//     //         anyVisible = true;
//     //         break;
//     //       }
//     //     }
//     //     if(anyVisible == false){
//     //       last_invisible = i;
//     //       break;
//     //     }
//     //     last_visible = i;
//     //   }
//     //   int end_waypoint = last_invisible + 5;
//     //   if(end_waypoint > positionHis->size()){
//     //     end_waypoint = positionHis->size();
//     //   }
//     //   cout << "end_waypoint " << end_waypoint << endl;;
//     //   for(int i = end_waypoint; i >= 1; i--){
//     //     cout << i << endl;
//     //     beliefs->getAgentState()->getCurrentTask()->createNewWaypoint(CartesianPoint(positionHis->at(positionHis->size()-i).getX(), positionHis->at(positionHis->size()-i).getY()), true);
//     //   }
//     //   beliefs->getAgentState()->setDirectionEnd(true);
//     // }
//     if(!can_see_points[beliefs->getAgentState()->getCurrentDirection()-1] and !can_see_points[beliefs->getAgentState()->getDesiredDirection()-1]){
//       Task* completedTask = beliefs->getAgentState()->getCurrentTask();
//       vector<Position> *pos_hist = completedTask->getPositionHistory();
//       vector< vector<CartesianPoint> > *laser_hist = completedTask->getLaserHistory();
//       vector< vector<CartesianPoint> > all_trace = beliefs->getAgentState()->getAllTrace();
//       vector<CartesianPoint> trace;
//       for(int i = 0 ; i < pos_hist->size() ; i++){
//         trace.push_back(CartesianPoint((*pos_hist)[i].getX(),(*pos_hist)[i].getY()));
//       }
//       all_trace.push_back(trace);
//       beliefs->getSpatialModel()->getRegionList()->learnRegions(pos_hist, laser_hist);
//       ROS_DEBUG("Regions Learned");
//       beliefs->getSpatialModel()->getRegionList()->clearAllExits();
//       beliefs->getSpatialModel()->getRegionList()->learnExits(all_trace);
//       ROS_DEBUG("Exits Learned");
//     }
//     // if(can_see_points[beliefs->getAgentState()->getDesiredDirection()-1] and distances[beliefs->getAgentState()->getDesiredDirection()-1] > distances[beliefs->getAgentState()->getCurrentDirection()-1]){
//     //   int dd = beliefs->getAgentState()->getDesiredDirection();
//     //   double x_diff = task.get_x() - currentPosition.getX();
//     //   double y_diff = task.get_y() - currentPosition.getY();
//     //   if(dd == 1 or dd == 2){
//     //     if(y_diff > 0){
//     //       beliefs->getAgentState()->setDesiredDirection(3);
//     //     }
//     //     else{
//     //       beliefs->getAgentState()->setDesiredDirection(4);
//     //     }
//     //   }
//     //   else{
//     //     if(x_diff > 0){
//     //       beliefs->getAgentState()->setDesiredDirection(1);
//     //     }
//     //     else{
//     //       beliefs->getAgentState()->setDesiredDirection(2);
//     //     }
//     //   }
//     //   beliefs->getAgentState()->setCurrentDirection(dd);
//     //   (*decision) = beliefs->getAgentState()->moveTowards(shifted_points[dd-1]);
//     //   decisionMade = true;
//     // }
//     // if(decisionMade == false){
//     //   if(can_see_points[beliefs->getAgentState()->getCurrentDirection()-1]){
//     //     (*decision) = beliefs->getAgentState()->moveTowards(shifted_points[beliefs->getAgentState()->getCurrentDirection()-1]);
//     //     decisionMade = true;
//     //   }
//     // }
//     // if(decisionMade == false){
//     //   beliefs->getAgentState()->setDirectionEnd(true);
//     //   if(beliefs->getAgentState()->getDesiredDirection() == 1){
//     //     (*decision) = beliefs->getAgentState()->moveTowards(shifted_points[1]);
//     //     decisionMade = true;
//     //   }
//     //   else if(beliefs->getAgentState()->getDesiredDirection() == 2){
//     //     (*decision) = beliefs->getAgentState()->moveTowards(shifted_points[0]);
//     //     decisionMade = true;
//     //   }
//     //   else if(beliefs->getAgentState()->getDesiredDirection() == 3){
//     //     (*decision) = beliefs->getAgentState()->moveTowards(shifted_points[3]);
//     //     decisionMade = true;
//     //   }
//     //   else if(beliefs->getAgentState()->getDesiredDirection() == 4){
//     //     (*decision) = beliefs->getAgentState()->moveTowards(shifted_points[2]);
//     //     decisionMade = true;
//     //   }
//       // if(can_see_1){
//       //   (*decision) = beliefs->getAgentState()->moveTowards(shifted_point1);
//       //   decisionMade = true;
//       // }
//       // else if(can_see_3){
//       //   (*decision) = beliefs->getAgentState()->moveTowards(shifted_point3);
//       //   decisionMade = true;
//       // }
//       // else if(can_see_2){
//       //   (*decision) = beliefs->getAgentState()->moveTowards(shifted_point2);
//       //   decisionMade = true;
//       // }
//       // else if(can_see_4){
//       //   (*decision) = beliefs->getAgentState()->moveTowards(shifted_point4);
//       //   decisionMade = true;
//       // }
//       // else{
//       //   (*decision) = FORRAction(FORWARD, 2);
//       //   decisionMade = true;
//       // }
//     // }
//   }
//   cout << "decisionMade " << decisionMade << endl;
//   return decisionMade;
// }



// (*decision) = beliefs->getAgentState()->moveTowards(task);
//     FORRAction forward = beliefs->getAgentState()->maxForwardAction();
//     Position expectedPosition = beliefs->getAgentState()->getExpectedPositionAfterAction((*decision));
//     if(((decision->type == RIGHT_TURN or decision->type == LEFT_TURN) or (forward.parameter >= decision->parameter)) and decision->parameter != 0 and expectedPosition.getDistance(beliefs->getAgentState()->getCurrentPosition()) >= 0.1){
//       ROS_DEBUG("Target in sight and no obstacles, victory advisor to take decision");