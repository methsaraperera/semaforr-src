
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
  ROS_DEBUG("leaving notOpposite");
  return;
}

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
          }
        }
      }
    }
  }
  if(decisionMade == true){
    beliefs->getAgentState()->setRepositionTriggered(false);
  }
  return decisionMade;
}

bool Tier1Advisor::advisorEnforcer(FORRAction *decision) {
  ROS_DEBUG("Begin Enforcer advisor");
  // if the robot is oriented towards the goal and the robot actions which are not vetoed allows the robot to reach the goal then take that action.
  bool decisionMade = false;
  ROS_DEBUG("Check if waypoint can be spotted using laser scan");
  cout << "PlannerName " << beliefs->getAgentState()->getCurrentTask()->getPlannerName() << " PlanSize " << beliefs->getAgentState()->getCurrentTask()->getPlanSize() << endl;
  if(beliefs->getAgentState()->getCurrentTask()->getPlannerName() != "skeleton" and beliefs->getAgentState()->getCurrentTask()->getPlannerName() != "hallwayskel"){
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
  else if(beliefs->getAgentState()->getCurrentTask()->getPlannerName() == "skeleton" and beliefs->getAgentState()->getCurrentTask()->getPlanSize() > 0){
    cout << "Waypoint Region = " << beliefs->getAgentState()->getCurrentTask()->getX() << " " << beliefs->getAgentState()->getCurrentTask()->getY() << endl;
    bool waypointRegionInSight = false;
    int regionID = -1;
    bool nextWaypointRegionInSight = false;
    int nextRegionID = -1;
    bool waypointPathInSight = false;
    int pathID = -1;
    double waypointDistThreshold = 20;
    if(beliefs->getAgentState()->getCurrentTask()->getSkeletonWaypoint().getType() == 0){
      waypointRegionInSight = beliefs->getAgentState()->canSeeRegion(beliefs->getAgentState()->getCurrentTask()->getSkeletonWaypoint().getRegion().getCenter(), beliefs->getAgentState()->getCurrentTask()->getSkeletonWaypoint().getRegion().getRadius(), waypointDistThreshold);
      regionID = 0;
    }
    else{
      if(beliefs->getAgentState()->getCurrentTask()->getPlanSize() > 1){
        if(beliefs->getAgentState()->getCurrentTask()->getSkeletonWaypoints()[1].getType() == 0){
          waypointRegionInSight = beliefs->getAgentState()->canSeeRegion(beliefs->getAgentState()->getCurrentTask()->getSkeletonWaypoints()[1].getRegion().getCenter(), beliefs->getAgentState()->getCurrentTask()->getSkeletonWaypoints()[1].getRegion().getRadius(), waypointDistThreshold);
          regionID = 1;
        }
      }
    }
    if(beliefs->getAgentState()->getCurrentTask()->getSkeletonWaypoint().getType() == 0 and beliefs->getAgentState()->getCurrentTask()->getPlanSize() >= 3){
      nextWaypointRegionInSight = beliefs->getAgentState()->canSeeRegion(beliefs->getAgentState()->getCurrentTask()->getSkeletonWaypoints()[2].getRegion().getCenter(), beliefs->getAgentState()->getCurrentTask()->getSkeletonWaypoints()[2].getRegion().getRadius(), waypointDistThreshold);
      nextRegionID = 2;
    }
    else if(beliefs->getAgentState()->getCurrentTask()->getPlanSize() >= 4 and !(beliefs->getAgentState()->getCurrentTask()->getSkeletonWaypoint().getType() == 0)){
      nextWaypointRegionInSight = beliefs->getAgentState()->canSeeRegion(beliefs->getAgentState()->getCurrentTask()->getSkeletonWaypoints()[3].getRegion().getCenter(), beliefs->getAgentState()->getCurrentTask()->getSkeletonWaypoints()[3].getRegion().getRadius(), waypointDistThreshold);
      nextRegionID = 3;
    }
    if(!(beliefs->getAgentState()->getCurrentTask()->getSkeletonWaypoint().getType() == 0)){
      vector<CartesianPoint> pathBetween = beliefs->getAgentState()->getCurrentTask()->getSkeletonWaypoint().getPath();
      for(int i = 0; i < pathBetween.size(); i++){
        if(beliefs->getAgentState()->canSeePoint(pathBetween[i], waypointDistThreshold)){
          waypointPathInSight = true;
          pathID = 0;
          break;
        }
      }
    }
    else{
      if(beliefs->getAgentState()->getCurrentTask()->getPlanSize() > 1){
        if(!(beliefs->getAgentState()->getCurrentTask()->getSkeletonWaypoints()[1].getType() == 0)){
          vector<CartesianPoint> pathBetween = beliefs->getAgentState()->getCurrentTask()->getSkeletonWaypoints()[1].getPath();
          for(int i = 0; i < pathBetween.size(); i++){
            if(beliefs->getAgentState()->canSeePoint(pathBetween[i], waypointDistThreshold)){
              waypointPathInSight = true;
              pathID = 1;
              break;
            }
          }
        }
      }
    }
    // cout << "waypointRegionInSight " << waypointRegionInSight << " regionID " << regionID << " nextWaypointRegionInSight " << nextWaypointRegionInSight << " nextRegionID " << nextRegionID << " waypointPathInSight " << waypointPathInSight << " pathID " << pathID << endl;
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
          if(nextRegionID == 2){
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
          else if(nextRegionID == 3){
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
        if(nextRegionID == 2){
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
        else if(nextRegionID == 3){
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
  else if(beliefs->getAgentState()->getCurrentTask()->getPlannerName() == "hallwayskel" and beliefs->getAgentState()->getCurrentTask()->getPlanSize() > 0){
    cout << "Passage Waypoint = " << beliefs->getAgentState()->getCurrentTask()->getX() << " " << beliefs->getAgentState()->getCurrentTask()->getY() << endl;
    bool waypointRegionInSight = false;
    int regionID = -1;
    bool nextWaypointRegionInSight = false;
    int nextRegionID = -1;
    bool waypointPathInSight = false;
    int pathID = -1;
    bool intersectionInSight = false;
    int intersectionID = -1;
    bool nextIntersectionInSight = false;
    int nextIntersectionID = -1;
    bool passageInSight = false;
    int passageID = -1;
    int lookAhead = 3;
    if(beliefs->getAgentState()->getCurrentTask()->getPlanSize() < lookAhead){
      lookAhead = beliefs->getAgentState()->getCurrentTask()->getPlanSize();
    }
    for(int i = lookAhead-1; i >=0; i--){
      if(beliefs->getAgentState()->getCurrentTask()->getSkeletonWaypoints()[i].getType() == 0){
        regionID = i;
        waypointRegionInSight = beliefs->getAgentState()->canSeeRegion(beliefs->getAgentState()->getCurrentTask()->getSkeletonWaypoints()[regionID].getRegion().getCenter(), beliefs->getAgentState()->getCurrentTask()->getSkeletonWaypoints()[regionID].getRegion().getRadius(), 25);
        if(waypointRegionInSight){
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
              if(beliefs->getAgentState()->getCurrentTask()->getSkeletonWaypoints()[regionID].getRegion().inRegion(expectedPosition.getX(), expectedPosition.getY())){
                FORRAction a(forrAction.type,forrAction.parameter);
                ROS_DEBUG_STREAM("Potential action : " << a.type << " " << a.parameter);
                possible_set.insert(a);
              }
              else{
                ROS_DEBUG_STREAM("Get closer action : " << forrAction.type << " " << forrAction.parameter);
                double dist_from_exp = beliefs->getAgentState()->getCurrentTask()->getSkeletonWaypoints()[regionID].getRegion().getCenter().get_distance(CartesianPoint(expectedPosition.getX(), expectedPosition.getY()));
                // cout << dist_from_exp << endl;
                if(dist_from_exp < dist_to_region){
                  dist_to_region = dist_from_exp;
                  closest_action = forrAction;
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
            double minDistance = 1000000;
            for(actionIter = possible_set.begin(); actionIter != possible_set.end(); actionIter++){
              FORRAction forrAction = *actionIter;
              ROS_DEBUG_STREAM("Potential action : " << forrAction.type << " " << forrAction.parameter);
              Position expectedPosition = beliefs->getAgentState()->getExpectedPositionAfterAction(forrAction);
              if(beliefs->getAgentState()->getCurrentTask()->getSkeletonWaypoints()[regionID].getRegion().getCenter().get_distance(CartesianPoint(expectedPosition.getX(), expectedPosition.getY())) < minDistance){
                minDistance = beliefs->getAgentState()->getCurrentTask()->getSkeletonWaypoints()[regionID].getRegion().getCenter().get_distance(CartesianPoint(expectedPosition.getX(), expectedPosition.getY()));
                // cout << "New minDistance " << minDistance << endl;
                (*decision) = forrAction;
                decisionMade = true;
              }
            }
          }
          else if(dist_to_region < 100000){
            ROS_DEBUG("Action to get closest to visible Waypoint Region, Enforcer advisor to take decision");
            (*decision) = closest_action;
            decisionMade = true;
          }
          if(decisionMade){
            break;
          }
        }
      }
      else if(beliefs->getAgentState()->getCurrentTask()->getSkeletonWaypoints()[i].getType() == 1){
        pathID = i;
        vector<CartesianPoint> pathBetween = beliefs->getAgentState()->getCurrentTask()->getSkeletonWaypoints()[pathID].getPath();
        for(int j = 0; j < pathBetween.size(); j++){
          if(beliefs->getAgentState()->canSeePoint(pathBetween[j], 20)){
            waypointPathInSight = true;
            break;
          }
        }
      }
      else if(beliefs->getAgentState()->getCurrentTask()->getSkeletonWaypoints()[i].getType() == 2){
        intersectionID = i;
        vector< vector<int> > passagePoints = beliefs->getAgentState()->getCurrentTask()->getSkeletonWaypoints()[intersectionID].getPassagePoints();
        for(int j = 0; j < passagePoints.size(); j++){
          if(beliefs->getAgentState()->canSeePoint(CartesianPoint(passagePoints[j][0], passagePoints[j][1]), 20)){
            intersectionInSight = true;
            break;
          }
        }
        if(intersectionInSight){
          ROS_DEBUG("Waypoint Intersection in sight, Enforcer advisor active");
          CartesianPoint intersectionCentroid = beliefs->getAgentState()->getCurrentTask()->getSkeletonWaypoints()[intersectionID].getPassageCentroid();
          bool intersectionCentroidInSight = beliefs->getAgentState()->canSeePoint(intersectionCentroid, 20);
          if(intersectionCentroidInSight){
            ROS_DEBUG("Waypoint Intersection Centroid in sight");
            (*decision) = beliefs->getAgentState()->moveTowards(intersectionCentroid);
            if(decision->parameter != 0){
              set<FORRAction> *vetoed_actions = beliefs->getAgentState()->getVetoedActions();
              if(vetoed_actions->find(*decision) != vetoed_actions->end()){
                decisionMade = false;
              }
              else{
                Position expectedPosition = beliefs->getAgentState()->getExpectedPositionAfterAction((*decision));
                if(expectedPosition.getDistance(beliefs->getAgentState()->getCurrentPosition()) >= 0.1){
                  if(decision->type == RIGHT_TURN or decision->type == LEFT_TURN){
                    ROS_DEBUG("Waypoint centroid in sight and no obstacles and not vetoed, Enforcer advisor to take decision");
                    decisionMade = true;
                  }
                  else{
                    FORRAction forward = beliefs->getAgentState()->maxForwardAction();
                    if(forward.parameter >= decision->parameter){
                      ROS_DEBUG("Waypoint centroid in sight and no obstacles and not vetoed, Enforcer advisor to take decision");
                      decisionMade = true;
                    }
                  }
                }
              }
            }
          }
          // if(decisionMade){
          //   break;
          // }
        }
      }
      else if(beliefs->getAgentState()->getCurrentTask()->getSkeletonWaypoints()[i].getType() == 3){
        passageID = i;
        vector<CartesianPoint> pathBetween = beliefs->getAgentState()->getCurrentTask()->getSkeletonWaypoints()[passageID].getPath();
        for(int j = 0; j < pathBetween.size(); j++){
          if(beliefs->getAgentState()->canSeePoint(pathBetween[j], 20)){
            passageInSight = true;
            break;
          }
        }
        if(passageInSight){
          ROS_DEBUG("Waypoint Passage in sight, Enforcer advisor active");
          CartesianPoint farthestVisible;
          int farthest = -1;
          for(int i = pathBetween.size()-1; i >= 0; i--){
            if(beliefs->getAgentState()->canSeePoint(pathBetween[i], 20)){
              farthestVisible = pathBetween[i];
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
                    ROS_DEBUG("Waypoint Passage path in sight and no obstacles and not vetoed, Enforcer advisor to take decision");
                    decisionMade = true;
                  }
                  else{
                    FORRAction forward = beliefs->getAgentState()->maxForwardAction();
                    if(forward.parameter >= decision->parameter){
                      ROS_DEBUG("Waypoint Passage path in sight and no obstacles and not vetoed, Enforcer advisor to take decision");
                      decisionMade = true;
                    }
                  }
                }
              }
            }
          }
        }
        // if(decisionMade){
        //   break;
        // }
      }
    }

    // cout << "waypointRegionInSight " << waypointRegionInSight << " regionID " << regionID << " nextWaypointRegionInSight " << nextWaypointRegionInSight << " nextRegionID " << nextRegionID << " waypointPathInSight " << waypointPathInSight << " pathID " << pathID << " intersectionInSight " << intersectionInSight << " intersectionID " << intersectionID << " nextIntersectionInSight " << nextIntersectionInSight << " nextIntersectionID " << nextIntersectionID << " passageInSight " << passageInSight << " passageID " << passageID << endl;
    if(decisionMade == false and pathID > -1 and waypointPathInSight){
      ROS_DEBUG("Waypoint Region or Intersection not in sight or no action can get there, try waypoint path");
      vector<CartesianPoint> pathBetweenWaypoints = beliefs->getAgentState()->getCurrentTask()->getSkeletonWaypoints()[pathID].getPath();
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
                ROS_DEBUG("Waypoint path in sight and no obstacles and not vetoed, Enforcer advisor to take decision");
                decisionMade = true;
              }
              else{
                FORRAction forward = beliefs->getAgentState()->maxForwardAction();
                if(forward.parameter >= decision->parameter){
                  ROS_DEBUG("Waypoint path in sight and no obstacles and not vetoed, Enforcer advisor to take decision");
                  decisionMade = true;
                }
              }
            }
          }
        }
      }
    }
    else{
      ROS_DEBUG("Waypoint Path not in sight");
    }
    if(decisionMade == false){
      ROS_DEBUG("No Plan element in sight");
    }

  }
  if(decisionMade == true){
    beliefs->getAgentState()->setRepositionTriggered(false);
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
    // cout << "actions size " << size << " rotation size " << rotation_set->size()/2 << endl;
    if(size >= 5){
      FORRAction lastAction = actions[size - 1];
      FORRAction lastlastAction = actions[size - 2];
      FORRAction lastlastlastAction = actions[size - 3];
      FORRAction lastlastlastlastAction = actions[size - 4];
      // cout << "lastAction " << lastAction.type << " " << lastAction.parameter << " lastlastAction " <<  lastlastAction.type << " " << lastlastAction.parameter << " lastlastlastAction " <<  lastlastlastAction.type << " " << lastlastlastAction.parameter << " lastlastlastlastAction " <<  lastlastlastlastAction.type << " " << lastlastlastlastAction.parameter << endl;
      if(lastlastAction.type == RIGHT_TURN and lastlastAction.parameter == rotation_set->size()/2 and lastAction.type == RIGHT_TURN and lastAction.parameter == rotation_set->size()/2 and lastlastlastAction.type == RIGHT_TURN and lastlastlastAction.parameter == rotation_set->size()/2 and lastlastlastlastAction.type == RIGHT_TURN and lastlastlastlastAction.parameter == rotation_set->size()/2){
        vector<Position> *positionHis = beliefs->getAgentState()->getAllPositionTrace();
        vector< vector <CartesianPoint> > *laserHis = beliefs->getAgentState()->getAllLaserHistory();
        CartesianPoint current_position = CartesianPoint(positionHis->at(positionHis->size()-1).getX(), positionHis->at(positionHis->size()-1).getY());
        // cout << "current_position " << current_position.get_x() << " " << current_position.get_y() << endl;
        vector<Position> last_positions;
        vector< vector<CartesianPoint> > last_endpoints;
        int previous_count = 4;
        for(int i = 1; i < previous_count+1; i++){
          last_positions.push_back(positionHis->at(positionHis->size()-i));
          last_endpoints.push_back(laserHis->at(laserHis->size()-i));
        }
        // cout << last_positions.size() << " " << last_lasers.size() << " " << last_endpoints.size() << endl;
        // cout << last_positions.size() << " " << last_endpoints.size() << endl;
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
        // cout << "last_visible " << last_visible << " last_invisible " << last_invisible << endl;
        CartesianPoint last_invisible_point = CartesianPoint(positionHis->at(positionHis->size()-last_invisible).getX(), positionHis->at(positionHis->size()-last_invisible).getY());
        CartesianPoint last_visible_point = CartesianPoint(positionHis->at(positionHis->size()-last_visible).getX(), positionHis->at(positionHis->size()-last_visible).getY());
        // cout << "last_invisible_point " << last_invisible_point.get_x() << " " << last_invisible_point.get_y() << endl;
        // cout << "last_visible_point " << last_visible_point.get_x() << " " << last_visible_point.get_y() << endl;
        beliefs->getAgentState()->setGetOutTriggered(true);
        // cout << "after setGetOutTriggered" << endl;
        beliefs->getAgentState()->setFarthestPoint(last_invisible_point);
        // cout << "after setFarthestPoint" << endl;
        beliefs->getAgentState()->setIntermediatePoint(last_visible_point);
        // cout << "after setIntermediatePoint" << endl;
        // (*decision) = beliefs->getAgentState()->moveTowards(last_visible_point);
        // ROS_DEBUG("last_visible_point in sight and no obstacles, get out advisor to take decision");
        // decisionMade = true;
        // cout << "after decisionMade" << endl;
        int end_waypoint = last_invisible + 2;
        if(end_waypoint > positionHis->size()){
          end_waypoint = positionHis->size();
        }
        // cout << "end_waypoint " << end_waypoint << endl;
        std::vector<CartesianPoint> trailPositions;
        trailPositions.push_back(CartesianPoint(positionHis->at(positionHis->size()-end_waypoint).getX(), positionHis->at(positionHis->size()-end_waypoint).getY()));
        // Find the furthest point on path that can be seen from current position, push that point to trail and then move to that point
        for(int i = end_waypoint; i >= 1; i--){
          for(int j = 1; j < i; j++){
            if(canAccessPoint(laserHis->at(positionHis->size()-i), CartesianPoint(positionHis->at(positionHis->size()-i).getX(), positionHis->at(positionHis->size()-i).getY()), CartesianPoint(positionHis->at(positionHis->size()-j).getX(), positionHis->at(positionHis->size()-j).getY()), 3)){
              trailPositions.push_back(CartesianPoint(positionHis->at(positionHis->size()-j).getX(), positionHis->at(positionHis->size()-j).getY()));
              i = j+1;
            }
          }
          // cout << i << endl;
        }
        for(int i = 0; i < trailPositions.size(); i++){
          beliefs->getAgentState()->getCurrentTask()->createNewWaypoint(trailPositions[i], true);
        }
      }
      else{
        // cout << "else make turn" << endl;
        (*decision) = FORRAction(RIGHT_TURN, rotation_set->size()/2);
        ROS_DEBUG("Rotate in place, get out advisor to take decision");
        decisionMade = true;
      }
    }
  }
  // cout << "decisionMade " << decisionMade << endl;
  return decisionMade;
}

bool Tier1Advisor::advisorDoorway(FORRAction *decision){
  ROS_DEBUG("In advisor doorway");
  bool decisionMade = false;
  if(beliefs->getAgentState()->getRepositionTriggered()){
    cout << "Reposition already triggered, move towards point" << endl;
    (*decision) = beliefs->getAgentState()->moveTowards(beliefs->getAgentState()->getRepositionPoint());
    if(decision->parameter != 0){
      Position expectedPosition = beliefs->getAgentState()->getExpectedPositionAfterAction((*decision));
      if(expectedPosition.getDistance(beliefs->getAgentState()->getCurrentPosition()) >= 0.1){
        if(decision->type == RIGHT_TURN or decision->type == LEFT_TURN){
          ROS_DEBUG("Reposition point in sight and no obstacles, Doorway advisor to take decision");
          decisionMade = true;
        }
        else{
          FORRAction forward = beliefs->getAgentState()->maxForwardAction();
          if(forward.parameter >= decision->parameter){
            ROS_DEBUG("Reposition point in sight and no obstacles, Doorway advisor to take decision");
            decisionMade = true;
          }
        }
      }
    }
  }
  else{
    CartesianPoint task(beliefs->getAgentState()->getCurrentTask()->getTaskX(),beliefs->getAgentState()->getCurrentTask()->getTaskY());
    ROS_DEBUG("Check if target or waypoint can be spotted using laser scan");
    cout << "Target = " << task.get_x() << " " << task.get_y() << endl;
    bool targetInSight = beliefs->getAgentState()->canSeePoint(task, 20);
    CartesianPoint waypoint(beliefs->getAgentState()->getCurrentTask()->getX(),beliefs->getAgentState()->getCurrentTask()->getY());
    cout << "Waypoint = " << waypoint.get_x() << " " << waypoint.get_y() << endl;
    bool waypointInSight = beliefs->getAgentState()->canSeePoint(waypoint, 20);
    cout << "targetInSight " << targetInSight << " waypointInSight " << waypointInSight << endl;
    CartesianPoint subgoal;
    if(targetInSight){
      subgoal = task;
    }
    else if(waypointInSight){
      subgoal = waypoint;
    }
    if(targetInSight or waypointInSight){
      std::vector<CartesianPoint> givenLaserEndpoints = beliefs->getAgentState()->getCurrentLaserEndpoints();
      CartesianPoint laserPos = CartesianPoint(beliefs->getAgentState()->getCurrentPosition().getX(), beliefs->getAgentState()->getCurrentPosition().getY());
      double point_direction = atan2((subgoal.get_y() - laserPos.get_y()), (subgoal.get_x() - laserPos.get_x()));
      cout << "Robot position " << laserPos.get_x() << " " << laserPos.get_y() << " " << beliefs->getAgentState()->getCurrentPosition().getTheta() << " point_direction " << point_direction << endl;
      int index = 0;
      double min_angle = 100000;
      for(int i = 0; i < givenLaserEndpoints.size(); i++){
        double laser_direction = atan2((givenLaserEndpoints[i].get_y() - laserPos.get_y()), (givenLaserEndpoints[i].get_x() - laserPos.get_x()));
        double angle_diff = laser_direction - point_direction;
        if(angle_diff > M_PI)
          angle_diff = angle_diff - 2*M_PI;
        if(angle_diff < -M_PI)
          angle_diff = angle_diff + 2*M_PI;
        angle_diff = fabs(angle_diff);
        // cout << "point_direction " << point_direction << " laser_direction " << laser_direction << " angle_diff " << angle_diff << endl;
        if(angle_diff < min_angle){
          // cout << "Laser Direction : " << laser_direction << ", Point Direction : " << point_direction << endl;
          min_angle = angle_diff;
          index = i;
        }
      }
      if(index + 1 >= givenLaserEndpoints.size()){
        index = index - 1;
      }
      cout << "min_angle " << min_angle << " index " << index << " endpoint " << givenLaserEndpoints[index].get_x() << " " << givenLaserEndpoints[index].get_y() << endl;
      double distLeft = 0, xLeft = 0, yLeft = 0, countLeft = 0;
      double distRight = 0, xRight = 0, yRight = 0, countRight = 0;
      for(int i = 0; i < index; i++){
        double dist_to_endpoint = givenLaserEndpoints[i].get_distance(laserPos);
        distLeft += dist_to_endpoint;
        countLeft++;
        xLeft += givenLaserEndpoints[i].get_x();
        yLeft += givenLaserEndpoints[i].get_y();
      }
      for(int i = index + 1; i < givenLaserEndpoints.size(); i++){
        double dist_to_endpoint = givenLaserEndpoints[i].get_distance(laserPos);
        distRight += dist_to_endpoint;
        countRight++;
        xRight += givenLaserEndpoints[i].get_x();
        yRight += givenLaserEndpoints[i].get_y();
      }
      cout << "distLeft " << distLeft << " xLeft " << xLeft << " yLeft " << yLeft << " countLeft " << countLeft << " distRight " << distRight << " xRight " << xRight << " yRight " << yRight << " countRight " << countRight << endl;
      distLeft = distLeft / countLeft;
      xLeft = xLeft / countLeft;
      yLeft = yLeft / countLeft;
      distRight = distRight / countRight;
      xRight = xRight / countRight;
      yRight = yRight / countRight;
      cout << "distLeft " << distLeft << " xLeft " << xLeft << " yLeft " << yLeft << " countLeft " << countLeft << " distRight " << distRight << " xRight " << xRight << " yRight " << yRight << " countRight " << countRight << endl;
      if(distLeft > distRight){
        double tx, ty;
        for(double j = 0; j <= 1; j += 0.1){
          tx = (laserPos.get_x() * j) + (xLeft * (1 - j));
          ty = (laserPos.get_y() * j) + (yLeft * (1 - j));
          if(beliefs->getAgentState()->canSeePoint(CartesianPoint(tx, ty), 20)){
            break;
          }
        }
        beliefs->getAgentState()->setRepositionPoint(CartesianPoint(tx, ty));
        beliefs->getAgentState()->setRepositionTriggered(true);
      }
      else{
        double tx, ty;
        for(double j = 0; j <= 1; j += 0.1){
          tx = (laserPos.get_x() * j) + (xRight * (1 - j));
          ty = (laserPos.get_y() * j) + (yRight * (1 - j));
          if(beliefs->getAgentState()->canSeePoint(CartesianPoint(tx, ty), 20)){
            break;
          }
        }
        beliefs->getAgentState()->setRepositionPoint(CartesianPoint(tx, ty));
        beliefs->getAgentState()->setRepositionTriggered(true);
      }
      if(beliefs->getAgentState()->getRepositionTriggered()){
        cout << "Reposition triggered, move towards point" << endl;
        (*decision) = beliefs->getAgentState()->moveTowards(beliefs->getAgentState()->getRepositionPoint());
        if(decision->parameter != 0){
          Position expectedPosition = beliefs->getAgentState()->getExpectedPositionAfterAction((*decision));
          if(expectedPosition.getDistance(beliefs->getAgentState()->getCurrentPosition()) >= 0.1){
            if(decision->type == RIGHT_TURN or decision->type == LEFT_TURN){
              ROS_DEBUG("Reposition point in sight and no obstacles, Doorway advisor to take decision");
              decisionMade = true;
            }
            else{
              FORRAction forward = beliefs->getAgentState()->maxForwardAction();
              if(forward.parameter >= decision->parameter){
                ROS_DEBUG("Reposition point in sight and no obstacles, Doorway advisor to take decision");
                decisionMade = true;
              }
            }
          }
        }
      }
    }
  }
  return decisionMade;
}

bool Tier1Advisor::advisorFindAWay(FORRAction *decision){
  ROS_DEBUG("In advisor FindAWay");
  bool decisionMade = false;
  if(beliefs->getAgentState()->getCurrentTask()->getPlanSize() == 0 or !beliefs->getAgentState()->getCurrentTask()->getIsPlanActive()){
    cout << "No active plan, try to do local exploration" << endl;
    vector< vector<Position> > remaining_candidates = beliefs->getAgentState()->getRemainingCandidates();
  }
  return decisionMade;
}