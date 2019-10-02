#include<FORRSituations.h>

using namespace std;



//----------------------//------------------------//


void FORRSituations::addObservationToSituations(sensor_msgs::LaserScan ls, Position pose, bool add_to_existing, FORRAction actual_action) {
  double angle = ls.angle_min;
  double increment = ls.angle_increment;
  vector<float> laser_ranges = ls.ranges;
  vector< vector<int> > grid;
  for(int i = 0; i < 51; i++){
    vector<int> col;
    for(int j = 0; j < 51; j++){
      col.push_back(0);
    }
    grid.push_back(col);
  }
  for(int i = 0; i < laser_ranges.size(); i++){
    for(double j = 0.0; j <= laser_ranges[i]; j+=0.9){
      int x = (int)(round(j * cos(angle)))+25;
      int y = (int)(round(j * sin(angle)))+25;
      grid[x][y] = 1;
      x = (int)(j * cos(angle))+25;
      y = (int)(j * sin(angle))+25;
      grid[x][y] = 1;
      x = (int)(floor(j * cos(angle)))+25;
      y = (int)(floor(j * sin(angle)))+25;
      grid[x][y] = 1;
      x = (int)(ceil(j * cos(angle)))+25;
      y = (int)(ceil(j * sin(angle)))+25;
      grid[x][y] = 1;
    }
    angle = angle + increment;
  }
  vector<int> new_situation;
  vector<float> new_situation_float;
  for(int i = 16; i < grid.size(); i++){
    for(int j = 0; j < grid[i].size(); j++){
      new_situation.push_back(grid[i][j]);
      new_situation_float.push_back((float)(grid[i][j]));
      cout << grid[i][j] << " ";
    }
    cout << endl;
  }
  vector<float> distances;
  for(int i = 0; i < situations.size(); i++){
    float dist = 0;
    for(int j = 0; j < new_situation.size(); j++){
      dist += abs(situations[i][j] - (float)(new_situation[j]));
    }
    cout << "Situation " << i << " " << situation_counts[i] << " dist : " << dist << endl;
    distances.push_back(dist);
  }
  int min_pos = distance(distances.begin(),min_element(distances.begin(),distances.end()));
  if(distances[min_pos] <= dist_cutoff){
    situation_observations.push_back(SituationMarker(ls, pose, min_pos, new_situation_float, actual_action));
    // situation_assignments.push_back(min_pos);
    // situation_laserscans.push_back(ls);
    if(add_to_existing){
      // cout << min_pos << " " << situation_counts[min_pos] << endl;
      vector<float> min_sit = situations[min_pos];
      vector<float> min_sit_counts;
      for(int i = 0; i < min_sit.size(); i++){
        // cout << (min_sit[i]*((float)(situation_counts[min_pos]))) << " " << ((float)(new_situation[i])) << " " << ((float)(situation_counts[min_pos]+1)) << " " << (((min_sit[i]*((float)(situation_counts[min_pos])))+((float)(new_situation[i])))/((float)(situation_counts[min_pos]+1))) << endl;
        min_sit_counts.push_back(((min_sit[i]*((float)(situation_counts[min_pos])))+((float)(new_situation[i])))/((float)(situation_counts[min_pos]+1)));
      }
      situations[min_pos] = min_sit_counts;
      situation_counts[min_pos] = situation_counts[min_pos]+1;
      // for(int i = 0; i < situation_counts.size(); i++){
      //   cout << "Situation " << i << " : " << situation_counts[i] << endl;
      // }
    }
  }
  // else if(add_to_existing){
  //   situation_assignments.push_back(situations.size());
  //   situation_laserscans.push_back(ls);
  //   situation_counts.push_back(1);
  //   situations.push_back(new_situation_float);
  // }
  else{
    situation_observations.push_back(SituationMarker(ls, pose, -1, new_situation_float, actual_action));
    // situation_assignments.push_back(-1);
    // situation_laserscans.push_back(ls);
  }
}


//----------------------//------------------------//


void FORRSituations::clusterOutlierObservations(){
  vector< vector<float> > action_grids_for_clustering;
  // vector<SituationMarker> observations_for_clustering;
  vector<int> observation_inds;
  for(int i = 0; i < situation_observations.size(); i++){
    if(situation_observations[i].assignment == -1){
      int matched_situation = identifySituationAssignment(situation_observations[i].action_grid);
      // cout << "Before " << i << " " << situation_observations[i].assignment << " " << matched_situation << endl;
      if(matched_situation != -1){
        situation_observations[i].assignment = matched_situation;
        // cout << "After " << i << " " << situation_observations[i].assignment << " " << matched_situation << endl;
      }
      else{
        // observations_for_clustering.push_back(situation_observations[i]);
        observation_inds.push_back(i);
        action_grids_for_clustering.push_back(situation_observations[i].action_grid);
      }
    }
  }
  cout << "Unclustered obs size: " << observation_inds.size() << " " << action_grids_for_clustering.size() << endl;
  if(action_grids_for_clustering.size() > 200){
    SpectralCluster clustering = SpectralCluster(15, 10, 50, 0.95);
    int situation_ind = situations.size();
    vector< vector<float> > results = clustering.cluster(action_grids_for_clustering);
    if(results.size() > 0){
      for(int i = 0; i < results.size(); i++){
        int count = (int)(results[i][0]);
        vector<float> values = results[i];
        values.erase(values.begin());
        createSituations(count, values);
      }
      vector<int> new_assignments = clustering.clusterAssignments();
      cout << "New assignments size: " << observation_inds.size() << " " << new_assignments.size() << " " << situation_ind << endl;
      for(int i = 0; i < observation_inds.size(); i++){
        int matched_situation = new_assignments[i];
        // cout << "Before " << i << " " << situation_observations[observation_inds[i]].assignment << " " << matched_situation << endl;
        if(matched_situation != -1){
          situation_observations[observation_inds[i]].assignment = matched_situation + situation_ind;
          // cout << "After " << i << " " << situation_observations[observation_inds[i]].assignment << " " << matched_situation + situation_ind << endl;
        }
      }
    }
    clustering.eraseFiles();
  }
}


//----------------------//------------------------//


vector<int> FORRSituations::identifySituation(sensor_msgs::LaserScan ls) {
  double angle = ls.angle_min;
  double increment = ls.angle_increment;
  vector<float> laser_ranges = ls.ranges;
  vector< vector<int> > grid;
  for(int i = 0; i < 51; i++){
    vector<int> col;
    for(int j = 0; j < 51; j++){
      col.push_back(0);
    }
    grid.push_back(col);
  }
  for(int i = 0; i < laser_ranges.size(); i++){
    for(double j = 0.0; j <= laser_ranges[i]; j+=0.9){
      int x = (int)(round(j * cos(angle)))+25;
      int y = (int)(round(j * sin(angle)))+25;
      grid[x][y] = 1;
      x = (int)(j * cos(angle))+25;
      y = (int)(j * sin(angle))+25;
      grid[x][y] = 1;
      x = (int)(floor(j * cos(angle)))+25;
      y = (int)(floor(j * sin(angle)))+25;
      grid[x][y] = 1;
      x = (int)(ceil(j * cos(angle)))+25;
      y = (int)(ceil(j * sin(angle)))+25;
      grid[x][y] = 1;
    }
    angle = angle + increment;
  }
  vector<int> new_situation;
  for(int i = 16; i < grid.size(); i++){
    for(int j = 0; j < grid[i].size(); j++){
      new_situation.push_back(grid[i][j]);
      // cout << grid[i][j] << " ";
    }
    // cout << endl;
  }
  vector<float> distances;
  for(int i = 0; i < situations.size(); i++){
    float dist = 0;
    for(int j = 0; j < new_situation.size(); j++){
      dist += abs(situations[i][j] - (float)(new_situation[j]));
    }
    // cout << dist << endl;
    distances.push_back(dist);
  }
  int min_pos = distance(distances.begin(),min_element(distances.begin(),distances.end()));
  // cout << min_pos << endl;
  vector<int> situation_median;
  for(int i = 0; i < situations[min_pos].size(); i++){
    if(situations[min_pos][i] >= 0.25){
      situation_median.push_back(1);
    }
    else{
      situation_median.push_back(0);
    }
  }
  return situation_median;
}


//----------------------//------------------------//


int FORRSituations::identifySituationAssignment(sensor_msgs::LaserScan ls) {
  double angle = ls.angle_min;
  double increment = ls.angle_increment;
  vector<float> laser_ranges = ls.ranges;
  vector< vector<int> > grid;
  for(int i = 0; i < 51; i++){
    vector<int> col;
    for(int j = 0; j < 51; j++){
      col.push_back(0);
    }
    grid.push_back(col);
  }
  for(int i = 0; i < laser_ranges.size(); i++){
    for(double j = 0.0; j <= laser_ranges[i]; j+=0.9){
      int x = (int)(round(j * cos(angle)))+25;
      int y = (int)(round(j * sin(angle)))+25;
      grid[x][y] = 1;
      x = (int)(j * cos(angle))+25;
      y = (int)(j * sin(angle))+25;
      grid[x][y] = 1;
      x = (int)(floor(j * cos(angle)))+25;
      y = (int)(floor(j * sin(angle)))+25;
      grid[x][y] = 1;
      x = (int)(ceil(j * cos(angle)))+25;
      y = (int)(ceil(j * sin(angle)))+25;
      grid[x][y] = 1;
    }
    angle = angle + increment;
  }
  vector<int> new_situation;
  for(int i = 16; i < grid.size(); i++){
    for(int j = 0; j < grid[i].size(); j++){
      new_situation.push_back(grid[i][j]);
      // cout << grid[i][j] << " ";
    }
    // cout << endl;
  }
  vector<float> distances;
  for(int i = 0; i < situations.size(); i++){
    float dist = 0;
    for(int j = 0; j < new_situation.size(); j++){
      dist += abs(situations[i][j] - (float)(new_situation[j]));
    }
    // cout << "Situation dist " << i << " : " << dist << endl;
    distances.push_back(dist);
  }
  int min_pos = distance(distances.begin(),min_element(distances.begin(),distances.end()));
  if(distances[min_pos] <= dist_cutoff){
    return min_pos;
  }
  else{
    return -1;
  }
}


//----------------------//------------------------//


int FORRSituations::identifySituationAssignment(vector<float> action_grid) {
  vector<float> distances;
  for(int i = 0; i < situations.size(); i++){
    float dist = 0;
    for(int j = 0; j < action_grid.size(); j++){
      dist += abs(situations[i][j] - action_grid[j]);
    }
    // cout << "Situation dist " << i << " : " << dist << endl;
    distances.push_back(dist);
  }
  int min_pos = distance(distances.begin(),min_element(distances.begin(),distances.end()));
  if(distances[min_pos] <= dist_cutoff){
    return min_pos;
  }
  else{
    return -1;
  }
}


//----------------------//------------------------//


void FORRSituations::learnSituationActions(AgentState *agentState, vector<TrailMarker> trail) {
  Position target(agentState->getCurrentTask()->getTaskX(),agentState->getCurrentTask()->getTaskY(),0);
  vector<double> target_distances;
  vector<double> target_angles;
  vector<FORRAction> trail_actions;
  vector<FORRAction> actual_actions;
  clusterOutlierObservations();
  vector<SituationMarker> current_situation_observations(situation_observations.end() - agentState->getCurrentTask()->getPositionHistory()->size(), situation_observations.end());
  cout << trail.size() << " " << current_situation_observations.size() << endl;
  vector<int> trail_situation_assignments;

  set<FORRAction> *action_set = agentState->getActionSet();
  set<FORRAction>::iterator actionIter;
  // for(int i = 0 ; i < pos_hist->size() ; i++){
  for(int i = 0 ; i < current_situation_observations.size() ; i++){
    for(int j = trail.size()-1; j >= 1; j--){
      vector<CartesianPoint> current_laser = agentState->transformToEndpoints(current_situation_observations[i].pose, current_situation_observations[i].ls);
      if(agentState->canAccessPoint(current_laser, CartesianPoint(current_situation_observations[i].pose.getX(), current_situation_observations[i].pose.getY()), trail[j].coordinates, 20)) {
        target_distances.push_back(current_situation_observations[i].pose.getDistance(target));
        double angle_to_target = atan2((target.getY() - current_situation_observations[i].pose.getY()), (target.getX() - current_situation_observations[i].pose.getX()));
        double required_rotation = angle_to_target - current_situation_observations[i].pose.getTheta();
        if(required_rotation > M_PI){
          required_rotation = required_rotation - (2 * M_PI);
        }
        if(required_rotation < -M_PI){
          required_rotation = required_rotation + (2 * M_PI);
        }
        target_angles.push_back(required_rotation);
        trail_situation_assignments.push_back(current_situation_observations[i].assignment);
        actual_actions.push_back(current_situation_observations[i].actual_action);
        // cout << "found furthest trail marker" << endl;
        std::map <FORRAction, double> result;
        typedef map<FORRAction, double>::iterator mapIt;
        for(actionIter = action_set->begin(); actionIter != action_set->end(); actionIter++){
          FORRAction forrAction = *actionIter;
          if(forrAction.type == PAUSE){
          // if(forrAction.type == PAUSE or forrAction.type == FORWARD){
            continue;
          }
          else{
            Position expectedPosition = agentState->getExpectedPositionAfterAction(forrAction, current_laser, current_situation_observations[i].pose);
            result[forrAction] = expectedPosition.getDistance(trail[j].coordinates.get_x(), trail[j].coordinates.get_y());
            // cout << forrAction.type << " " << forrAction.parameter << " " << expectedPosition.getDistance(trail[j].coordinates.get_x(), trail[j].coordinates.get_y()) << endl;
          }
        }
        double minDistanceToTrailMarker = 1000000;
        for(mapIt iterator = result.begin(); iterator != result.end(); iterator++){
          if(iterator->second < minDistanceToTrailMarker){
            minDistanceToTrailMarker = iterator->second;
          }
        }
        // cout << minDistanceToTrailMarker << endl;
        for(mapIt iterator = result.begin(); iterator != result.end(); iterator++){
          if(iterator->second == minDistanceToTrailMarker){
            // cout << iterator->first.type << " " << iterator->first.parameter << endl;
            trail_actions.push_back(iterator->first);
            break;
          }
        }
        break;
      }
    }
    // cout << target_distances.size() << " " << target_angles.size() << " " << trail_actions.size() << " " << trail_situation_assignments.size() << endl;
  }
  // cout << target_distances.size() << " " << target_angles.size() << " " << trail_actions.size() << " " << trail_situation_assignments.size() << endl;
  for(int i = 0; i < target_distances.size(); i++){
    // cout << "Action Assignment " << i << " : " << target_distances[i] << " " << target_angles[i] << " " << trail_actions[i].type << " " << trail_actions[i].parameter << " " << trail_situation_assignments[i] << endl;
    vector<int> assignment_values;
    assignment_values.push_back(trail_situation_assignments[i]);
    if(target_distances[i] <= 2){
      assignment_values.push_back(2);
    }
    else if(target_distances[i] <= 4){
      assignment_values.push_back(4);
    }
    else if(target_distances[i] <= 8){
      assignment_values.push_back(8);
    }
    else if(target_distances[i] <= 16){
      assignment_values.push_back(16);
    }
    else if(target_distances[i] <= 32){
      assignment_values.push_back(32);
    }
    else if(target_distances[i] <= 64){
      assignment_values.push_back(64);
    }
    else if(target_distances[i] <= 128){
      assignment_values.push_back(128);
    }
    else if(target_distances[i] <= 256){
      assignment_values.push_back(256);
    }

    if(target_angles[i] >= -M_PI/8.0 and target_angles[i] < M_PI/8.0){
      assignment_values.push_back(1);
    }
    else if(target_angles[i] >= M_PI/8.0 and target_angles[i] < 3.0*M_PI/8.0){
      assignment_values.push_back(2);
    }
    else if(target_angles[i] >= 3.0*M_PI/8.0 and target_angles[i] < 5.0*M_PI/8.0){
      assignment_values.push_back(3);
    }
    else if(target_angles[i] >= 5.0*M_PI/8.0 and target_angles[i] < 7.0*M_PI/8.0){
      assignment_values.push_back(4);
    }
    else if(target_angles[i] >= 7.0*M_PI/8.0 or target_angles[i] < -7.0*M_PI/8.0){
      assignment_values.push_back(5);
    }
    else if(target_angles[i] >= -7.0*M_PI/8.0 and target_angles[i] < -5.0*M_PI/8.0){
      assignment_values.push_back(6);
    }
    else if(target_angles[i] >= -5.0*M_PI/8.0 and target_angles[i] < -3.0*M_PI/8.0){
      assignment_values.push_back(7);
    }
    else if(target_angles[i] >= -3.0*M_PI/8.0 and target_angles[i] < -M_PI/8.0){
      assignment_values.push_back(8);
    }
    if(assignment_values.size() == 3){
      if(action_assignments.count(assignment_values) == 0){
        action_assignments.insert(pair< vector<int>, vector<FORRAction> >(assignment_values, vector<FORRAction>()));
        accuracy_action_assignments.insert(pair< vector<int>, vector<double> >(assignment_values, vector<double>()));
      }
      // else{
      //   for(int j = 0; j < action_assignments[assignment_values].size(); j++){
      //     // cout << "Action Prediction " << i << " : " << action_assignments[assignment_values][j].type << " " << action_assignments[assignment_values][j].parameter << endl;
      //   }
      // }
      action_assignments[assignment_values].push_back(trail_actions[i]);
      if(trail_actions[i] == actual_actions[i]){
        accuracy_action_assignments[assignment_values].push_back(1.0);
      }
      else if(trail_actions[i].type == actual_actions[i].type){
        accuracy_action_assignments[assignment_values].push_back(0.5);
      }
      else{
        accuracy_action_assignments[assignment_values].push_back(0.0);
      }
    }
  }
  action_assignment_combinations.clear();
  map< vector<int>, vector<FORRAction> >::iterator aait;
  for(aait = action_assignments.begin(); aait != action_assignments.end(); aait++){
    map< FORRAction, double > action_weights;
    map< FORRAction, double >::iterator awit;
    set<FORRAction> *action_set = agentState->getActionSet();
    set<FORRAction>::iterator actionIter;
    for(actionIter = action_set->begin(); actionIter != action_set->end(); actionIter++){
      FORRAction forrAction = *actionIter;
      action_weights[forrAction] = 1.0;
    }
    for(int j = 0; j < aait->second.size(); j++){
      action_weights[aait->second[j]] += 1.0;
    }
    double max_value = 0.0;
    for(awit = action_weights.begin(); awit != action_weights.end(); awit++){
      // cout << awit->first.type << " " << awit->first.parameter << " " << awit->second << endl;
      if(awit->second > max_value){
        max_value = awit->second;
      }
    }

    for(awit = action_weights.begin(); awit != action_weights.end(); awit++){
      // cout << aait->first[0] << " " << aait->first[1] << " " << aait->first[2] << " " << awit->first.type << " " << awit->first.parameter << " " << awit->second-1 << endl;
      vector<double> assignment_counts;
      assignment_counts.push_back(aait->first[0]);
      assignment_counts.push_back(aait->first[1]);
      assignment_counts.push_back(aait->first[2]);
      assignment_counts.push_back(awit->first.type);
      assignment_counts.push_back(awit->first.parameter);
      assignment_counts.push_back(awit->second-1);
      action_assignment_combinations.push_back(assignment_counts);
      awit->second = awit->second / max_value;
    }
    action_assignment_weights[aait->first] = action_weights;
  }
  // cout << "Finished action assignment" << endl;
  // printSituations();
}


//----------------------//------------------------//


double FORRSituations::getWeightForAction(AgentState *agentState, FORRAction action){
  Position target(agentState->getCurrentTask()->getTaskX(),agentState->getCurrentTask()->getTaskY(),0);
  Position curr = agentState->getCurrentPosition();
  double target_dist = curr.getDistance(target);

  double angle_to_target = atan2((target.getY() - curr.getY()), (target.getX() - curr.getX()));
  double required_rotation = angle_to_target - curr.getTheta();
  if(required_rotation > M_PI){
    required_rotation = required_rotation - (2 * M_PI);
  }
  if(required_rotation < -M_PI){
    required_rotation = required_rotation + (2 * M_PI);
  }
  double target_angle = required_rotation;
  int sit_assignment = identifySituationAssignment(agentState->getCurrentLaserScan());
  vector<int> assignment_values;
  assignment_values.push_back(sit_assignment);
  if(target_dist <= 2){
    assignment_values.push_back(2);
  }
  else if(target_dist <= 4){
    assignment_values.push_back(4);
  }
  else if(target_dist <= 8){
    assignment_values.push_back(8);
  }
  else if(target_dist <= 16){
    assignment_values.push_back(16);
  }
  else if(target_dist <= 32){
    assignment_values.push_back(32);
  }
  else if(target_dist <= 64){
    assignment_values.push_back(64);
  }
  else if(target_dist <= 128){
    assignment_values.push_back(128);
  }
  else if(target_dist <= 256){
    assignment_values.push_back(256);
  }

  if(target_angle >= -M_PI/8.0 and target_angle < M_PI/8.0){
    assignment_values.push_back(1);
  }
  else if(target_angle >= M_PI/8.0 and target_angle < 3.0*M_PI/8.0){
    assignment_values.push_back(2);
  }
  else if(target_angle >= 3.0*M_PI/8.0 and target_angle < 5.0*M_PI/8.0){
    assignment_values.push_back(3);
  }
  else if(target_angle >= 5.0*M_PI/8.0 and target_angle < 7.0*M_PI/8.0){
    assignment_values.push_back(4);
  }
  else if(target_angle >= 7.0*M_PI/8.0 or target_angle < -7.0*M_PI/8.0){
    assignment_values.push_back(5);
  }
  else if(target_angle >= -7.0*M_PI/8.0 and target_angle < -5.0*M_PI/8.0){
    assignment_values.push_back(6);
  }
  else if(target_angle >= -5.0*M_PI/8.0 and target_angle < -3.0*M_PI/8.0){
    assignment_values.push_back(7);
  }
  else if(target_angle >= -3.0*M_PI/8.0 and target_angle < -M_PI/8.0){
    assignment_values.push_back(8);
  }
  if(assignment_values.size() == 3){
    if(action_assignments.count(assignment_values) == 0){
      return 1.0;
    }
    else{
      return action_assignment_weights[assignment_values][action];
    }
  }
  return 1.0;
}


//----------------------//------------------------//


double FORRSituations::getAccuracyForSituation(AgentState *agentState){
  Position target(agentState->getCurrentTask()->getTaskX(),agentState->getCurrentTask()->getTaskY(),0);
  Position curr = agentState->getCurrentPosition();
  double target_dist = curr.getDistance(target);

  double angle_to_target = atan2((target.getY() - curr.getY()), (target.getX() - curr.getX()));
  double required_rotation = angle_to_target - curr.getTheta();
  if(required_rotation > M_PI){
    required_rotation = required_rotation - (2 * M_PI);
  }
  if(required_rotation < -M_PI){
    required_rotation = required_rotation + (2 * M_PI);
  }
  double target_angle = required_rotation;
  int sit_assignment = identifySituationAssignment(agentState->getCurrentLaserScan());
  vector<int> assignment_values;
  assignment_values.push_back(sit_assignment);
  if(target_dist <= 2){
    assignment_values.push_back(2);
  }
  else if(target_dist <= 4){
    assignment_values.push_back(4);
  }
  else if(target_dist <= 8){
    assignment_values.push_back(8);
  }
  else if(target_dist <= 16){
    assignment_values.push_back(16);
  }
  else if(target_dist <= 32){
    assignment_values.push_back(32);
  }
  else if(target_dist <= 64){
    assignment_values.push_back(64);
  }
  else if(target_dist <= 128){
    assignment_values.push_back(128);
  }
  else if(target_dist <= 256){
    assignment_values.push_back(256);
  }

  if(target_angle >= -M_PI/8.0 and target_angle < M_PI/8.0){
    assignment_values.push_back(1);
  }
  else if(target_angle >= M_PI/8.0 and target_angle < 3.0*M_PI/8.0){
    assignment_values.push_back(2);
  }
  else if(target_angle >= 3.0*M_PI/8.0 and target_angle < 5.0*M_PI/8.0){
    assignment_values.push_back(3);
  }
  else if(target_angle >= 5.0*M_PI/8.0 and target_angle < 7.0*M_PI/8.0){
    assignment_values.push_back(4);
  }
  else if(target_angle >= 7.0*M_PI/8.0 or target_angle < -7.0*M_PI/8.0){
    assignment_values.push_back(5);
  }
  else if(target_angle >= -7.0*M_PI/8.0 and target_angle < -5.0*M_PI/8.0){
    assignment_values.push_back(6);
  }
  else if(target_angle >= -5.0*M_PI/8.0 and target_angle < -3.0*M_PI/8.0){
    assignment_values.push_back(7);
  }
  else if(target_angle >= -3.0*M_PI/8.0 and target_angle < -M_PI/8.0){
    assignment_values.push_back(8);
  }
  if(assignment_values.size() == 3){
    if(action_assignments.count(assignment_values) == 0){
      return 0.0;
    }
    else{
      double num_instances = accuracy_action_assignments[assignment_values].size();
      double sum_of_instances = accumulate(accuracy_action_assignments[assignment_values].begin(), accuracy_action_assignments[assignment_values].end(), 0.0);
      return sum_of_instances/num_instances;
    }
  }
  return 0.0;
}