#include<FORRSituations.h>

using namespace std;



//----------------------//------------------------//


void FORRSituations::addObservationToSituations(sensor_msgs::LaserScan ls, Position pose, bool add_to_existing) {
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
    situation_observations.push_back(SituationMarker(ls, pose, min_pos, new_situation_float));
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
    situation_observations.push_back(SituationMarker(ls, pose, -1, new_situation_float));
    // situation_assignments.push_back(-1);
    // situation_laserscans.push_back(ls);
  }
}


//----------------------//------------------------//


void FORRSituations::clusterOutlierObservations(){
  vector< vector<float> > action_grids_for_clustering;
  vector<SituationMarker> observations_for_clustering;
  vector<int> observation_inds;
  for(int i = 0; i < situation_observations.size(); i++){
    if(situation_observations[i].assignment == -1){
      int matched_situation = identifySituationAssignment(situation_observations[i].ls);
      cout << i << " " << situation_observations[i].assignment << " " << matched_situation << endl;
      if(matched_situation != -1){
        situation_observations[i].assignment = matched_situation;
        cout << i << " " << situation_observations[i].assignment << " " << matched_situation << endl;
      }
      else{
        observations_for_clustering.push_back(situation_observations[i]);
        observation_inds.push_back(i);
        action_grids_for_clustering.push_back(situation_observations[i].action_grid);
      }
    }
  }
  cout << observations_for_clustering.size() << " " << observation_inds.size() << " " << action_grids_for_clustering.size() << endl;
  SpectralCluster clustering = SpectralCluster(15, 15, 0.95);
  vector<int> results = clustering.cluster(action_grids_for_clustering);
}


//----------------------//------------------------//


void FORRSituations::updateSituations(AgentState *agentState, std_msgs::String sits, vector< Position > *position_hist, vector< vector<CartesianPoint> > *laser_hist, vector< sensor_msgs::LaserScan > ls_hist, vector< vector< TrailMarker> > trails) {
  cout << "In update situations" << endl;
  clearAllSituations();
  cout << "Cleared situations" << endl;
  string input_data = sits.data;
  // cout << sits.data << endl;
  std::istringstream iss(input_data);
  for(std::string line; std::getline(iss, line); ){
    // cout << line << endl;
    std::stringstream ss(line);
    std::istream_iterator<std::string> begin(ss);
    std::istream_iterator<std::string> end;
    std::vector<std::string> vstrings(begin, end);
    int count = atoi(vstrings[0].c_str());
    // cout << count << endl;
    vector<float> values;
    for (int i=1; i<vstrings.size(); i++){
      // cout << atof(vstrings[i].c_str()) << endl;
      values.push_back(atof(vstrings[i].c_str()));
    }
    createSituations(count, values);
    // cout << "Situation: " << count << endl;
  }
  list<Task*> agenda = agentState->getAllAgenda();
  cout << position_hist->size() << " " << laser_hist->size() << " " << ls_hist.size() << " " << trails.size() << " " << agenda.size() << endl;
  for(int i = 0; i < ls_hist.size(); i++){
    addObservationToSituations(ls_hist[i], (*position_hist)[i], false);
  }
  vector<int> decision_count = agentState->getTaskDecisionCount();
  int i = 0;
  int j = 0;
  for(list<Task*>::iterator it = agenda.begin(); it != agenda.end(); it++){
    double x = (*it)->getTaskX();
    double y = (*it)->getTaskY();
    int task_count = decision_count[i];
    // cout << i << " " << j << " " << task_count << endl;
    vector< Position > *pos_dec = new vector<Position>;
    vector< vector<CartesianPoint> > *las_dec = new vector< vector<CartesianPoint> >;
    for(int k = j; k < j+task_count; k++){
      // cout << k << endl;
      pos_dec->push_back((*position_hist)[k]);
      las_dec->push_back((*laser_hist)[k]);
    }
    // cout << pos_dec->size() << " " << las_dec->size() << endl;
    // learnSituationActions(agentState, x, y, pos_dec, las_dec, trails[i], j, j+task_count);
    i++;
    j = j + task_count;
    if(i == trails.size()){
      break;
    }
  }
  cout << "Finished updateSituations" << endl;
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


void FORRSituations::learnSituationActions(AgentState *agentState, vector<TrailMarker> trail, int begin_vec, int end_vec) {
  Position target(agentState->getCurrentTask()->getTaskX(),agentState->getCurrentTask()->getTaskY(),0);
  vector<double> target_distances;
  vector<double> target_angles;
  vector<FORRAction> trail_actions;
  vector<SituationMarker> current_situation_observations;
  // vector<int> current_situation_assignments;
  // cout << begin_vec << " " << end_vec << endl;
  if(begin_vec == -1 and end_vec == -1){
    vector<SituationMarker> c_situation_observations(situation_observations.end() - agentState->getCurrentTask()->getPositionHistory()->size(), situation_observations.end());
    current_situation_observations = c_situation_observations;
    // vector< sensor_msgs::LaserScan > *laser_scan_hist = agentState->getCurrentTask()->getLaserScanHistory();
    // for(int i = 0; i < laser_scan_hist->size(); i++){
    //   current_situation_assignments.push_back(identifySituationAssignment((*laser_scan_hist)[i]));
    // }
  }
  // else{
  //   vector<int> c_situation_assignments(situation_assignments.begin() + begin_vec, situation_assignments.begin() + end_vec);
  //   // cout << c_situation_assignments.size() << endl;
  //   current_situation_assignments = c_situation_assignments;
  // }
  // cout << pos_hist->size() << " " << laser_hist->size() << " " << trail.size() << " " << current_situation_assignments.size() << endl;
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
      }
      else{
        for(int j = 0; j < action_assignments[assignment_values].size(); j++){
          // cout << "Action Prediction " << i << " : " << action_assignments[assignment_values][j].type << " " << action_assignments[assignment_values][j].parameter << endl;
        }
      }
      action_assignments[assignment_values].push_back(trail_actions[i]);
    }
  }

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
      cout << aait->first[0] << " " << aait->first[1] << " " << aait->first[2] << " " << awit->first.type << " " << awit->first.parameter << " " << awit->second-1 << endl;
      awit->second = awit->second / max_value;
    }
    action_assignment_weights[aait->first] = action_weights;
  }
  // cout << "Finished action assignment" << endl;
  // printSituations();
  clusterOutlierObservations();
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