#include<FORRSituations.h>

using namespace std;



//----------------------//------------------------//


void FORRSituations::addObservationToSituations(sensor_msgs::LaserScan ls) {
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
    cout << "Situation dist " << i << " : " << dist << endl;
    distances.push_back(dist);
  }
  int min_pos = distance(distances.begin(),min_element(distances.begin(),distances.end()));
  // cout << min_pos << " " << situation_counts[min_pos] << endl;
  // vector<float> min_sit = situations[min_pos];
  // vector<float> min_sit_counts;
  // for(int i = 0; i < min_sit.size(); i++){
  //   // cout << (min_sit[i]*((float)(situation_counts[min_pos]))) << " " << ((float)(new_situation[i])) << " " << ((float)(situation_counts[min_pos]+1)) << " " << (((min_sit[i]*((float)(situation_counts[min_pos])))+((float)(new_situation[i])))/((float)(situation_counts[min_pos]+1))) << endl;
  //   min_sit_counts.push_back(((min_sit[i]*((float)(situation_counts[min_pos])))+((float)(new_situation[i])))/((float)(situation_counts[min_pos]+1)));
  // }
  // situations[min_pos] = min_sit_counts;
  // situation_counts[min_pos] = situation_counts[min_pos]+1;
  // for(int i = 0; i < situation_counts.size(); i++){
  //   cout << "Situation " << i << " : " << situation_counts[i] << endl;
  // }
  situation_assignments.push_back(min_pos);
}


//----------------------//------------------------//


void FORRSituations::updateSituations() {
  clearAllSituations();
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
    cout << dist << endl;
    distances.push_back(dist);
  }
  int min_pos = distance(distances.begin(),min_element(distances.begin(),distances.end()));
  cout << min_pos << endl;
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


void FORRSituations::learnSituationActions(AgentState *agentState, double x, double y, vector<Position> *pos_hist, vector< vector<CartesianPoint> > *laser_hist, vector<TrailMarker> trail) {
  Position target(x,y,0);
  vector<double> target_distances;
  vector<double> target_angles;
  vector<FORRAction> trail_actions;
  vector<int> current_situation_assignments(situation_assignments.end() - pos_hist->size(), situation_assignments.end());
  // cout << trail.size() << endl;

  set<FORRAction> *action_set = agentState->getActionSet();
  set<FORRAction>::iterator actionIter;
  for(int i = 0 ; i < pos_hist->size() ; i++){
    for(int j = trail.size()-1; j >= 1; j--){
      if(agentState->canAccessPoint((*laser_hist)[i], CartesianPoint((*pos_hist)[i].getX(), (*pos_hist)[i].getY()), trail[j].coordinates, 20)) {
        target_distances.push_back((*pos_hist)[i].getDistance(target));
        double angle_to_target = atan2((target.getY() - (*pos_hist)[i].getY()), (target.getX() - (*pos_hist)[i].getX()));
        double required_rotation = angle_to_target - (*pos_hist)[i].getTheta();
        if(required_rotation > M_PI){
          required_rotation = required_rotation - (2 * M_PI);
        }
        if(required_rotation < -M_PI){
          required_rotation = required_rotation + (2 * M_PI);
        }
        target_angles.push_back(required_rotation);
        // cout << "found furthest trail marker" << endl;
        std::map <FORRAction, double> result;
        typedef map<FORRAction, double>::iterator mapIt;
        for(actionIter = action_set->begin(); actionIter != action_set->end(); actionIter++){
          FORRAction forrAction = *actionIter;
          if(forrAction.type == PAUSE){
            continue;
          }
          else{
            Position expectedPosition = agentState->getExpectedPositionAfterAction(forrAction, (*laser_hist)[i], (*pos_hist)[i]);
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
    // cout << target_distances.size() << " " << target_angles.size() << " " << trail_actions.size() << " " << current_situation_assignments.size() << endl;
  }
  // cout << target_distances.size() << " " << target_angles.size() << " " << trail_actions.size() << " " << current_situation_assignments.size() << endl;
  for(int i = 0; i < target_distances.size(); i++){
    cout << "Action Assignment " << i << " : " << target_distances[i] << " " << target_angles[i] << " " << trail_actions[i].type << " " << trail_actions[i].parameter << " " << current_situation_assignments[i] << endl;
    vector<int> assignment_values;
    assignment_values.push_back(current_situation_assignments[i]);
    if(target_distances[i] <= 1){
      assignment_values.push_back(1);
    }
    else if(target_distances[i] <= 2){
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
          cout << "Action Prediction " << i << " : " << action_assignments[assignment_values][j].type << " " << action_assignments[assignment_values][j].parameter << endl;
        }
      }
      action_assignments[assignment_values].push_back(trail_actions[i]);
    }
  }
}