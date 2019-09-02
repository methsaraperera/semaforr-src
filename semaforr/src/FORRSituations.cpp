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
  vector<float> min_sit = situations[min_pos];
  vector<float> min_sit_counts;
  for(int i = 0; i < min_sit.size(); i++){
    // cout << (min_sit[i]*((float)(situation_counts[min_pos]))) << " " << ((float)(new_situation[i])) << " " << ((float)(situation_counts[min_pos]+1)) << " " << (((min_sit[i]*((float)(situation_counts[min_pos])))+((float)(new_situation[i])))/((float)(situation_counts[min_pos]+1))) << endl;
    min_sit_counts.push_back(((min_sit[i]*((float)(situation_counts[min_pos])))+((float)(new_situation[i])))/((float)(situation_counts[min_pos]+1)));
  }
  situations[min_pos] = min_sit_counts;
  situation_counts[min_pos] = situation_counts[min_pos]+1;
  for(int i = 0; i < situation_counts.size(); i++){
    cout << "Situation " << i << " : " << situation_counts[i] << endl;
  }
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
        target_angles.push_back(atan2(((*pos_hist)[i].getY() - target.getY()), ((*pos_hist)[i].getX() - target.getX())));
        // cout << "found furthest trail marker" << endl;
        std::map <FORRAction, double> result;
        typedef map<FORRAction, double>::iterator mapIt;
        for(actionIter = action_set->begin(); actionIter != action_set->end(); actionIter++){
          FORRAction forrAction = *actionIter;
          Position expectedPosition = agentState->getExpectedPositionAfterAction(forrAction, (*laser_hist)[i], (*pos_hist)[i]);
          result[forrAction] = expectedPosition.getDistance(trail[j].coordinates.get_x(), trail[j].coordinates.get_y());
          // cout << forrAction.type << " " << forrAction.parameter << " " << expectedPosition.getDistance(trail[j].coordinates.get_x(), trail[j].coordinates.get_y()) << endl;
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
  }
}